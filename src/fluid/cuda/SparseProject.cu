#include "fluid/cuda/SparseProject.h"

#include <cmath>

#include <cub/device/device_reduce.cuh>

#include "fluid/cuda/CudaCheck.h"
#include "fluid/cuda/SparseGrid.cuh"

namespace Fluid
{
    namespace
    {
        constexpr int kDq = 0;
        constexpr int kRrOld = 1;
        constexpr int kRrNew = 2;
        constexpr int kScalarCount = 3;
        constexpr int kWarpSize = 32;
        constexpr int kSubcellThreads = kSubcellDim * kSubcellDim * kSubcellDim;

        const dim3 kSubcellBlock(kSubcellDim, kSubcellDim, kSubcellDim);

        int SubcellCount(const SparseTopology& topology)
        {
            return (topology.BrickCapacity() > 0 ? topology.BrickCapacity() : 1) * kSubcellsPerBrick;
        }

        template <typename T>
        __device__ T Load(cudaSurfaceObject_t surface, int3 texel)
        {
            return surf3Dread<T>(surface, texel.x * static_cast<int>(sizeof(T)), texel.y, texel.z);
        }

        template <typename T>
        __device__ void Store(cudaSurfaceObject_t surface, int3 texel, T value)
        {
            surf3Dwrite(value, surface, texel.x * static_cast<int>(sizeof(T)), texel.y, texel.z);
        }

        __device__ int3 Offset(int3 texel, int axis, int side)
        {
            return make_int3(texel.x + (axis == 0 ? side : 0),
                             texel.y + (axis == 1 ? side : 0),
                             texel.z + (axis == 2 ? side : 0));
        }

        __device__ int Lane()
        {
            return threadIdx.x + kSubcellDim * (threadIdx.y + kSubcellDim * threadIdx.z);
        }

        __device__ bool SubcellTexel(const int* brickNodes, int3& texel)
        {
            const int subcell = blockIdx.x;
            const int brick = subcell / kSubcellsPerBrick;
            if (brickNodes[brick] == kNullNode)
                return false;
            const int s = subcell % kSubcellsPerBrick;
            texel = AtlasTexel(brick,
                               make_int3((s % kSubcellsPerAxis) * kSubcellDim + threadIdx.x,
                                         ((s / kSubcellsPerAxis) % kSubcellsPerAxis) * kSubcellDim + threadIdx.y,
                                         (s / (kSubcellsPerAxis * kSubcellsPerAxis)) * kSubcellDim + threadIdx.z));
            return true;
        }

        __device__ void SubcellSum(double value, double* partials)
        {
            __shared__ double warpSums[kSubcellThreads / kWarpSize];
            for (int offset = kWarpSize / 2; offset > 0; offset >>= 1)
                value += __shfl_down_sync(0xffffffffu, value, offset);
            const int lane = Lane();
            if (lane % kWarpSize == 0)
                warpSums[lane / kWarpSize] = value;
            __syncthreads();
            if (lane == 0)
            {
                double sum = 0.0;
                for (int w = 0; w < kSubcellThreads / kWarpSize; ++w)
                    sum += warpSums[w];
                partials[blockIdx.x] = sum;
            }
        }

        __global__ void ApplyForcesKernel(const int* brickNodes,
                                          cudaSurfaceObject_t markers,
                                          cudaSurfaceObject_t velocity,
                                          float3 add)
        {
            int3 texel;
            if (!SubcellTexel(brickNodes, texel))
                return;
            const bool solid = Load<unsigned char>(markers, texel) == kCellSolid;
            float4 v = Load<float4>(velocity, texel);
            const bool solidX = solid || Load<unsigned char>(markers, Offset(texel, 0, -1)) == kCellSolid;
            const bool solidY = solid || Load<unsigned char>(markers, Offset(texel, 1, -1)) == kCellSolid;
            const bool solidZ = solid || Load<unsigned char>(markers, Offset(texel, 2, -1)) == kCellSolid;
            v.x = solidX ? 0.0f : v.x + add.x;
            v.y = solidY ? 0.0f : v.y + add.y;
            v.z = solidZ ? 0.0f : v.z + add.z;
            Store(velocity, texel, v);
        }

        __global__ void DivergenceKernel(const int* brickNodes,
                                         cudaSurfaceObject_t markers,
                                         cudaSurfaceObject_t velocity,
                                         cudaSurfaceObject_t divergence,
                                         float h)
        {
            int3 texel;
            if (!SubcellTexel(brickNodes, texel))
                return;
            float b = 0.0f;
            if (Load<unsigned char>(markers, texel) == kCellFluid)
            {
                const float4 v = Load<float4>(velocity, texel);
                const float sum = Load<float4>(velocity, Offset(texel, 0, 1)).x - v.x +
                                  Load<float4>(velocity, Offset(texel, 1, 1)).y - v.y +
                                  Load<float4>(velocity, Offset(texel, 2, 1)).z - v.z;
                b = -h * sum;
            }
            Store(divergence, texel, b);
        }

        __global__ void InitKernel(const int* brickNodes,
                                   cudaSurfaceObject_t markers,
                                   cudaSurfaceObject_t divergence,
                                   cudaSurfaceObject_t x,
                                   cudaSurfaceObject_t r,
                                   cudaSurfaceObject_t d,
                                   double* partials)
        {
            int3 texel;
            if (!SubcellTexel(brickNodes, texel))
            {
                if (Lane() == 0)
                    partials[blockIdx.x] = 0.0;
                return;
            }
            const float value =
                Load<unsigned char>(markers, texel) == kCellFluid ? Load<float>(divergence, texel) : 0.0f;
            Store(x, texel, 0.0f);
            Store(r, texel, value);
            Store(d, texel, value);
            SubcellSum(static_cast<double>(value) * value, partials);
        }

        __global__ void SpMVKernel(const int* brickNodes,
                                   cudaSurfaceObject_t markers,
                                   cudaSurfaceObject_t d,
                                   cudaSurfaceObject_t q,
                                   double* partials)
        {
            int3 texel;
            if (!SubcellTexel(brickNodes, texel))
            {
                if (Lane() == 0)
                    partials[blockIdx.x] = 0.0;
                return;
            }
            float result = 0.0f;
            double dq = 0.0;
            if (Load<unsigned char>(markers, texel) == kCellFluid)
            {
                int nonSolid = 0;
                float neighbors = 0.0f;
                for (int axis = 0; axis < 3; ++axis)
                {
                    for (int side = -1; side <= 1; side += 2)
                    {
                        const int3 n = Offset(texel, axis, side);
                        const unsigned char marker = Load<unsigned char>(markers, n);
                        if (marker == kCellSolid)
                            continue;
                        ++nonSolid;
                        if (marker == kCellFluid)
                            neighbors += Load<float>(d, n);
                    }
                }
                const float center = Load<float>(d, texel);
                result = nonSolid * center - neighbors;
                dq = static_cast<double>(center) * result;
            }
            Store(q, texel, result);
            SubcellSum(dq, partials);
        }

        __global__ void UpdateXRKernel(const int* brickNodes,
                                       cudaSurfaceObject_t d,
                                       cudaSurfaceObject_t q,
                                       cudaSurfaceObject_t x,
                                       cudaSurfaceObject_t r,
                                       const double* scalars,
                                       double* partials)
        {
            int3 texel;
            if (!SubcellTexel(brickNodes, texel))
            {
                if (Lane() == 0)
                    partials[blockIdx.x] = 0.0;
                return;
            }
            const double dq = scalars[kDq];
            const float alpha = dq > 0.0 ? static_cast<float>(scalars[kRrOld] / dq) : 0.0f;
            Store(x, texel, Load<float>(x, texel) + alpha * Load<float>(d, texel));
            const float value = Load<float>(r, texel) - alpha * Load<float>(q, texel);
            Store(r, texel, value);
            SubcellSum(static_cast<double>(value) * value, partials);
        }

        __global__ void UpdateDKernel(const int* brickNodes,
                                      cudaSurfaceObject_t r,
                                      cudaSurfaceObject_t d,
                                      const double* scalars)
        {
            int3 texel;
            if (!SubcellTexel(brickNodes, texel))
                return;
            const double rrOld = scalars[kRrOld];
            const float beta = rrOld > 0.0 ? static_cast<float>(scalars[kRrNew] / rrOld) : 0.0f;
            Store(d, texel, Load<float>(r, texel) + beta * Load<float>(d, texel));
        }

        __global__ void RollKernel(double* scalars)
        {
            scalars[kRrOld] = scalars[kRrNew];
            scalars[kRrNew] = 0.0;
            scalars[kDq] = 0.0;
        }

        __global__ void PressureToVelocityKernel(const int* brickNodes,
                                                 cudaSurfaceObject_t markers,
                                                 cudaSurfaceObject_t pressure,
                                                 cudaSurfaceObject_t velocity,
                                                 float h)
        {
            int3 texel;
            if (!SubcellTexel(brickNodes, texel))
                return;
            const unsigned char marker = Load<unsigned char>(markers, texel);
            const float p = Load<float>(pressure, texel);
            float4 v = Load<float4>(velocity, texel);
            float* components[3] = {&v.x, &v.y, &v.z};
            for (int axis = 0; axis < 3; ++axis)
            {
                const int3 left = Offset(texel, axis, -1);
                const unsigned char leftMarker = Load<unsigned char>(markers, left);
                if (marker == kCellSolid || leftMarker == kCellSolid)
                    *components[axis] = 0.0f;
                else if (marker == kCellFluid || leftMarker == kCellFluid)
                    *components[axis] -= (p - Load<float>(pressure, left)) / h;
            }
            Store(velocity, texel, v);
        }

        void BeginStage(GpuTimer* timer, const char* name)
        {
            if (timer)
                timer->Begin(name);
        }

        void EndStage(GpuTimer* timer)
        {
            if (timer)
                timer->End();
        }
    }

    SparseCg::~SparseCg()
    {
        Release();
    }

    void SparseCg::Release()
    {
        if (m_graph)
            CUDA_CHECK(cudaGraphExecDestroy(m_graph));
        if (m_stream)
            CUDA_CHECK(cudaStreamDestroy(m_stream));
        m_graph = nullptr;
        m_stream = nullptr;
        m_partials.Free();
        m_scalars.Free();
        m_temp.Free();
        m_tempBytes = 0;
    }

    bool SparseCg::Prepare(int subcellCount)
    {
        if (!m_stream && !CUDA_CHECK(cudaStreamCreate(&m_stream)))
            return false;
        if (!m_partials.Resize(subcellCount) || !m_scalars.Resize(kScalarCount))
            return false;
        m_tempBytes = 0;
        return CUDA_CHECK(cub::DeviceReduce::Sum(
                   nullptr, m_tempBytes, m_partials.Data(), m_scalars.Data(), subcellCount, m_stream)) &&
               m_temp.Resize(m_tempBytes > 0 ? m_tempBytes : 1);
    }

    bool SparseCg::EnqueueSum(int subcellCount, int slot)
    {
        size_t bytes = m_tempBytes;
        return CUDA_CHECK(cub::DeviceReduce::Sum(
            m_temp.Data(), bytes, m_partials.Data(), m_scalars.Data() + slot, subcellCount, m_stream));
    }

    bool SparseCg::EnqueueIteration(BrickAtlas& atlas, const SparseTopology& topology, int subcellCount)
    {
        const int* brickNodes = atlas.BrickNodes();
        const cudaSurfaceObject_t markers = atlas.Surface(AtlasChannel::Marker);
        const cudaSurfaceObject_t x = atlas.Surface(AtlasChannel::Pressure);
        const cudaSurfaceObject_t r = atlas.Surface(AtlasChannel::R);
        const cudaSurfaceObject_t d = atlas.Surface(AtlasChannel::D);
        const cudaSurfaceObject_t q = atlas.Surface(AtlasChannel::Q);

        SpMVKernel<<<subcellCount, kSubcellBlock, 0, m_stream>>>(brickNodes, markers, d, q, m_partials.Data());
        if (!EnqueueSum(subcellCount, kDq))
            return false;
        UpdateXRKernel<<<subcellCount, kSubcellBlock, 0, m_stream>>>(
            brickNodes, d, q, x, r, m_scalars.Data(), m_partials.Data());
        if (!EnqueueSum(subcellCount, kRrNew))
            return false;
        UpdateDKernel<<<subcellCount, kSubcellBlock, 0, m_stream>>>(brickNodes, r, d, m_scalars.Data());
        if (!atlas.UpdateApron(AtlasChannel::D, topology, 0.0f, m_stream))
            return false;
        RollKernel<<<1, 1, 0, m_stream>>>(m_scalars.Data());
        return CUDA_CHECK(cudaGetLastError());
    }

    bool SparseCg::BuildGraph(BrickAtlas& atlas, const SparseTopology& topology, int subcellCount)
    {
        if (!CUDA_CHECK(cudaStreamBeginCapture(m_stream, cudaStreamCaptureModeThreadLocal)))
            return false;
        bool enqueued = true;
        for (int i = 0; i < kIterationsPerGraph && enqueued; ++i)
            enqueued = EnqueueIteration(atlas, topology, subcellCount);
        cudaGraph_t graph = nullptr;
        const bool captured = CUDA_CHECK(cudaStreamEndCapture(m_stream, &graph)) && enqueued;
        if (!captured)
        {
            if (graph)
                cudaGraphDestroy(graph);
            return false;
        }

        if (m_graph)
        {
            cudaGraphExecUpdateResultInfo info{};
            if (cudaGraphExecUpdate(m_graph, graph, &info) != cudaSuccess)
            {
                cudaGetLastError();
                CUDA_CHECK(cudaGraphExecDestroy(m_graph));
                m_graph = nullptr;
            }
        }
        bool ok = true;
        if (!m_graph)
            ok = CUDA_CHECK(cudaGraphInstantiate(&m_graph, graph, 0));
        CUDA_CHECK(cudaGraphDestroy(graph));
        if (!ok)
            m_graph = nullptr;
        return ok;
    }

    bool SparseCg::ReadScalar(int slot, double& value)
    {
        return CUDA_CHECK(cudaMemcpyAsync(
                   &value, m_scalars.Data() + slot, sizeof(double), cudaMemcpyDeviceToHost, m_stream)) &&
               CUDA_CHECK(cudaStreamSynchronize(m_stream));
    }

    CgResult SparseCg::Solve(BrickAtlas& atlas, const SparseTopology& topology, float tolerance, int maxIterations)
    {
        CgResult result;
        const int subcellCount = SubcellCount(topology);
        if (atlas.Capacity() * kSubcellsPerBrick < subcellCount || !Prepare(subcellCount) ||
            !CUDA_CHECK(cudaMemsetAsync(m_scalars.Data(), 0, m_scalars.Bytes(), m_stream)))
            return result;

        InitKernel<<<subcellCount, kSubcellBlock, 0, m_stream>>>(atlas.BrickNodes(),
                                                                 atlas.Surface(AtlasChannel::Marker),
                                                                 atlas.Surface(AtlasChannel::Divergence),
                                                                 atlas.Surface(AtlasChannel::Pressure),
                                                                 atlas.Surface(AtlasChannel::R),
                                                                 atlas.Surface(AtlasChannel::D),
                                                                 m_partials.Data());
        double rr0 = 0.0;
        if (!CUDA_CHECK(cudaGetLastError()) || !EnqueueSum(subcellCount, kRrOld) || !ReadScalar(kRrOld, rr0))
            return result;
        if (rr0 <= 0.0)
        {
            result.converged = true;
            return result;
        }
        if (!atlas.UpdateApron(AtlasChannel::D, topology, 0.0f, m_stream) ||
            !BuildGraph(atlas, topology, subcellCount))
            return result;

        const double threshold = static_cast<double>(tolerance) * tolerance * rr0;
        double rr = rr0;
        int iterations = 0;
        while (iterations < maxIterations)
        {
            if (!CUDA_CHECK(cudaGraphLaunch(m_graph, m_stream)) || !ReadScalar(kRrOld, rr))
                break;
            iterations += kIterationsPerGraph;
            if (rr <= threshold)
            {
                result.converged = true;
                break;
            }
        }

        result.iterations = iterations;
        result.relativeResidual = static_cast<float>(std::sqrt(rr / rr0));
        return result;
    }

    CgResult SparseProject(BrickAtlas& atlas,
                           const SparseTopology& topology,
                           SparseCg& cg,
                           float3 gravity,
                           float dt,
                           float tolerance,
                           int maxIterations,
                           GpuTimer* timer)
    {
        CgResult result;
        const int subcellCount = SubcellCount(topology);
        if (atlas.Capacity() * kSubcellsPerBrick < subcellCount)
            return result;

        const int* brickNodes = atlas.BrickNodes();
        const cudaSurfaceObject_t markers = atlas.Surface(AtlasChannel::Marker);
        const cudaSurfaceObject_t velocity = atlas.Surface(AtlasChannel::Velocity);
        const float h = topology.Space().h;

        BeginStage(timer, "Sparse forces");
        bool ok = atlas.CopyChannel(AtlasChannel::Velocity, AtlasChannel::VelocityOld);
        ApplyForcesKernel<<<subcellCount, kSubcellBlock>>>(
            brickNodes, markers, velocity, make_float3(gravity.x * dt, gravity.y * dt, gravity.z * dt));
        ok = ok && CUDA_CHECK(cudaGetLastError()) && atlas.UpdateApron(AtlasChannel::Velocity, topology);
        DivergenceKernel<<<subcellCount, kSubcellBlock>>>(
            brickNodes, markers, velocity, atlas.Surface(AtlasChannel::Divergence), h);
        ok = ok && CUDA_CHECK(cudaGetLastError());
        EndStage(timer);
        if (!ok)
            return result;

        BeginStage(timer, "Sparse CG");
        result = cg.Solve(atlas, topology, tolerance, maxIterations);
        EndStage(timer);

        BeginStage(timer, "Sparse project");
        ok = atlas.UpdateApron(AtlasChannel::Pressure, topology);
        PressureToVelocityKernel<<<subcellCount, kSubcellBlock>>>(
            brickNodes, markers, atlas.Surface(AtlasChannel::Pressure), velocity, h);
        ok = ok && CUDA_CHECK(cudaGetLastError()) && atlas.UpdateApron(AtlasChannel::Velocity, topology);
        EndStage(timer);
        if (!ok)
            result.converged = false;
        return result;
    }
}
