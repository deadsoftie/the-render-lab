#include "fluid/cuda/DenseFlip.h"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <utility>

#include <cub/device/device_radix_sort.cuh>

#include "fluid/cuda/CudaCheck.h"
#include "fluid/cuda/GridMath.cuh"
#include "fluid/cuda/Reduce.cuh"
#include "fluid/cuda/Shapes.cuh"

namespace Fluid
{
    namespace
    {
        constexpr int kBlockSize = 256;
        constexpr float kCfl = 1.0f;
        constexpr int kMaxSubsteps = 8;
        constexpr float kWallMargin = 0.01f;
        constexpr float kNormalEpsilon = 0.05f;

        int Blocks(int n)
        {
            return (n + kBlockSize - 1) / kBlockSize;
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

        __device__ void FaceCells(const GridInfo& grid, int axis, int3 f, int& left, int& right, bool& wall)
        {
            int a = Axis(f, axis);
            wall = a == 0 || a == Axis(grid.dims, axis);
            if (wall)
            {
                left = -1;
                right = -1;
                return;
            }
            right = CellIndex(grid, f.x, f.y, f.z);
            left = CellIndex(grid,
                             f.x - (axis == 0 ? 1 : 0),
                             f.y - (axis == 1 ? 1 : 0),
                             f.z - (axis == 2 ? 1 : 0));
        }

        __global__ void CellKeysKernel(const float4* positions,
                                       int count,
                                       GridInfo grid,
                                       unsigned int* keys,
                                       unsigned int* indices)
        {
            int p = blockIdx.x * blockDim.x + threadIdx.x;
            if (p >= count)
                return;

            float3 g = (XYZ(positions[p]) - grid.origin) * (1.0f / grid.h);
            int i = ClampIndex(static_cast<int>(floorf(g.x)), 0, grid.dims.x - 1);
            int j = ClampIndex(static_cast<int>(floorf(g.y)), 0, grid.dims.y - 1);
            int k = ClampIndex(static_cast<int>(floorf(g.z)), 0, grid.dims.z - 1);
            keys[p] = static_cast<unsigned int>(CellIndex(grid, i, j, k));
            indices[p] = static_cast<unsigned int>(p);
        }

        __global__ void PermuteKernel(const unsigned int* order,
                                      int count,
                                      const float4* positionsIn,
                                      const float4* velocitiesIn,
                                      float4* positionsOut,
                                      float4* velocitiesOut)
        {
            int p = blockIdx.x * blockDim.x + threadIdx.x;
            if (p >= count)
                return;
            unsigned int source = order[p];
            positionsOut[p] = positionsIn[source];
            velocitiesOut[p] = velocitiesIn[source];
        }

        __global__ void CellRangesKernel(const unsigned int* keys,
                                         int count,
                                         unsigned int* start,
                                         unsigned int* end)
        {
            int p = blockIdx.x * blockDim.x + threadIdx.x;
            if (p >= count)
                return;
            unsigned int key = keys[p];
            if (p == 0 || keys[p - 1] != key)
                start[key] = static_cast<unsigned int>(p);
            if (p == count - 1 || keys[p + 1] != key)
                end[key] = static_cast<unsigned int>(p + 1);
        }

        __global__ void GatherKernel(GridInfo grid,
                                     ShapeList obstacles,
                                     const float4* positions,
                                     const float4* velocities,
                                     const unsigned int* start,
                                     const unsigned int* end,
                                     float* u,
                                     float* v,
                                     float* w,
                                     unsigned char* markers)
        {
            int c = blockIdx.x * blockDim.x + threadIdx.x;
            if (c >= CellCount(grid))
                return;

            int3 ijk = CellCoord(grid, c);
            float3 fu = make_float3(ijk.x, ijk.y + 0.5f, ijk.z + 0.5f);
            float3 fv = make_float3(ijk.x + 0.5f, ijk.y, ijk.z + 0.5f);
            float3 fw = make_float3(ijk.x + 0.5f, ijk.y + 0.5f, ijk.z);
            float invH = 1.0f / grid.h;

            float su = 0.0f, sv = 0.0f, sw = 0.0f;
            float wu = 0.0f, wv = 0.0f, ww = 0.0f;

            for (int k = ijk.z - 1; k <= ijk.z + 1; ++k)
            {
                for (int j = ijk.y - 1; j <= ijk.y + 1; ++j)
                {
                    for (int i = ijk.x - 1; i <= ijk.x + 1; ++i)
                    {
                        if (!InGrid(grid, i, j, k))
                            continue;
                        int n = CellIndex(grid, i, j, k);
                        unsigned int first = start[n];
                        if (first == kEmptyCell)
                            continue;
                        unsigned int last = end[n];
                        for (unsigned int p = first; p < last; ++p)
                        {
                            float3 g = (XYZ(positions[p]) - grid.origin) * invH;
                            float3 vel = XYZ(velocities[p]);
                            float weightU = Hat(g.x - fu.x) * Hat(g.y - fu.y) * Hat(g.z - fu.z);
                            float weightV = Hat(g.x - fv.x) * Hat(g.y - fv.y) * Hat(g.z - fv.z);
                            float weightW = Hat(g.x - fw.x) * Hat(g.y - fw.y) * Hat(g.z - fw.z);
                            su += weightU * vel.x;
                            sv += weightV * vel.y;
                            sw += weightW * vel.z;
                            wu += weightU;
                            wv += weightV;
                            ww += weightW;
                        }
                    }
                }
            }

            u[FaceIndex(FaceDims(grid, 0), ijk.x, ijk.y, ijk.z)] = wu > 0.0f ? su / wu : 0.0f;
            v[FaceIndex(FaceDims(grid, 1), ijk.x, ijk.y, ijk.z)] = wv > 0.0f ? sv / wv : 0.0f;
            w[FaceIndex(FaceDims(grid, 2), ijk.x, ijk.y, ijk.z)] = ww > 0.0f ? sw / ww : 0.0f;

            float3 center = grid.origin + make_float3(ijk.x + 0.5f, ijk.y + 0.5f, ijk.z + 0.5f) * grid.h;
            if (FirstContaining(obstacles, center) >= 0)
                markers[c] = kCellSolid;
            else
                markers[c] = start[c] != kEmptyCell ? kCellFluid : kCellAir;
        }

        __global__ void ApplyForcesKernel(GridInfo grid,
                                          int axis,
                                          const unsigned char* markers,
                                          float add,
                                          float* component)
        {
            int3 dims = FaceDims(grid, axis);
            int f = blockIdx.x * blockDim.x + threadIdx.x;
            if (f >= dims.x * dims.y * dims.z)
                return;

            int left, right;
            bool wall;
            FaceCells(grid, axis, FaceCoord(dims, f), left, right, wall);
            if (wall || markers[left] == kCellSolid || markers[right] == kCellSolid)
                component[f] = 0.0f;
            else
                component[f] += add;
        }

        __global__ void DivergenceKernel(GridInfo grid,
                                         const unsigned char* markers,
                                         const float* u,
                                         const float* v,
                                         const float* w,
                                         float* rhs)
        {
            int c = blockIdx.x * blockDim.x + threadIdx.x;
            if (c >= CellCount(grid))
                return;
            if (markers[c] != kCellFluid)
            {
                rhs[c] = 0.0f;
                return;
            }

            int3 ijk = CellCoord(grid, c);
            int3 du = FaceDims(grid, 0);
            int3 dv = FaceDims(grid, 1);
            int3 dw = FaceDims(grid, 2);
            float divergence =
                u[FaceIndex(du, ijk.x + 1, ijk.y, ijk.z)] - u[FaceIndex(du, ijk.x, ijk.y, ijk.z)] +
                v[FaceIndex(dv, ijk.x, ijk.y + 1, ijk.z)] - v[FaceIndex(dv, ijk.x, ijk.y, ijk.z)] +
                w[FaceIndex(dw, ijk.x, ijk.y, ijk.z + 1)] - w[FaceIndex(dw, ijk.x, ijk.y, ijk.z)];
            rhs[c] = -grid.h * divergence;
        }

        __global__ void ProjectKernel(GridInfo grid,
                                      int axis,
                                      const unsigned char* markers,
                                      const float* pressure,
                                      float* component)
        {
            int3 dims = FaceDims(grid, axis);
            int f = blockIdx.x * blockDim.x + threadIdx.x;
            if (f >= dims.x * dims.y * dims.z)
                return;

            int left, right;
            bool wall;
            FaceCells(grid, axis, FaceCoord(dims, f), left, right, wall);
            if (wall)
            {
                component[f] = 0.0f;
                return;
            }

            unsigned char ml = markers[left];
            unsigned char mr = markers[right];
            if (ml == kCellSolid || mr == kCellSolid)
                component[f] = 0.0f;
            else if (ml == kCellFluid || mr == kCellFluid)
                component[f] -= (pressure[right] - pressure[left]) / grid.h;
        }

        __global__ void AdvectKernel(GridInfo grid,
                                     ShapeList obstacles,
                                     const float* u,
                                     const float* v,
                                     const float* w,
                                     const float* uOld,
                                     const float* vOld,
                                     const float* wOld,
                                     float dt,
                                     float flipRatio,
                                     int count,
                                     float4* positions,
                                     float4* velocities)
        {
            int p = blockIdx.x * blockDim.x + threadIdx.x;
            if (p >= count)
                return;

            float4 x4 = positions[p];
            float3 x = XYZ(x4);
            float3 vNew = SampleVelocity(grid, u, v, w, x);
            float3 vOldGrid = SampleVelocity(grid, uOld, vOld, wOld, x);
            float3 flip = XYZ(velocities[p]) + vNew - vOldGrid;
            float3 vel = flip * flipRatio + vNew * (1.0f - flipRatio);

            float3 mid = x + vNew * (0.5f * dt);
            x = x + SampleVelocity(grid, u, v, w, mid) * dt;

            for (int s = 0; s < obstacles.count; ++s)
            {
                float d = ShapeSdf(obstacles.items[s], x);
                if (d >= 0.0f)
                    continue;
                float3 n = ShapeNormal(obstacles.items[s], x, kNormalEpsilon * grid.h);
                x = x - n * d;
                float vn = Dot(vel, n);
                if (vn < 0.0f)
                    vel = vel - n * vn;
            }

            float margin = kWallMargin * grid.h;
            float3 lo = grid.origin + make_float3(margin, margin, margin);
            float3 hi = grid.origin +
                        make_float3(grid.dims.x, grid.dims.y, grid.dims.z) * grid.h -
                        make_float3(margin, margin, margin);
            if (x.x < lo.x) { x.x = lo.x; vel.x = fmaxf(vel.x, 0.0f); }
            if (x.x > hi.x) { x.x = hi.x; vel.x = fminf(vel.x, 0.0f); }
            if (x.y < lo.y) { x.y = lo.y; vel.y = fmaxf(vel.y, 0.0f); }
            if (x.y > hi.y) { x.y = hi.y; vel.y = fminf(vel.y, 0.0f); }
            if (x.z < lo.z) { x.z = lo.z; vel.z = fmaxf(vel.z, 0.0f); }
            if (x.z > hi.z) { x.z = hi.z; vel.z = fminf(vel.z, 0.0f); }

            positions[p] = make_float4(x.x, x.y, x.z, x4.w);
            velocities[p] = make_float4(vel.x, vel.y, vel.z, 0.0f);
        }

        __global__ void MaxSpeedKernel(const float4* velocities, int count, unsigned int* result)
        {
            int p = blockIdx.x * blockDim.x + threadIdx.x;
            float speed = p < count ? Length(XYZ(velocities[p])) : 0.0f;
            BlockMaxAtomic<kBlockSize>(speed, result);
        }
    }

    bool DenseFlip::Init(const GridInfo& grid, const ShapeList& obstacles, int particleCount)
    {
        m_grid = grid;
        m_obstacles = obstacles;

        const size_t cells = static_cast<size_t>(CellCount(grid));
        m_keyBits = 1;
        while ((size_t{1} << m_keyBits) < cells)
            ++m_keyBits;

        return m_u.Resize(FaceCount(grid, 0)) && m_v.Resize(FaceCount(grid, 1)) &&
               m_w.Resize(FaceCount(grid, 2)) && m_uOld.Resize(FaceCount(grid, 0)) &&
               m_vOld.Resize(FaceCount(grid, 1)) && m_wOld.Resize(FaceCount(grid, 2)) &&
               m_markers.Resize(cells) && m_rhs.Resize(cells) && m_pressure.Resize(cells) &&
               m_cellStart.Resize(cells) && m_cellEnd.Resize(cells) && m_maxSpeed.Resize(1) &&
               ReserveParticles(particleCount);
    }

    void DenseFlip::Release()
    {
        for (DeviceBuffer<float>* buffer : {&m_u, &m_v, &m_w, &m_uOld, &m_vOld, &m_wOld, &m_rhs, &m_pressure})
            buffer->Free();
        for (DeviceBuffer<unsigned int>* buffer :
             {&m_cellStart, &m_cellEnd, &m_keys, &m_sortedKeys, &m_indices, &m_sortedIndices, &m_maxSpeed})
            buffer->Free();
        m_markers.Free();
        m_sortTemp.Free();
        m_scratchPositions.Free();
        m_scratchVelocities.Free();
        m_cg.Release();
    }

    bool DenseFlip::ReserveParticles(int count)
    {
        const size_t n = static_cast<size_t>(count);
        return m_keys.Resize(n) && m_sortedKeys.Resize(n) && m_indices.Resize(n) &&
               m_sortedIndices.Resize(n) && m_scratchPositions.Resize(n) &&
               m_scratchVelocities.Resize(n);
    }

    StepStats DenseFlip::Step(DeviceBuffer<float4>& positions,
                              DeviceBuffer<float4>& velocities,
                              int count,
                              float frameSeconds,
                              const StepSettings& settings,
                              GpuTimer* timer)
    {
        StepStats stats;
        if (count <= 0 || frameSeconds <= 0.0f)
            return stats;

        stats.maxSpeed = MaxSpeed(velocities, count);
        const float speedBound =
            stats.maxSpeed + std::sqrt(5.0f * m_grid.h * Length(settings.gravity));
        const int substeps = std::clamp(
            static_cast<int>(std::ceil(frameSeconds * speedBound / (kCfl * m_grid.h))), 1, kMaxSubsteps);
        const float dt = frameSeconds / static_cast<float>(substeps);

        for (int s = 0; s < substeps; ++s)
        {
            if (!Transfer(positions, velocities, count, dt, settings.gravity, timer))
                break;
            CgResult cg = Project(settings.cgTolerance, settings.cgMaxIterations, timer);
            if (!Advect(positions, velocities, count, dt, settings.flipRatio, timer))
                break;
            stats.cg = cg;
            stats.cgIterationsTotal += cg.iterations;
            ++stats.substeps;
        }
        return stats;
    }

    float DenseFlip::MaxSpeed(const DeviceBuffer<float4>& velocities, int count)
    {
        if (count <= 0 || !m_maxSpeed.Clear())
            return 0.0f;
        MaxSpeedKernel<<<Blocks(count), kBlockSize>>>(velocities.Data(), count, m_maxSpeed.Data());
        unsigned int bits = 0;
        if (!CUDA_CHECK(cudaGetLastError()) ||
            !CUDA_CHECK(cudaMemcpy(&bits, m_maxSpeed.Data(), sizeof(bits), cudaMemcpyDeviceToHost)))
            return 0.0f;
        float speed = 0.0f;
        std::memcpy(&speed, &bits, sizeof(speed));
        return speed;
    }

    bool DenseFlip::Transfer(DeviceBuffer<float4>& positions,
                             DeviceBuffer<float4>& velocities,
                             int count,
                             float dt,
                             float3 gravity,
                             GpuTimer* timer)
    {
        if (count <= 0 || !ReserveParticles(count))
            return false;
        const int cells = CellCount(m_grid);

        BeginStage(timer, "Sort");
        CellKeysKernel<<<Blocks(count), kBlockSize>>>(
            positions.Data(), count, m_grid, m_keys.Data(), m_indices.Data());
        size_t tempBytes = 0;
        if (!CUDA_CHECK(cub::DeviceRadixSort::SortPairs(nullptr,
                                                        tempBytes,
                                                        m_keys.Data(),
                                                        m_sortedKeys.Data(),
                                                        m_indices.Data(),
                                                        m_sortedIndices.Data(),
                                                        count,
                                                        0,
                                                        m_keyBits)) ||
            !m_sortTemp.Resize(tempBytes) ||
            !CUDA_CHECK(cub::DeviceRadixSort::SortPairs(m_sortTemp.Data(),
                                                        tempBytes,
                                                        m_keys.Data(),
                                                        m_sortedKeys.Data(),
                                                        m_indices.Data(),
                                                        m_sortedIndices.Data(),
                                                        count,
                                                        0,
                                                        m_keyBits)))
        {
            EndStage(timer);
            return false;
        }
        PermuteKernel<<<Blocks(count), kBlockSize>>>(m_sortedIndices.Data(),
                                                     count,
                                                     positions.Data(),
                                                     velocities.Data(),
                                                     m_scratchPositions.Data(),
                                                     m_scratchVelocities.Data());
        std::swap(positions, m_scratchPositions);
        std::swap(velocities, m_scratchVelocities);
        CUDA_CHECK(cudaMemsetAsync(m_cellStart.Data(), 0xff, m_cellStart.Bytes()));
        CellRangesKernel<<<Blocks(count), kBlockSize>>>(
            m_sortedKeys.Data(), count, m_cellStart.Data(), m_cellEnd.Data());
        EndStage(timer);

        BeginStage(timer, "Particle to grid");
        m_u.Clear();
        m_v.Clear();
        m_w.Clear();
        GatherKernel<<<Blocks(cells), kBlockSize>>>(m_grid,
                                                    m_obstacles,
                                                    positions.Data(),
                                                    velocities.Data(),
                                                    m_cellStart.Data(),
                                                    m_cellEnd.Data(),
                                                    m_u.Data(),
                                                    m_v.Data(),
                                                    m_w.Data(),
                                                    m_markers.Data());
        CUDA_CHECK(cudaMemcpyAsync(m_uOld.Data(), m_u.Data(), m_u.Bytes(), cudaMemcpyDeviceToDevice));
        CUDA_CHECK(cudaMemcpyAsync(m_vOld.Data(), m_v.Data(), m_v.Bytes(), cudaMemcpyDeviceToDevice));
        CUDA_CHECK(cudaMemcpyAsync(m_wOld.Data(), m_w.Data(), m_w.Bytes(), cudaMemcpyDeviceToDevice));
        DeviceBuffer<float>* faces[3] = {&m_u, &m_v, &m_w};
        for (int axis = 0; axis < 3; ++axis)
            ApplyForcesKernel<<<Blocks(FaceCount(m_grid, axis)), kBlockSize>>>(
                m_grid, axis, m_markers.Data(), Axis(gravity, axis) * dt, faces[axis]->Data());
        EndStage(timer);

        return CUDA_CHECK(cudaGetLastError());
    }

    CgResult DenseFlip::Project(float tolerance, int maxIterations, GpuTimer* timer)
    {
        BeginStage(timer, "Pressure solve");
        DivergenceKernel<<<Blocks(CellCount(m_grid)), kBlockSize>>>(
            m_grid, m_markers.Data(), m_u.Data(), m_v.Data(), m_w.Data(), m_rhs.Data());
        CgResult result = m_cg.Solve(
            m_grid, m_markers.Data(), m_rhs.Data(), m_pressure.Data(), tolerance, maxIterations);
        DeviceBuffer<float>* faces[3] = {&m_u, &m_v, &m_w};
        for (int axis = 0; axis < 3; ++axis)
            ProjectKernel<<<Blocks(FaceCount(m_grid, axis)), kBlockSize>>>(
                m_grid, axis, m_markers.Data(), m_pressure.Data(), faces[axis]->Data());
        EndStage(timer);

        CUDA_CHECK(cudaGetLastError());
        return result;
    }

    bool DenseFlip::Advect(DeviceBuffer<float4>& positions,
                           DeviceBuffer<float4>& velocities,
                           int count,
                           float dt,
                           float flipRatio,
                           GpuTimer* timer)
    {
        BeginStage(timer, "Advect");
        AdvectKernel<<<Blocks(count), kBlockSize>>>(m_grid,
                                                    m_obstacles,
                                                    m_u.Data(),
                                                    m_v.Data(),
                                                    m_w.Data(),
                                                    m_uOld.Data(),
                                                    m_vOld.Data(),
                                                    m_wOld.Data(),
                                                    dt,
                                                    flipRatio,
                                                    count,
                                                    positions.Data(),
                                                    velocities.Data());
        EndStage(timer);
        return CUDA_CHECK(cudaGetLastError());
    }
}
