#include "fluid/cuda/Particles.h"

#include <algorithm>
#include <cmath>
#include <cstring>

#include "fluid/cuda/CudaCheck.h"
#include "fluid/cuda/Reduce.cuh"
#include "fluid/cuda/VecMath.cuh"

namespace Fluid
{
    namespace
    {
        constexpr int kBlockSize = 256;
        constexpr float kCfl = 1.0f;
        constexpr int kMaxSubsteps = 8;

        __global__ void MaxSpeedKernel(const float4* velocities, int count, unsigned int* result)
        {
            int p = blockIdx.x * blockDim.x + threadIdx.x;
            float speed = p < count ? Length(XYZ(velocities[p])) : 0.0f;
            BlockMaxAtomic<kBlockSize>(speed, result);
        }

        constexpr float kQuantizeSteps = 65535.0f;

        float3 DomainExtent(const GridInfo& grid)
        {
            return make_float3(grid.dims.x * grid.h, grid.dims.y * grid.h, grid.dims.z * grid.h);
        }

        __device__ unsigned short QuantizeAxis(float value, float origin, float extent)
        {
            const float t = fminf(fmaxf((value - origin) / extent, 0.0f), 1.0f);
            return static_cast<unsigned short>(t * kQuantizeSteps + 0.5f);
        }

        __global__ void QuantizeKernel(const float4* positions,
                                       int count,
                                       float3 origin,
                                       float3 extent,
                                       unsigned short* quantized)
        {
            int p = blockIdx.x * blockDim.x + threadIdx.x;
            if (p >= count)
                return;
            const float4 x = positions[p];
            quantized[3 * p] = QuantizeAxis(x.x, origin.x, extent.x);
            quantized[3 * p + 1] = QuantizeAxis(x.y, origin.y, extent.y);
            quantized[3 * p + 2] = QuantizeAxis(x.z, origin.z, extent.z);
        }

        __global__ void DequantizeKernel(const unsigned short* quantized,
                                         int count,
                                         float3 origin,
                                         float3 extent,
                                         float4* positions)
        {
            int p = blockIdx.x * blockDim.x + threadIdx.x;
            if (p >= count)
                return;
            const float scale = 1.0f / kQuantizeSteps;
            positions[p] = make_float4(origin.x + quantized[3 * p] * scale * extent.x,
                                       origin.y + quantized[3 * p + 1] * scale * extent.y,
                                       origin.z + quantized[3 * p + 2] * scale * extent.z,
                                       1.0f);
        }
    }

    bool QuantizePositions(const float4* positions,
                           int count,
                           const GridInfo& grid,
                           DeviceBuffer<unsigned short>& quantized)
    {
        if (!quantized.Resize(static_cast<size_t>(count) * 3))
            return false;
        if (count == 0)
            return true;
        QuantizeKernel<<<(count + kBlockSize - 1) / kBlockSize, kBlockSize>>>(
            positions, count, grid.origin, DomainExtent(grid), quantized.Data());
        return CUDA_CHECK(cudaGetLastError());
    }

    bool DequantizePositions(const DeviceBuffer<unsigned short>& quantized,
                             int count,
                             const GridInfo& grid,
                             DeviceBuffer<float4>& positions)
    {
        if (quantized.Size() < static_cast<size_t>(count) * 3 || !positions.Resize(count))
            return false;
        if (count == 0)
            return true;
        DequantizeKernel<<<(count + kBlockSize - 1) / kBlockSize, kBlockSize>>>(
            quantized.Data(), count, grid.origin, DomainExtent(grid), positions.Data());
        return CUDA_CHECK(cudaGetLastError());
    }

    float MaxParticleSpeed(const float4* velocities, int count, DeviceBuffer<unsigned int>& scratch)
    {
        if (count <= 0 || !scratch.Resize(1) || !scratch.Clear())
            return 0.0f;
        MaxSpeedKernel<<<(count + kBlockSize - 1) / kBlockSize, kBlockSize>>>(velocities, count, scratch.Data());
        unsigned int bits = 0;
        if (!CUDA_CHECK(cudaGetLastError()) ||
            !CUDA_CHECK(cudaMemcpy(&bits, scratch.Data(), sizeof(bits), cudaMemcpyDeviceToHost)))
            return 0.0f;
        float speed = 0.0f;
        std::memcpy(&speed, &bits, sizeof(speed));
        return speed;
    }

    // CFL substeps for one frame. The square root of 5 times the cell size times the gravity
    // magnitude bounds the speed gravity can add within a step (Bridson's estimate), so a fluid at
    // rest still gets a sensible step.
    int SubstepCount(float maxSpeed, float h, float3 gravity, float frameSeconds)
    {
        const float speedBound = maxSpeed + std::sqrt(5.0f * h * Length(gravity));
        return std::clamp(static_cast<int>(std::ceil(frameSeconds * speedBound / (kCfl * h))), 1, kMaxSubsteps);
    }
}
