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

    int SubstepCount(float maxSpeed, float h, float3 gravity, float frameSeconds)
    {
        const float speedBound = maxSpeed + std::sqrt(5.0f * h * Length(gravity));
        return std::clamp(static_cast<int>(std::ceil(frameSeconds * speedBound / (kCfl * h))), 1, kMaxSubsteps);
    }
}
