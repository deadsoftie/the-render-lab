#include "fluid/cuda/ParticleKernels.h"

#include <algorithm>
#include <cmath>

#include "fluid/cuda/CudaCheck.h"

namespace Fluid
{
    namespace
    {
        constexpr int kBlockSize = 256;

        int BlockCount(int count)
        {
            return (count + kBlockSize - 1) / kBlockSize;
        }

        __device__ unsigned int Hash(unsigned int x)
        {
            x ^= x >> 16;
            x *= 0x7feb352dU;
            x ^= x >> 15;
            x *= 0x846ca68bU;
            x ^= x >> 16;
            return x;
        }

        __device__ float Jitter(unsigned int key)
        {
            return static_cast<float>(Hash(key) & 0xffffffU) / 16777216.0f - 0.5f;
        }

        __device__ void Bounce(float& p, float& v, float lo, float hi, float restitution)
        {
            if (p < lo)
            {
                p = lo;
                v = fabsf(v) * restitution;
            }
            else if (p > hi)
            {
                p = hi;
                v = -fabsf(v) * restitution;
            }
        }

        __global__ void SeedParticlesKernel(float4* positions,
                                            float4* velocities,
                                            int count,
                                            float3 origin,
                                            float3 cell,
                                            int nx,
                                            int nz)
        {
            int i = blockIdx.x * blockDim.x + threadIdx.x;
            if (i >= count)
                return;

            int x = i % nx;
            int z = (i / nx) % nz;
            int y = i / (nx * nz);
            unsigned int key = static_cast<unsigned int>(i) * 3u;

            positions[i] = make_float4(origin.x + (x + 0.5f + 0.5f * Jitter(key)) * cell.x,
                                       origin.y + (y + 0.5f + 0.5f * Jitter(key + 1u)) * cell.y,
                                       origin.z + (z + 0.5f + 0.5f * Jitter(key + 2u)) * cell.z,
                                       1.0f);
            velocities[i] = make_float4(0.0f, 0.0f, 0.0f, 0.0f);
        }

        __global__ void BallisticStepKernel(float4* positions,
                                            float4* velocities,
                                            int count,
                                            float3 lo,
                                            float3 hi,
                                            float3 gravity,
                                            float dt,
                                            float restitution)
        {
            int i = blockIdx.x * blockDim.x + threadIdx.x;
            if (i >= count)
                return;

            float4 p = positions[i];
            float4 v = velocities[i];

            v.x += gravity.x * dt;
            v.y += gravity.y * dt;
            v.z += gravity.z * dt;
            p.x += v.x * dt;
            p.y += v.y * dt;
            p.z += v.z * dt;

            Bounce(p.x, v.x, lo.x, hi.x, restitution);
            Bounce(p.y, v.y, lo.y, hi.y, restitution);
            Bounce(p.z, v.z, lo.z, hi.z, restitution);

            positions[i] = p;
            velocities[i] = v;
        }
    }

    bool LaunchSeedParticles(float4* positions,
                             float4* velocities,
                             int count,
                             const Box& seed,
                             cudaStream_t stream)
    {
        if (count <= 0)
            return true;

        float ex = seed.hi.x - seed.lo.x;
        float ey = seed.hi.y - seed.lo.y;
        float ez = seed.hi.z - seed.lo.z;
        float spacing = std::cbrt(ex * ey * ez / static_cast<float>(count));
        int nx = std::max(1, static_cast<int>(std::lround(ex / spacing)));
        int nz = std::max(1, static_cast<int>(std::lround(ez / spacing)));
        int ny = (count + nx * nz - 1) / (nx * nz);
        float3 cell = make_float3(ex / nx, ey / ny, ez / nz);

        SeedParticlesKernel<<<BlockCount(count), kBlockSize, 0, stream>>>(
            positions, velocities, count, seed.lo, cell, nx, nz);
        return CUDA_CHECK(cudaGetLastError());
    }

    bool LaunchBallisticStep(float4* positions,
                             float4* velocities,
                             int count,
                             const Box& bounds,
                             float3 gravity,
                             float dt,
                             float restitution,
                             cudaStream_t stream)
    {
        if (count <= 0)
            return true;

        BallisticStepKernel<<<BlockCount(count), kBlockSize, 0, stream>>>(
            positions, velocities, count, bounds.lo, bounds.hi, gravity, dt, restitution);
        return CUDA_CHECK(cudaGetLastError());
    }
}
