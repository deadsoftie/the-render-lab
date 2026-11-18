#include "fluid/cuda/Seeding.h"

#include <vector>

#include "fluid/cuda/CudaCheck.h"
#include "fluid/cuda/GridMath.cuh"
#include "fluid/cuda/Shapes.cuh"

namespace Fluid
{
    namespace
    {
        constexpr int kBlockSize = 256;
        constexpr int kPerCell = kParticlesPerCellAxis * kParticlesPerCellAxis * kParticlesPerCellAxis;

        int Blocks(int n)
        {
            return (n + kBlockSize - 1) / kBlockSize;
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

        __global__ void SeedKernel(GridInfo grid,
                                   ShapeList seeds,
                                   ShapeList obstacles,
                                   float4* positions,
                                   float4* velocities,
                                   int capacity,
                                   int* counter)
        {
            int candidate = blockIdx.x * blockDim.x + threadIdx.x;
            if (candidate >= CellCount(grid) * kPerCell)
                return;

            int3 cell = CellCoord(grid, candidate / kPerCell);
            int sub = candidate % kPerCell;
            int sx = sub % kParticlesPerCellAxis;
            int sy = (sub / kParticlesPerCellAxis) % kParticlesPerCellAxis;
            int sz = sub / (kParticlesPerCellAxis * kParticlesPerCellAxis);
            unsigned int key = static_cast<unsigned int>(candidate) * 3u;
            const float spacing = 1.0f / kParticlesPerCellAxis;

            float3 local = make_float3(cell.x + (sx + 0.5f + 0.5f * Jitter(key)) * spacing,
                                       cell.y + (sy + 0.5f + 0.5f * Jitter(key + 1u)) * spacing,
                                       cell.z + (sz + 0.5f + 0.5f * Jitter(key + 2u)) * spacing);
            float3 p = grid.origin + local * grid.h;

            int seed = FirstContaining(seeds, p);
            if (seed < 0 || FirstContaining(obstacles, p) >= 0)
                return;

            int slot = atomicAdd(counter, 1);
            if (!positions || slot >= capacity)
                return;

            float3 v = seeds.items[seed].velocity;
            positions[slot] = make_float4(p.x, p.y, p.z, 1.0f);
            velocities[slot] = make_float4(v.x, v.y, v.z, 0.0f);
        }
    }

    bool SeedParticles(const GridInfo& grid,
                       const ShapeList& seeds,
                       const ShapeList& obstacles,
                       DeviceBuffer<float4>& positions,
                       DeviceBuffer<float4>& velocities,
                       int& count)
    {
        count = 0;
        const int candidates = CellCount(grid) * kPerCell;
        if (candidates <= 0)
            return false;

        DeviceBuffer<int> counter(1);
        std::vector<int> host;
        if (!counter.Clear())
            return false;
        SeedKernel<<<Blocks(candidates), kBlockSize>>>(
            grid, seeds, obstacles, nullptr, nullptr, 0, counter.Data());
        if (!CUDA_CHECK(cudaGetLastError()) || !counter.Download(host))
            return false;

        const int seeded = host[0];
        if (seeded == 0)
            return true;

        if (!positions.Resize(seeded) || !velocities.Resize(seeded) || !counter.Clear())
            return false;
        SeedKernel<<<Blocks(candidates), kBlockSize>>>(
            grid, seeds, obstacles, positions.Data(), velocities.Data(), seeded, counter.Data());
        if (!CUDA_CHECK(cudaGetLastError()))
            return false;

        count = seeded;
        return true;
    }
}
