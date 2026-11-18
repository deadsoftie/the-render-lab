#include "fluid/cuda/Seeding.h"

#include <algorithm>
#include <climits>
#include <cmath>
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

        struct CellRange
        {
            int3 lo{0, 0, 0};
            int3 size{0, 0, 0};
        };

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

        int CellOf(float coordinate, float origin, float h, int dims)
        {
            return ClampIndex(static_cast<int>(std::floor((coordinate - origin) / h)), 0, dims - 1);
        }

        CellRange ShapeCells(const GridInfo& grid, const ShapeList& shapes)
        {
            float3 lo = make_float3(INFINITY, INFINITY, INFINITY);
            float3 hi = make_float3(-INFINITY, -INFINITY, -INFINITY);
            for (int i = 0; i < shapes.count; ++i)
            {
                const Shape& shape = shapes.items[i];
                const float3 r = make_float3(shape.radius, shape.radius, shape.radius);
                const float3 a = shape.type == ShapeType::Sphere ? shape.center - r : shape.lo;
                const float3 b = shape.type == ShapeType::Sphere ? shape.center + r : shape.hi;
                lo = make_float3(std::min(lo.x, a.x), std::min(lo.y, a.y), std::min(lo.z, a.z));
                hi = make_float3(std::max(hi.x, b.x), std::max(hi.y, b.y), std::max(hi.z, b.z));
            }

            CellRange range;
            if (shapes.count == 0)
                return range;
            const float3 top = grid.origin + make_float3(grid.dims.x, grid.dims.y, grid.dims.z) * grid.h;
            if (hi.x <= grid.origin.x || hi.y <= grid.origin.y || hi.z <= grid.origin.z || lo.x >= top.x ||
                lo.y >= top.y || lo.z >= top.z)
                return range;

            range.lo = make_int3(CellOf(lo.x, grid.origin.x, grid.h, grid.dims.x),
                                 CellOf(lo.y, grid.origin.y, grid.h, grid.dims.y),
                                 CellOf(lo.z, grid.origin.z, grid.h, grid.dims.z));
            const int3 last = make_int3(CellOf(hi.x, grid.origin.x, grid.h, grid.dims.x),
                                        CellOf(hi.y, grid.origin.y, grid.h, grid.dims.y),
                                        CellOf(hi.z, grid.origin.z, grid.h, grid.dims.z));
            range.size = make_int3(last.x - range.lo.x + 1, last.y - range.lo.y + 1, last.z - range.lo.z + 1);
            return range;
        }

        __global__ void SeedKernel(GridInfo grid,
                                   CellRange range,
                                   unsigned int salt,
                                   ShapeList seeds,
                                   ShapeList obstacles,
                                   float4* positions,
                                   float4* velocities,
                                   int capacity,
                                   int* counter)
        {
            int candidate = blockIdx.x * blockDim.x + threadIdx.x;
            int cells = range.size.x * range.size.y * range.size.z;
            if (candidate >= cells * kPerCell)
                return;

            int c = candidate / kPerCell;
            int3 cell = make_int3(range.lo.x + c % range.size.x,
                                  range.lo.y + (c / range.size.x) % range.size.y,
                                  range.lo.z + c / (range.size.x * range.size.y));
            int sub = candidate % kPerCell;
            int sx = sub % kParticlesPerCellAxis;
            int sy = (sub / kParticlesPerCellAxis) % kParticlesPerCellAxis;
            int sz = sub / (kParticlesPerCellAxis * kParticlesPerCellAxis);
            const unsigned int cellKey =
                static_cast<unsigned int>(cell.x) +
                static_cast<unsigned int>(grid.dims.x) *
                    (static_cast<unsigned int>(cell.y) +
                     static_cast<unsigned int>(grid.dims.y) * static_cast<unsigned int>(cell.z));
            unsigned int key = (cellKey * kPerCell + sub) * 3u + salt * 0x9e3779b9u;
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

        // Two passes over the cells inside the shapes' bounds: count the 8 jittered candidates per cell
        // that land in a shape and outside every obstacle, then grow the buffers and write them
        // after `offset`. Visiting only the shapes' bounds keeps huge domains cheap to seed.
        bool Fill(const GridInfo& grid,
                  const ShapeList& shapes,
                  const ShapeList& obstacles,
                  DeviceBuffer<float4>& positions,
                  DeviceBuffer<float4>& velocities,
                  int offset,
                  int maxCount,
                  unsigned int salt,
                  int& added)
        {
            added = 0;
            const CellRange range = ShapeCells(grid, shapes);
            const long long candidates =
                static_cast<long long>(range.size.x) * range.size.y * range.size.z * kPerCell;
            if (candidates == 0 || offset >= maxCount)
                return true;
            if (candidates > INT_MAX)
                return false;
            const int blocks = static_cast<int>((candidates + kBlockSize - 1) / kBlockSize);

            DeviceBuffer<int> counter(1);
            std::vector<int> host;
            if (!counter.Clear())
                return false;
            SeedKernel<<<blocks, kBlockSize>>>(
                grid, range, salt, shapes, obstacles, nullptr, nullptr, 0, counter.Data());
            if (!CUDA_CHECK(cudaGetLastError()) || !counter.Download(host))
                return false;

            const int fits = std::min(host[0], maxCount - offset);
            if (fits <= 0)
                return true;
            if (!positions.Grow(static_cast<size_t>(offset) + fits) ||
                !velocities.Grow(static_cast<size_t>(offset) + fits) || !counter.Clear())
                return false;
            SeedKernel<<<blocks, kBlockSize>>>(grid,
                                               range,
                                               salt,
                                               shapes,
                                               obstacles,
                                               positions.Data() + offset,
                                               velocities.Data() + offset,
                                               fits,
                                               counter.Data());
            if (!CUDA_CHECK(cudaGetLastError()))
                return false;
            added = fits;
            return true;
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
        if (grid.dims.x <= 0 || grid.dims.y <= 0 || grid.dims.z <= 0 || !positions.Resize(0) ||
            !velocities.Resize(0))
            return false;
        return Fill(grid, seeds, obstacles, positions, velocities, 0, INT_MAX, 0u, count);
    }

    bool AppendParticles(const GridInfo& grid,
                         const ShapeList& shapes,
                         const ShapeList& obstacles,
                         DeviceBuffer<float4>& positions,
                         DeviceBuffer<float4>& velocities,
                         int& count,
                         int maxCount,
                         unsigned int salt)
    {
        if (!positions.Resize(count) || !velocities.Resize(count))
            return false;
        int added = 0;
        if (!Fill(grid, shapes, obstacles, positions, velocities, count, maxCount, salt, added))
            return false;
        count += added;
        return true;
    }
}
