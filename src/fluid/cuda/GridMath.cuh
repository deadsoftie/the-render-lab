#pragma once
#include "fluid/cuda/FluidTypes.h"
#include "fluid/cuda/VecMath.cuh"

namespace Fluid
{
    constexpr unsigned int kEmptyCell = 0xffffffffu;

    __host__ __device__ inline int CellIndex(const GridInfo& grid, int i, int j, int k)
    {
        return i + grid.dims.x * (j + grid.dims.y * k);
    }

    __host__ __device__ inline int3 CellCoord(const GridInfo& grid, int c)
    {
        return make_int3(c % grid.dims.x, (c / grid.dims.x) % grid.dims.y, c / (grid.dims.x * grid.dims.y));
    }

    __host__ __device__ inline bool InGrid(const GridInfo& grid, int i, int j, int k)
    {
        return i >= 0 && j >= 0 && k >= 0 && i < grid.dims.x && j < grid.dims.y && k < grid.dims.z;
    }

    __host__ __device__ inline int3 FaceDims(const GridInfo& grid, int axis)
    {
        return make_int3(grid.dims.x + (axis == 0 ? 1 : 0),
                         grid.dims.y + (axis == 1 ? 1 : 0),
                         grid.dims.z + (axis == 2 ? 1 : 0));
    }

    __host__ __device__ inline int FaceCount(const GridInfo& grid, int axis)
    {
        int3 dims = FaceDims(grid, axis);
        return dims.x * dims.y * dims.z;
    }

    __host__ __device__ inline int FaceIndex(int3 dims, int i, int j, int k)
    {
        return i + dims.x * (j + dims.y * k);
    }

    __host__ __device__ inline int3 FaceCoord(int3 dims, int f)
    {
        return make_int3(f % dims.x, (f / dims.x) % dims.y, f / (dims.x * dims.y));
    }

    __host__ __device__ inline int ClampIndex(int v, int lo, int hi)
    {
        return v < lo ? lo : (v > hi ? hi : v);
    }

    __host__ __device__ inline float Hat(float x)
    {
        return fmaxf(0.0f, 1.0f - fabsf(x));
    }

    __host__ __device__ inline float SampleFace(const float* data, int3 dims, float3 g)
    {
        float gx = fminf(fmaxf(g.x, 0.0f), dims.x - 1.0f);
        float gy = fminf(fmaxf(g.y, 0.0f), dims.y - 1.0f);
        float gz = fminf(fmaxf(g.z, 0.0f), dims.z - 1.0f);
        int i = ClampIndex(static_cast<int>(floorf(gx)), 0, dims.x - 2);
        int j = ClampIndex(static_cast<int>(floorf(gy)), 0, dims.y - 2);
        int k = ClampIndex(static_cast<int>(floorf(gz)), 0, dims.z - 2);
        float tx = gx - i;
        float ty = gy - j;
        float tz = gz - k;

        float c00 = Lerp(data[FaceIndex(dims, i, j, k)], data[FaceIndex(dims, i + 1, j, k)], tx);
        float c10 =
            Lerp(data[FaceIndex(dims, i, j + 1, k)], data[FaceIndex(dims, i + 1, j + 1, k)], tx);
        float c01 =
            Lerp(data[FaceIndex(dims, i, j, k + 1)], data[FaceIndex(dims, i + 1, j, k + 1)], tx);
        float c11 = Lerp(data[FaceIndex(dims, i, j + 1, k + 1)],
                         data[FaceIndex(dims, i + 1, j + 1, k + 1)],
                         tx);
        return Lerp(Lerp(c00, c10, ty), Lerp(c01, c11, ty), tz);
    }

    __host__ __device__ inline float3 SampleVelocity(const GridInfo& grid,
                                                     const float* u,
                                                     const float* v,
                                                     const float* w,
                                                     float3 p)
    {
        float3 g = (p - grid.origin) * (1.0f / grid.h);
        return make_float3(SampleFace(u, FaceDims(grid, 0), make_float3(g.x, g.y - 0.5f, g.z - 0.5f)),
                           SampleFace(v, FaceDims(grid, 1), make_float3(g.x - 0.5f, g.y, g.z - 0.5f)),
                           SampleFace(w, FaceDims(grid, 2), make_float3(g.x - 0.5f, g.y - 0.5f, g.z)));
    }
}
