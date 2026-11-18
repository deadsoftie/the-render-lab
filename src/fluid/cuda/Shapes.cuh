#pragma once
#include "fluid/cuda/FluidTypes.h"
#include "fluid/cuda/VecMath.cuh"

namespace Fluid
{
    __host__ __device__ inline float ShapeSdf(const Shape& shape, float3 p)
    {
        if (shape.type == ShapeType::Sphere)
            return Length(p - shape.center) - shape.radius;

        float3 center = (shape.lo + shape.hi) * 0.5f;
        float3 half = (shape.hi - shape.lo) * 0.5f;
        float3 q = Abs(p - center) - half;
        float inside = fminf(fmaxf(q.x, fmaxf(q.y, q.z)), 0.0f);
        return Length(Max(q, 0.0f)) + inside;
    }

    __host__ __device__ inline float3 ShapeNormal(const Shape& shape, float3 p, float eps)
    {
        float3 dx = make_float3(eps, 0.0f, 0.0f);
        float3 dy = make_float3(0.0f, eps, 0.0f);
        float3 dz = make_float3(0.0f, 0.0f, eps);
        float3 n = make_float3(ShapeSdf(shape, p + dx) - ShapeSdf(shape, p - dx),
                               ShapeSdf(shape, p + dy) - ShapeSdf(shape, p - dy),
                               ShapeSdf(shape, p + dz) - ShapeSdf(shape, p - dz));
        float len = Length(n);
        return len > 0.0f ? n * (1.0f / len) : make_float3(0.0f, 1.0f, 0.0f);
    }

    __host__ __device__ inline int FirstContaining(const ShapeList& list, float3 p)
    {
        for (int i = 0; i < list.count; ++i)
            if (ShapeSdf(list.items[i], p) < 0.0f)
                return i;
        return -1;
    }
}
