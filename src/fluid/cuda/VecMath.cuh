#pragma once
#include <cmath>

#include <cuda_runtime.h>

namespace Fluid
{
    __host__ __device__ inline float3 operator+(float3 a, float3 b)
    {
        return make_float3(a.x + b.x, a.y + b.y, a.z + b.z);
    }

    __host__ __device__ inline float3 operator-(float3 a, float3 b)
    {
        return make_float3(a.x - b.x, a.y - b.y, a.z - b.z);
    }

    __host__ __device__ inline float3 operator-(float3 a)
    {
        return make_float3(-a.x, -a.y, -a.z);
    }

    __host__ __device__ inline float3 operator*(float3 a, float s)
    {
        return make_float3(a.x * s, a.y * s, a.z * s);
    }

    __host__ __device__ inline float3 operator*(float s, float3 a)
    {
        return a * s;
    }

    __host__ __device__ inline float3 operator*(float3 a, float3 b)
    {
        return make_float3(a.x * b.x, a.y * b.y, a.z * b.z);
    }

    __host__ __device__ inline float Dot(float3 a, float3 b)
    {
        return a.x * b.x + a.y * b.y + a.z * b.z;
    }

    __host__ __device__ inline float Length(float3 a)
    {
        return sqrtf(Dot(a, a));
    }

    __host__ __device__ inline float3 Abs(float3 a)
    {
        return make_float3(fabsf(a.x), fabsf(a.y), fabsf(a.z));
    }

    __host__ __device__ inline float3 Max(float3 a, float s)
    {
        return make_float3(fmaxf(a.x, s), fmaxf(a.y, s), fmaxf(a.z, s));
    }

    __host__ __device__ inline float3 XYZ(float4 v)
    {
        return make_float3(v.x, v.y, v.z);
    }

    __host__ __device__ inline float Axis(float3 v, int axis)
    {
        return axis == 0 ? v.x : (axis == 1 ? v.y : v.z);
    }

    __host__ __device__ inline int Axis(int3 v, int axis)
    {
        return axis == 0 ? v.x : (axis == 1 ? v.y : v.z);
    }

    __host__ __device__ inline float Lerp(float a, float b, float t)
    {
        return a + (b - a) * t;
    }
}
