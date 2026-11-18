#pragma once
#include <cub/block/block_reduce.cuh>

namespace Fluid
{
    struct MaxOp
    {
        __device__ float operator()(float a, float b) const { return fmaxf(a, b); }
    };

    template <int BlockSize>
    __device__ void BlockSumAtomic(double value, double* target)
    {
        using Reduce = cub::BlockReduce<double, BlockSize>;
        __shared__ typename Reduce::TempStorage storage;
        double sum = Reduce(storage).Sum(value);
        if (threadIdx.x == 0)
            atomicAdd(target, sum);
    }

    template <int BlockSize>
    __device__ void BlockMaxAtomic(float value, unsigned int* target)
    {
        using Reduce = cub::BlockReduce<float, BlockSize>;
        __shared__ typename Reduce::TempStorage storage;
        float result = Reduce(storage).Reduce(value, MaxOp{});
        if (threadIdx.x == 0)
            atomicMax(target, __float_as_uint(result));
    }
}
