#include <iostream>

#include <cuda_runtime.h>

#include "FluidTests.h"
#include "TestFramework.h"

int main()
{
    int deviceCount = 0;
    if (cudaGetDeviceCount(&deviceCount) != cudaSuccess || deviceCount == 0)
    {
        std::cerr << "No CUDA device available\n";
        return 1;
    }

    RunDeviceBufferTests();
    RunParticleKernelTests();
    RunGpuTimerTests();
    return Test::Summarize();
}
