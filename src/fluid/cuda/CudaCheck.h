#pragma once
#include <cuda_runtime.h>

namespace Fluid
{
    bool CudaCheck(cudaError_t result, const char* expr, const char* file, int line);
}

#define CUDA_CHECK(expr) ::Fluid::CudaCheck((expr), #expr, __FILE__, __LINE__)
