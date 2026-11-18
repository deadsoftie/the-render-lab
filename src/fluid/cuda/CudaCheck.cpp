#include "fluid/cuda/CudaCheck.h"

#include <iostream>

namespace Fluid
{
    bool CudaCheck(cudaError_t result, const char* expr, const char* file, int line)
    {
        if (result == cudaSuccess)
            return true;
        std::cerr << "[CUDA] " << file << ":" << line << ": " << expr
                  << " failed: " << cudaGetErrorString(result) << "\n";
        return false;
    }
}
