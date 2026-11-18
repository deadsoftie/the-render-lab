#include "pch.h"
#include "fluid/CudaContext.h"

#include <glad/glad.h>
#include <cuda_gl_interop.h>

#include "fluid/cuda/CudaCheck.h"

namespace Fluid
{
    // Interop only works when CUDA runs on the GPU that owns the GL context, so pick that one
    // instead of CUDA's default device (they differ on hybrid-graphics laptops).
    bool SelectGLDevice(CudaDeviceInfo& out, std::string& error)
    {
        unsigned int count = 0;
        int devices[4] = {};
        cudaError_t result = cudaGLGetDevices(&count, devices, 4, cudaGLDeviceListAll);
        if (result != cudaSuccess || count == 0)
        {
            cudaGetLastError();
            error = std::string("no CUDA device drives the OpenGL context (") +
                    cudaGetErrorString(result) + ")";
            return false;
        }

        cudaDeviceProp props{};
        if (!CUDA_CHECK(cudaSetDevice(devices[0])) ||
            !CUDA_CHECK(cudaGetDeviceProperties(&props, devices[0])))
        {
            error = "failed to select CUDA device " + std::to_string(devices[0]);
            return false;
        }

        out.index = devices[0];
        out.name = props.name;
        out.smCount = props.multiProcessorCount;
        out.ccMajor = props.major;
        out.ccMinor = props.minor;
        out.totalMemoryBytes = props.totalGlobalMem;
        return true;
    }
}  // namespace Fluid
