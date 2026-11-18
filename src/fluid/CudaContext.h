#pragma once
#include <cstddef>
#include <string>

namespace Fluid
{
    struct CudaDeviceInfo
    {
        int index = -1;
        std::string name;
        int smCount = 0;
        int ccMajor = 0;
        int ccMinor = 0;
        size_t totalMemoryBytes = 0;
    };

    bool SelectGLDevice(CudaDeviceInfo& out, std::string& error);
}
