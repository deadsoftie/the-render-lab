#pragma once
#include <cstddef>

#include <cuda_runtime.h>

namespace Fluid
{
    class InteropBuffer
    {
       public:
        InteropBuffer() = default;
        ~InteropBuffer();

        InteropBuffer(const InteropBuffer&) = delete;
        InteropBuffer& operator=(const InteropBuffer&) = delete;

        bool Register(unsigned int glBuffer, unsigned int flags);
        void Unregister();

        void* Map(size_t& bytes, cudaStream_t stream = 0);
        void Unmap(cudaStream_t stream = 0);

       private:
        cudaGraphicsResource_t m_resource = nullptr;
        bool m_mapped = false;
    };

    class InteropImage
    {
       public:
        InteropImage() = default;
        ~InteropImage();

        InteropImage(const InteropImage&) = delete;
        InteropImage& operator=(const InteropImage&) = delete;

        bool Register(unsigned int glTexture, unsigned int target, unsigned int flags);
        void Unregister();

        cudaArray_t Map(cudaStream_t stream = 0);
        void Unmap(cudaStream_t stream = 0);

       private:
        cudaGraphicsResource_t m_resource = nullptr;
        bool m_mapped = false;
    };
}
