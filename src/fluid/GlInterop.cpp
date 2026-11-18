#include "pch.h"
#include "fluid/GlInterop.h"

#include <glad/glad.h>
#include <cuda_gl_interop.h>

#include "fluid/cuda/CudaCheck.h"

namespace Fluid
{
    InteropBuffer::~InteropBuffer()
    {
        Unregister();
    }

    bool InteropBuffer::Register(unsigned int glBuffer, unsigned int flags)
    {
        Unregister();
        if (!CUDA_CHECK(cudaGraphicsGLRegisterBuffer(&m_resource, glBuffer, flags)))
        {
            m_resource = nullptr;
            return false;
        }
        return true;
    }

    void InteropBuffer::Unregister()
    {
        if (!m_resource)
            return;
        if (m_mapped)
            Unmap();
        CUDA_CHECK(cudaGraphicsUnregisterResource(m_resource));
        m_resource = nullptr;
    }

    void* InteropBuffer::Map(size_t& bytes, cudaStream_t stream)
    {
        bytes = 0;
        if (!m_resource || m_mapped)
            return nullptr;
        if (!CUDA_CHECK(cudaGraphicsMapResources(1, &m_resource, stream)))
            return nullptr;
        m_mapped = true;

        void* ptr = nullptr;
        if (!CUDA_CHECK(cudaGraphicsResourceGetMappedPointer(&ptr, &bytes, m_resource)))
        {
            Unmap(stream);
            bytes = 0;
            return nullptr;
        }
        return ptr;
    }

    void InteropBuffer::Unmap(cudaStream_t stream)
    {
        if (!m_mapped)
            return;
        CUDA_CHECK(cudaGraphicsUnmapResources(1, &m_resource, stream));
        m_mapped = false;
    }

    InteropImage::~InteropImage()
    {
        Unregister();
    }

    bool InteropImage::Register(unsigned int glTexture, unsigned int target, unsigned int flags)
    {
        Unregister();
        if (!CUDA_CHECK(cudaGraphicsGLRegisterImage(&m_resource, glTexture, target, flags)))
        {
            m_resource = nullptr;
            return false;
        }
        return true;
    }

    void InteropImage::Unregister()
    {
        if (!m_resource)
            return;
        if (m_mapped)
            Unmap();
        CUDA_CHECK(cudaGraphicsUnregisterResource(m_resource));
        m_resource = nullptr;
    }

    cudaArray_t InteropImage::Map(cudaStream_t stream)
    {
        if (!m_resource || m_mapped)
            return nullptr;
        if (!CUDA_CHECK(cudaGraphicsMapResources(1, &m_resource, stream)))
            return nullptr;
        m_mapped = true;

        cudaArray_t array = nullptr;
        if (!CUDA_CHECK(cudaGraphicsSubResourceGetMappedArray(&array, m_resource, 0, 0)))
        {
            Unmap(stream);
            return nullptr;
        }
        return array;
    }

    void InteropImage::Unmap(cudaStream_t stream)
    {
        if (!m_mapped)
            return;
        CUDA_CHECK(cudaGraphicsUnmapResources(1, &m_resource, stream));
        m_mapped = false;
    }
}
