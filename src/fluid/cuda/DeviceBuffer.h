#pragma once
#include <cstddef>
#include <utility>
#include <vector>

#include "fluid/cuda/CudaCheck.h"

namespace Fluid
{
    template <typename T>
    class DeviceBuffer
    {
       public:
        DeviceBuffer() = default;
        explicit DeviceBuffer(size_t count) { Resize(count); }
        ~DeviceBuffer() { Free(); }

        DeviceBuffer(const DeviceBuffer&) = delete;
        DeviceBuffer& operator=(const DeviceBuffer&) = delete;

        DeviceBuffer(DeviceBuffer&& other) noexcept
            : m_data(std::exchange(other.m_data, nullptr)),
              m_count(std::exchange(other.m_count, 0)),
              m_capacity(std::exchange(other.m_capacity, 0))
        {
        }

        DeviceBuffer& operator=(DeviceBuffer&& other) noexcept
        {
            if (this != &other)
            {
                Free();
                m_data = std::exchange(other.m_data, nullptr);
                m_count = std::exchange(other.m_count, 0);
                m_capacity = std::exchange(other.m_capacity, 0);
            }
            return *this;
        }

        bool Resize(size_t count)
        {
            if (count <= m_capacity)
            {
                m_count = count;
                return true;
            }
            Free();
            void* data = nullptr;
            if (!CUDA_CHECK(cudaMalloc(&data, count * sizeof(T))))
                return false;
            m_data = static_cast<T*>(data);
            m_count = count;
            m_capacity = count;
            return true;
        }

        void Free()
        {
            if (m_data)
                CUDA_CHECK(cudaFree(m_data));
            m_data = nullptr;
            m_count = 0;
            m_capacity = 0;
        }

        bool Clear(cudaStream_t stream = 0)
        {
            return m_count == 0 || CUDA_CHECK(cudaMemsetAsync(m_data, 0, Bytes(), stream));
        }

        bool Upload(const std::vector<T>& host)
        {
            if (!Resize(host.size()))
                return false;
            return m_count == 0 ||
                   CUDA_CHECK(cudaMemcpy(m_data, host.data(), Bytes(), cudaMemcpyHostToDevice));
        }

        bool Download(std::vector<T>& host) const
        {
            host.resize(m_count);
            return m_count == 0 ||
                   CUDA_CHECK(cudaMemcpy(host.data(), m_data, Bytes(), cudaMemcpyDeviceToHost));
        }

        T* Data() const { return m_data; }
        size_t Size() const { return m_count; }
        size_t Bytes() const { return m_count * sizeof(T); }

       private:
        T* m_data = nullptr;
        size_t m_count = 0;
        size_t m_capacity = 0;
    };
}
