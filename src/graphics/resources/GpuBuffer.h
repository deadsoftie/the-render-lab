#pragma once
#include <cstddef>

// GL buffer (SSBO, UBO, copy). Resize only grows.
class GpuBuffer
{
   public:
    GpuBuffer() = default;
    ~GpuBuffer();

    GpuBuffer(const GpuBuffer&) = delete;
    GpuBuffer& operator=(const GpuBuffer&) = delete;

    bool Resize(size_t bytes, bool preserve = false);
    void Upload(const void* data, size_t bytes, size_t offset = 0);
    void Download(void* data, size_t bytes, size_t offset = 0) const;
    void Clear();
    void BindBase(unsigned int target, unsigned int binding) const;
    void Destroy();

    unsigned int ID() const { return m_id; }
    size_t Capacity() const { return m_capacity; }

   private:
    unsigned int m_id = 0;
    size_t m_capacity = 0;
};
