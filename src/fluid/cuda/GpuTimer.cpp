#include "fluid/cuda/GpuTimer.h"

#include <algorithm>

#include "fluid/cuda/CudaCheck.h"

namespace Fluid
{
    namespace
    {
        constexpr float kSmoothing = 0.1f;
    }

    GpuTimer::~GpuTimer()
    {
        Reset();
    }

    void GpuTimer::Reset()
    {
        for (Slot& slot : m_slots)
        {
            if (slot.start)
                cudaEventDestroy(slot.start);
            if (slot.stop)
                cudaEventDestroy(slot.stop);
        }
        m_slots.clear();
        m_stages.clear();
        m_open = -1;
    }

    void GpuTimer::BeginFrame()
    {
        m_stages.clear();
        for (Slot& slot : m_slots)
        {
            if (slot.pending && cudaEventQuery(slot.stop) == cudaSuccess)
            {
                float ms = 0.0f;
                if (CUDA_CHECK(cudaEventElapsedTime(&ms, slot.start, slot.stop)))
                {
                    slot.ms = slot.valid ? slot.ms + (ms - slot.ms) * kSmoothing : ms;
                    slot.valid = true;
                }
                slot.pending = false;
            }
            m_stages.push_back({slot.name, slot.ms, slot.valid});
        }
    }

    void GpuTimer::Begin(const char* name, cudaStream_t stream)
    {
        auto it = std::find_if(m_slots.begin(),
                               m_slots.end(),
                               [name](const Slot& slot) { return slot.name == name; });
        if (it == m_slots.end())
        {
            Slot slot;
            slot.name = name;
            if (!CUDA_CHECK(cudaEventCreate(&slot.start)) ||
                !CUDA_CHECK(cudaEventCreate(&slot.stop)))
            {
                if (slot.start)
                    cudaEventDestroy(slot.start);
                m_open = -1;
                return;
            }
            m_slots.push_back(slot);
            it = m_slots.end() - 1;
        }

        m_open = static_cast<int>(it - m_slots.begin());
        if (!CUDA_CHECK(cudaEventRecord(it->start, stream)))
            m_open = -1;
    }

    void GpuTimer::End(cudaStream_t stream)
    {
        if (m_open < 0)
            return;
        Slot& slot = m_slots[m_open];
        if (CUDA_CHECK(cudaEventRecord(slot.stop, stream)))
            slot.pending = true;
        m_open = -1;
    }

    float GpuTimer::TotalMs() const
    {
        float total = 0.0f;
        for (const Stage& stage : m_stages)
            if (stage.valid)
                total += stage.ms;
        return total;
    }
}
