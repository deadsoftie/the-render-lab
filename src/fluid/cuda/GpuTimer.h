#pragma once
#include <string>
#include <vector>

#include <cuda_runtime.h>

namespace Fluid
{
    class GpuTimer
    {
       public:
        struct Stage
        {
            std::string name;
            float ms = 0.0f;
            bool valid = false;
        };

        GpuTimer() = default;
        ~GpuTimer();

        GpuTimer(const GpuTimer&) = delete;
        GpuTimer& operator=(const GpuTimer&) = delete;

        void BeginFrame();
        void Begin(const char* name, cudaStream_t stream = 0);
        void End(cudaStream_t stream = 0);
        void Reset();

        const std::vector<Stage>& Stages() const { return m_stages; }
        float TotalMs() const;

       private:
        struct Slot
        {
            std::string name;
            cudaEvent_t start = nullptr;
            cudaEvent_t stop = nullptr;
            bool pending = false;
            bool valid = false;
            float ms = 0.0f;
        };

        std::vector<Slot> m_slots;
        std::vector<Stage> m_stages;
        int m_open = -1;
    };

    // Stage markers for code that runs with or without a timer.
    inline void BeginStage(GpuTimer* timer, const char* name, cudaStream_t stream = 0)
    {
        if (timer)
            timer->Begin(name, stream);
    }

    inline void EndStage(GpuTimer* timer, cudaStream_t stream = 0)
    {
        if (timer)
            timer->End(stream);
    }
}
