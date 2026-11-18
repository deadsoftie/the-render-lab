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
}
