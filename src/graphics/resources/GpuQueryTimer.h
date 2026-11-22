#pragma once
#include <string>
#include <vector>

class GpuQueryTimer
{
   public:
    struct Stage
    {
        std::string name;
        float ms = 0.0f;
    };

    GpuQueryTimer() = default;
    ~GpuQueryTimer();

    GpuQueryTimer(const GpuQueryTimer&) = delete;
    GpuQueryTimer& operator=(const GpuQueryTimer&) = delete;

    void BeginFrame();
    void Begin(const char* name);
    void End();
    void Destroy();

    const std::vector<Stage>& Stages() const { return m_stages; }
    float TotalMs() const;

   private:
    struct Slot
    {
        std::string name;
        unsigned int query[2] = {0, 0};
        bool issued[2] = {false, false};
    };

    std::vector<Slot> m_slots;
    std::vector<Stage> m_stages;
    int m_set = 0;
    int m_cursor = 0;
    bool m_open = false;
};
