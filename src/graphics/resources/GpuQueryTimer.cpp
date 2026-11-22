#include "pch.h"
#include "graphics/resources/GpuQueryTimer.h"

#include <glad/glad.h>

GpuQueryTimer::~GpuQueryTimer()
{
    Destroy();
}

void GpuQueryTimer::BeginFrame()
{
    if (m_open)
        End();

    m_set = 1 - m_set;
    std::vector<Stage> stages;
    for (Slot& slot : m_slots)
    {
        if (!slot.issued[m_set])
            continue;
        GLint available = 0;
        glGetQueryObjectiv(slot.query[m_set], GL_QUERY_RESULT_AVAILABLE, &available);
        if (!available)
        {
            stages.clear();
            break;
        }
        GLuint64 ns = 0;
        glGetQueryObjectui64v(slot.query[m_set], GL_QUERY_RESULT, &ns);
        slot.issued[m_set] = false;
        stages.push_back({slot.name, static_cast<float>(ns) * 1e-6f});
    }
    if (!stages.empty())
        m_stages = std::move(stages);
    m_cursor = 0;
}

void GpuQueryTimer::Begin(const char* name)
{
    if (m_open)
        End();
    if (m_cursor >= static_cast<int>(m_slots.size()))
    {
        Slot slot;
        glGenQueries(2, slot.query);
        m_slots.push_back(slot);
    }
    Slot& slot = m_slots[m_cursor];
    slot.name = name;
    glBeginQuery(GL_TIME_ELAPSED, slot.query[m_set]);
    slot.issued[m_set] = true;
    m_open = true;
}

void GpuQueryTimer::End()
{
    if (!m_open)
        return;
    glEndQuery(GL_TIME_ELAPSED);
    m_open = false;
    ++m_cursor;
}

void GpuQueryTimer::Destroy()
{
    if (m_open)
    {
        glEndQuery(GL_TIME_ELAPSED);
        m_open = false;
    }
    for (Slot& slot : m_slots)
        glDeleteQueries(2, slot.query);
    m_slots.clear();
    m_stages.clear();
    m_cursor = 0;
}

float GpuQueryTimer::TotalMs() const
{
    float total = 0.0f;
    for (const Stage& s : m_stages)
        total += s.ms;
    return total;
}
