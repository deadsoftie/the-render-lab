#include "pch.h"
#include "fluid/FluidSim.h"

#include <algorithm>
#include <glad/glad.h>

#include "fluid/cuda/ParticleKernels.h"

namespace Fluid
{
    namespace
    {
        constexpr float kStepSeconds = 1.0f / 60.0f;
        constexpr float kRestitution = 0.3f;
        const float3 kGravity = make_float3(0.0f, -9.81f, 0.0f);

        float3 ToFloat3(const glm::vec3& v)
        {
            return make_float3(v.x, v.y, v.z);
        }
    }

    bool FluidSim::InitDevice()
    {
        std::string error;
        m_deviceReady = SelectGLDevice(m_device, error);
        m_status = error;
        return m_deviceReady;
    }

    bool FluidSim::Load(const FluidBinding& binding)
    {
        Unload();
        if (!m_deviceReady)
            return false;

        m_binding = binding;
        m_particleCount = binding.particleCount;
        const size_t count = static_cast<size_t>(m_particleCount);

        if (!m_positions.Resize(count) || !m_velocities.Resize(count) ||
            !CreateRenderBuffer(m_particleCount) || !Seed())
        {
            Unload();
            m_status = "Failed to allocate " + std::to_string(binding.particleCount) + " particles";
            return false;
        }

        m_status.clear();
        m_loaded = true;
        playing = true;
        stepRequested = false;
        return true;
    }

    void FluidSim::Unload()
    {
        DestroyRenderBuffer();
        m_positions.Free();
        m_velocities.Free();
        m_timer.Reset();
        m_particleCount = 0;
        m_renderDirty = false;
        m_loaded = false;
    }

    void FluidSim::Shutdown()
    {
        Unload();
    }

    void FluidSim::Reset()
    {
        if (m_loaded)
            Seed();
    }

    void FluidSim::Update(float deltaSeconds)
    {
        if (!m_loaded)
            return;

        m_timer.BeginFrame();

        if (playing || stepRequested)
        {
            const float dt = playing ? deltaSeconds : kStepSeconds;
            const Box bounds{ToFloat3(m_binding.domainMin), ToFloat3(m_binding.domainMax)};
            m_timer.Begin("Particles");
            LaunchBallisticStep(m_positions.Data(),
                                m_velocities.Data(),
                                m_particleCount,
                                bounds,
                                kGravity,
                                dt,
                                kRestitution);
            m_timer.End();
            m_renderDirty = true;
            stepRequested = false;
        }

        if (m_renderDirty)
            PublishPositions();
    }

    bool FluidSim::CreateRenderBuffer(int count)
    {
        glGenBuffers(1, &m_renderBuffer);
        glBindBuffer(GL_ARRAY_BUFFER, m_renderBuffer);
        glBufferData(GL_ARRAY_BUFFER,
                     static_cast<GLsizeiptr>(count) * static_cast<GLsizeiptr>(sizeof(float4)),
                     nullptr,
                     GL_DYNAMIC_DRAW);

        glGenVertexArrays(1, &m_renderVAO);
        glBindVertexArray(m_renderVAO);
        glEnableVertexAttribArray(0);
        glVertexAttribPointer(0, 4, GL_FLOAT, GL_FALSE, sizeof(float4), nullptr);
        glBindVertexArray(0);
        glBindBuffer(GL_ARRAY_BUFFER, 0);

        return m_renderInterop.Register(m_renderBuffer, cudaGraphicsRegisterFlagsWriteDiscard);
    }

    void FluidSim::DestroyRenderBuffer()
    {
        m_renderInterop.Unregister();
        if (m_renderVAO)
        {
            glDeleteVertexArrays(1, &m_renderVAO);
            m_renderVAO = 0;
        }
        if (m_renderBuffer)
        {
            glDeleteBuffers(1, &m_renderBuffer);
            m_renderBuffer = 0;
        }
    }

    bool FluidSim::Seed()
    {
        const Box seed{ToFloat3(m_binding.seedMin), ToFloat3(m_binding.seedMax)};
        if (!LaunchSeedParticles(m_positions.Data(), m_velocities.Data(), m_particleCount, seed))
            return false;
        m_renderDirty = true;
        return true;
    }

    void FluidSim::PublishPositions()
    {
        size_t bytes = 0;
        void* destination = m_renderInterop.Map(bytes);
        if (!destination)
            return;

        m_timer.Begin("Copy to GL");
        CUDA_CHECK(cudaMemcpyAsync(destination,
                                   m_positions.Data(),
                                   std::min(bytes, m_positions.Bytes()),
                                   cudaMemcpyDeviceToDevice));
        m_timer.End();

        m_renderInterop.Unmap();
        m_renderDirty = false;
    }
}
