#include "pch.h"
#include "fluid/FluidSim.h"

#include <algorithm>
#include <cmath>
#include <glad/glad.h>

#include "fluid/cuda/Seeding.h"

namespace Fluid
{
    static_assert(kMaxFluidShapes == kMaxShapes);

    namespace
    {
        constexpr float kFrameSeconds = 1.0f / 60.0f;

        float3 ToFloat3(const glm::vec3& v)
        {
            return make_float3(v.x, v.y, v.z);
        }

        ShapeList ToShapeList(const std::vector<FluidShape>& shapes)
        {
            ShapeList list;
            for (const FluidShape& source : shapes)
            {
                Shape& shape = list.items[list.count++];
                shape.type = source.type == FluidShapeType::Box ? ShapeType::Box : ShapeType::Sphere;
                shape.lo = ToFloat3(source.boxMin);
                shape.hi = ToFloat3(source.boxMax);
                shape.center = ToFloat3(source.center);
                shape.radius = source.radius;
                shape.velocity = ToFloat3(source.velocity);
            }
            return list;
        }

        GridInfo BuildGrid(const FluidBinding& binding)
        {
            const glm::vec3 extent = binding.domainMax - binding.domainMin;
            const float longest = std::max({extent.x, extent.y, extent.z});
            const float h = longest / static_cast<float>(binding.gridResolution);

            GridInfo grid;
            grid.h = h;
            grid.origin = ToFloat3(binding.domainMin);
            grid.dims = make_int3(std::max(2, static_cast<int>(std::ceil(extent.x / h - 1e-3f))),
                                  std::max(2, static_cast<int>(std::ceil(extent.y / h - 1e-3f))),
                                  std::max(2, static_cast<int>(std::ceil(extent.z / h - 1e-3f))));
            return grid;
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
        m_grid = BuildGrid(binding);
        m_seeds = ToShapeList(binding.seeds);
        m_obstacles = ToShapeList(binding.obstacles);

        if (!Seed() || m_particleCount == 0)
        {
            Unload();
            m_status = "No particles seeded; check the seed shapes against the domain";
            return false;
        }
        if (!CreateRenderBuffer(m_particleCount) ||
            !m_solver.Init(m_grid, m_obstacles, m_particleCount))
        {
            Unload();
            m_status = "Failed to allocate the fluid simulation";
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
        m_solver.Release();
        m_positions.Free();
        m_velocities.Free();
        m_timer.Reset();
        m_lastStats = StepStats{};
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
        if (!m_loaded)
            return;
        Seed();
        m_lastStats = StepStats{};
    }

    void FluidSim::Update()
    {
        if (!m_loaded)
            return;

        m_timer.BeginFrame();

        if (playing || stepRequested)
        {
            m_lastStats = m_solver.Step(
                m_positions, m_velocities, m_particleCount, kFrameSeconds, settings, &m_timer);
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
        int count = 0;
        if (!SeedParticles(m_grid, m_seeds, m_obstacles, m_positions, m_velocities, count))
            return false;
        m_particleCount = count;
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
