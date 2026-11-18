#include "pch.h"
#include "fluid/FluidSim.h"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <glad/glad.h>

#include "fluid/cuda/Seeding.h"

namespace Fluid
{
    static_assert(kMaxFluidShapes == kMaxShapes);

    namespace
    {
        constexpr float kFrameSeconds = 1.0f / 60.0f;
        constexpr int kInitialBrickCapacity = 4096;

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
        if (!CreateRenderBuffer(m_particleCount) || !CreateBrickBuffers() ||
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
        DestroyBrickBuffers();
        m_subcells.Release();
        m_atlas.Release();
        m_topology.Release();
        m_uncovered = 0;
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
        {
            PublishPositions();
            RebuildTopology();
        }
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

    bool FluidSim::CreateBrickBuffers()
    {
        std::vector<float> corners;
        for (int a = 0; a < 8; ++a)
        {
            for (int bit = 1; bit < 8; bit <<= 1)
            {
                if (a & bit)
                    continue;
                for (int corner : {a, a | bit})
                {
                    corners.push_back((corner & 1) ? 1.0f : 0.0f);
                    corners.push_back((corner & 2) ? 1.0f : 0.0f);
                    corners.push_back((corner & 4) ? 1.0f : 0.0f);
                }
            }
        }

        glGenVertexArrays(1, &m_brickVAO);
        glGenBuffers(1, &m_brickCornerBuffer);
        glGenBuffers(1, &m_brickNodeBuffer);
        glBindVertexArray(m_brickVAO);

        glBindBuffer(GL_ARRAY_BUFFER, m_brickCornerBuffer);
        glBufferData(GL_ARRAY_BUFFER,
                     static_cast<GLsizeiptr>(corners.size() * sizeof(float)),
                     corners.data(),
                     GL_STATIC_DRAW);
        glEnableVertexAttribArray(0);
        glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), nullptr);

        glBindBuffer(GL_ARRAY_BUFFER, m_brickNodeBuffer);
        glBufferData(GL_ARRAY_BUFFER,
                     static_cast<GLsizeiptr>(kInitialBrickCapacity) * static_cast<GLsizeiptr>(sizeof(Node)),
                     nullptr,
                     GL_DYNAMIC_DRAW);
        glEnableVertexAttribArray(1);
        glVertexAttribIPointer(1, 1, GL_INT, sizeof(Node), reinterpret_cast<void*>(offsetof(Node, level)));
        glVertexAttribDivisor(1, 1);
        glEnableVertexAttribArray(2);
        glVertexAttribIPointer(2, 3, GL_INT, sizeof(Node), reinterpret_cast<void*>(offsetof(Node, coord)));
        glVertexAttribDivisor(2, 1);

        glBindVertexArray(0);
        glBindBuffer(GL_ARRAY_BUFFER, 0);

        m_brickNodeCapacity = kInitialBrickCapacity;
        m_brickInstanceCount = 0;
        return m_brickInterop.Register(m_brickNodeBuffer, cudaGraphicsRegisterFlagsWriteDiscard);
    }

    void FluidSim::DestroyBrickBuffers()
    {
        m_brickInterop.Unregister();
        if (m_brickVAO)
        {
            glDeleteVertexArrays(1, &m_brickVAO);
            m_brickVAO = 0;
        }
        for (unsigned int* buffer : {&m_brickCornerBuffer, &m_brickNodeBuffer})
        {
            if (*buffer)
            {
                glDeleteBuffers(1, buffer);
                *buffer = 0;
            }
        }
        m_brickNodeCapacity = 0;
        m_brickInstanceCount = 0;
    }

    void FluidSim::RebuildTopology()
    {
        VoxelSpace space;
        space.origin = m_grid.origin;
        space.h = m_grid.h;
        const float influence = m_grid.h;

        const bool rebuilt =
            incrementalTopology
                ? m_topology.Update(m_positions.Data(), m_particleCount, space, influence, &m_timer)
                : m_topology.FullRebuild(m_positions.Data(), m_particleCount, space, influence, &m_timer);
        if (!rebuilt)
        {
            m_uncovered = m_particleCount;
            m_brickInstanceCount = 0;
            return;
        }
        m_uncovered = m_topology.CountUncovered(m_positions.Data(), m_particleCount, influence);
        PublishNodes();

        if (m_atlas.Reserve(m_topology.BrickCapacity()) && m_atlas.BuildBrickTable(m_topology))
        {
            m_timer.Begin("Atlas clear");
            m_atlas.ClearActive();
            m_timer.End();
        }

        m_subcells.Build(m_positions.Data(), m_velocities.Data(), m_particleCount, m_topology, influence, &m_timer);
    }

    void FluidSim::PublishNodes()
    {
        const int nodes = m_topology.NodeSlots();
        if (nodes > m_brickNodeCapacity)
        {
            m_brickInterop.Unregister();
            const int capacity = std::max(nodes, 2 * m_brickNodeCapacity);
            glBindBuffer(GL_ARRAY_BUFFER, m_brickNodeBuffer);
            glBufferData(GL_ARRAY_BUFFER,
                         static_cast<GLsizeiptr>(capacity) * static_cast<GLsizeiptr>(sizeof(Node)),
                         nullptr,
                         GL_DYNAMIC_DRAW);
            glBindBuffer(GL_ARRAY_BUFFER, 0);
            m_brickNodeCapacity = capacity;
            if (!m_brickInterop.Register(m_brickNodeBuffer, cudaGraphicsRegisterFlagsWriteDiscard))
            {
                m_brickInstanceCount = 0;
                return;
            }
        }

        size_t bytes = 0;
        void* destination = m_brickInterop.Map(bytes);
        if (!destination)
        {
            m_brickInstanceCount = 0;
            return;
        }
        CUDA_CHECK(cudaMemcpyAsync(destination,
                                   m_topology.Nodes(),
                                   static_cast<size_t>(nodes) * sizeof(Node),
                                   cudaMemcpyDeviceToDevice));
        m_brickInterop.Unmap();
        m_brickInstanceCount = nodes;
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
