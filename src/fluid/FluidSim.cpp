#include "pch.h"
#include "fluid/FluidSim.h"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <glad/glad.h>

#include "fluid/cuda/Particles.h"
#include "fluid/cuda/Seeding.h"

namespace Fluid
{
    static_assert(kMaxFluidShapes == kMaxShapes);

    namespace
    {
        constexpr float kFrameSeconds = 1.0f / 60.0f;
        constexpr int kInitialBrickCapacity = 4096;
        constexpr int kInitialChildLists = 64;
        // Above this the dense baseline would need more than about 1 GB, so only sparse runs.
        constexpr long long kMaxDenseCells = 1ll << 24;

        bool CopyToGlBuffer(unsigned int buffer,
                            size_t& capacity,
                            InteropBuffer& interop,
                            const void* source,
                            size_t bytes)
        {
            if (bytes > capacity)
            {
                interop.Unregister();
                const size_t grown = std::max(bytes, 2 * capacity);
                glBindBuffer(GL_ARRAY_BUFFER, buffer);
                glBufferData(GL_ARRAY_BUFFER,
                             static_cast<GLsizeiptr>(grown),
                             nullptr,
                             GL_DYNAMIC_DRAW);
                glBindBuffer(GL_ARRAY_BUFFER, 0);
                capacity = grown;
                if (!interop.Register(buffer, cudaGraphicsRegisterFlagsWriteDiscard))
                    return false;
            }
            if (bytes == 0)
                return true;

            size_t mapped = 0;
            void* destination = interop.Map(mapped);
            if (!destination)
                return false;
            const bool copied =
                CUDA_CHECK(cudaMemcpyAsync(destination, source, bytes, cudaMemcpyDeviceToDevice));
            interop.Unmap();
            return copied;
        }

        float EmitterSpeed(const Shape& shape)
        {
            return std::sqrt(shape.velocity.x * shape.velocity.x +
                             shape.velocity.y * shape.velocity.y +
                             shape.velocity.z * shape.velocity.z);
        }

        // Distance the emitter velocity must carry a fill before the shape is refilled: the
        // shape's extent along the velocity direction.
        float EmitterLength(const Shape& shape)
        {
            if (shape.type == ShapeType::Sphere)
                return 2.0f * shape.radius;
            const float speed = EmitterSpeed(shape);
            if (speed <= 0.0f)
                return shape.hi.y - shape.lo.y;
            return (std::fabs(shape.velocity.x) * (shape.hi.x - shape.lo.x) +
                    std::fabs(shape.velocity.y) * (shape.hi.y - shape.lo.y) +
                    std::fabs(shape.velocity.z) * (shape.hi.z - shape.lo.z)) /
                   speed;
        }

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
                shape.type =
                    source.type == FluidShapeType::Box ? ShapeType::Box : ShapeType::Sphere;
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
    }  // namespace

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
        m_emitters = ToShapeList(binding.emitters);
        m_denseAvailable =
            static_cast<long long>(m_grid.dims.x) * m_grid.dims.y * m_grid.dims.z <= kMaxDenseCells;
        if (!m_denseAvailable)
            backend = Backend::Sparse;
        m_sparse.Init(m_grid, m_obstacles);

        if (!Seed() || (m_particleCount == 0 && m_emitters.count == 0))
        {
            Unload();
            m_status = "No particles seeded; check the seed shapes against the domain";
            return false;
        }
        m_maxParticles = m_emitters.count > 0 ? std::max(binding.maxParticles, m_particleCount)
                                              : m_particleCount;
        if (!CreateRenderBuffer(m_maxParticles) || !CreateTreeBuffers() ||
            (m_denseAvailable && !m_solver.Init(m_grid, m_obstacles, m_particleCount)))
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
        DestroyTreeBuffers();
        m_sparse.Release();
        m_topologyValid = false;
        m_levelSetValid = false;
        m_uncovered = 0;
        m_solver.Release();
        m_positions.Free();
        m_velocities.Free();
        m_speedScratch.Free();
        m_timer.Reset();
        m_lastStats = StepStats{};
        for (BackendSnapshot& snapshot : m_snapshots)
            snapshot = BackendSnapshot{};
        m_particleCount = 0;
        m_maxParticles = 0;
        m_emitters = ShapeList{};
        m_denseAvailable = false;
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

        if (!m_denseAvailable)
            backend = Backend::Sparse;
        if (backend != m_timedBackend)
        {
            m_timer.Reset();
            m_timedBackend = backend;
        }
        m_timer.BeginFrame();
        if (surfaceView != m_lastSurfaceView)
        {
            m_lastSurfaceView = surfaceView;
            m_renderDirty = true;
        }

        // Alg. 1: one max-speed readback picks the substep count for the whole frame, then each
        // substep emits and runs the active backend. GL buffers are published once at the end.
        bool topologyFresh = false;
        if (playing || stepRequested)
        {
            StepStats stats;
            stats.maxSpeed = MaxParticleSpeed(m_velocities.Data(), m_particleCount, m_speedScratch);
            const int substeps =
                SubstepCount(stats.maxSpeed, m_grid.h, settings.gravity, kFrameSeconds);
            const float dt = kFrameSeconds / static_cast<float>(substeps);
            for (int s = 0; s < substeps; ++s)
            {
                if (!Emit(dt) || m_particleCount == 0)
                    break;
                CgResult cg;
                const bool ok = backend == Backend::Dense ? DenseSubstep(dt, cg)
                                                          : SparseSubstep(dt, s == 0, cg);
                topologyFresh = backend == Backend::Sparse && m_topologyValid;
                if (!ok)
                    break;
                stats.cg = cg;
                stats.cgIterationsTotal += cg.iterations;
                ++stats.substeps;
            }
            m_lastStats = stats;
            BackendSnapshot& snapshot = m_snapshots[static_cast<int>(backend)];
            snapshot.valid = true;
            snapshot.stages = m_timer.Stages();
            snapshot.stats = stats;
            m_renderDirty = true;
            stepRequested = false;
        }

        if (m_renderDirty)
        {
            PublishPositions();
            if (!topologyFresh)
                RefreshGrid();
            PublishTree();
            PublishLevelSet();
        }
    }

    MemoryReport FluidSim::Memory() const
    {
        MemoryReport report;
        report.particles = m_positions.Bytes() + m_velocities.Bytes();
        report.topology = m_sparse.Topology().MemoryBytes();
        report.atlas = m_sparse.Atlas().MemoryBytes();
        report.subcells = m_sparse.Subcells().MemoryBytes();
        report.dense = m_denseAvailable ? m_solver.MemoryBytes() : 0;
        report.cells = static_cast<long long>(m_grid.dims.x) * m_grid.dims.y * m_grid.dims.z;
        return report;
    }

    bool FluidSim::Emit(float dt)
    {
        for (int i = 0; i < m_emitters.count; ++i)
        {
            const Shape& shape = m_emitters.items[i];
            const float length = EmitterLength(shape);
            m_emitTravel[i] += EmitterSpeed(shape) * dt;
            if (m_emitTravel[i] < length)
                continue;
            m_emitTravel[i] -= length;

            ShapeList single;
            single.items[single.count++] = shape;
            if (!AppendParticles(m_grid,
                                 single,
                                 m_obstacles,
                                 m_positions,
                                 m_velocities,
                                 m_particleCount,
                                 m_maxParticles,
                                 m_emitSalt++))
                return false;
        }
        return true;
    }

    bool FluidSim::DenseSubstep(float dt, CgResult& cg)
    {
        if (!m_solver.Transfer(m_positions,
                               m_velocities,
                               m_particleCount,
                               dt,
                               settings.gravity,
                               &m_timer))
            return false;
        cg = m_solver.Project(settings.cgTolerance, settings.cgMaxIterations, &m_timer);
        return m_solver
            .Advect(m_positions, m_velocities, m_particleCount, dt, settings.flipRatio, &m_timer);
    }

    bool FluidSim::SparseSubstep(float dt, bool countUncovered, CgResult& cg)
    {
        m_levelSetValid = false;
        m_topologyValid =
            m_sparse.Rebuild(m_positions.Data(), m_particleCount, incrementalTopology, &m_timer);
        if (!m_topologyValid)
        {
            m_uncovered = m_particleCount;
            return false;
        }
        if (countUncovered)
            m_uncovered = m_sparse.CountUncovered(m_positions.Data(), m_particleCount);
        m_levelSetValid =
            m_sparse.Transfer(m_positions.Data(), m_velocities.Data(), m_particleCount, &m_timer);
        if (!m_levelSetValid)
            return false;
        cg = m_sparse.Project(dt,
                              settings.gravity,
                              settings.cgTolerance,
                              settings.cgMaxIterations,
                              &m_timer);
        return m_sparse.Advect(m_positions.Data(),
                               m_velocities.Data(),
                               m_particleCount,
                               dt,
                               settings.flipRatio,
                               &m_timer);
    }

    // Rebuilds the sparse grid from the current particles when no sparse substep ran this frame
    // (dense backend, paused, reset), so the overlay and the raymarched surface stay current.
    void FluidSim::RefreshGrid()
    {
        m_levelSetValid = false;
        m_topologyValid =
            m_particleCount > 0 &&
            m_sparse.Rebuild(m_positions.Data(), m_particleCount, incrementalTopology, &m_timer);
        m_uncovered = m_topologyValid ? m_sparse.CountUncovered(m_positions.Data(), m_particleCount)
                                      : m_particleCount;
        if (m_topologyValid && surfaceView)
            m_levelSetValid = m_sparse.Transfer(m_positions.Data(),
                                                m_velocities.Data(),
                                                m_particleCount,
                                                &m_timer);
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

    bool FluidSim::CreateTreeBuffers()
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
        glBufferData(
            GL_ARRAY_BUFFER,
            static_cast<GLsizeiptr>(kInitialBrickCapacity) * static_cast<GLsizeiptr>(sizeof(Node)),
            nullptr,
            GL_DYNAMIC_DRAW);
        glEnableVertexAttribArray(1);
        glVertexAttribIPointer(1,
                               1,
                               GL_INT,
                               sizeof(Node),
                               reinterpret_cast<void*>(offsetof(Node, level)));
        glVertexAttribDivisor(1, 1);
        glEnableVertexAttribArray(2);
        glVertexAttribIPointer(2,
                               3,
                               GL_INT,
                               sizeof(Node),
                               reinterpret_cast<void*>(offsetof(Node, coord)));
        glVertexAttribDivisor(2, 1);

        glBindVertexArray(0);

        m_brickNodeCapacity = static_cast<size_t>(kInitialBrickCapacity) * sizeof(Node);
        m_childListCapacity = static_cast<size_t>(kInitialChildLists) * kChildCount * sizeof(int);
        glGenBuffers(1, &m_childListBuffer);
        glBindBuffer(GL_ARRAY_BUFFER, m_childListBuffer);
        glBufferData(GL_ARRAY_BUFFER,
                     static_cast<GLsizeiptr>(m_childListCapacity),
                     nullptr,
                     GL_DYNAMIC_DRAW);
        glBindBuffer(GL_ARRAY_BUFFER, 0);

        m_brickInstanceCount = 0;
        return m_brickInterop.Register(m_brickNodeBuffer, cudaGraphicsRegisterFlagsWriteDiscard) &&
               m_childListInterop.Register(m_childListBuffer,
                                           cudaGraphicsRegisterFlagsWriteDiscard);
    }

    void FluidSim::DestroyTreeBuffers()
    {
        m_brickInterop.Unregister();
        m_childListInterop.Unregister();
        m_levelSetInterop.Unregister();
        if (m_brickVAO)
        {
            glDeleteVertexArrays(1, &m_brickVAO);
            m_brickVAO = 0;
        }
        for (unsigned int* buffer : {&m_brickCornerBuffer, &m_brickNodeBuffer, &m_childListBuffer})
        {
            if (*buffer)
            {
                glDeleteBuffers(1, buffer);
                *buffer = 0;
            }
        }
        if (m_levelSetTexture)
        {
            glDeleteTextures(1, &m_levelSetTexture);
            m_levelSetTexture = 0;
        }
        m_levelSetDims = make_int3(0, 0, 0);
        m_brickNodeCapacity = 0;
        m_childListCapacity = 0;
        m_brickInstanceCount = 0;
        m_treeReady = false;
        m_surfaceReady = false;
    }

    void FluidSim::PublishTree()
    {
        m_brickInstanceCount = 0;
        m_treeReady = false;
        if (!m_topologyValid)
            return;

        const SparseTopology& topology = m_sparse.Topology();
        const size_t nodeBytes = static_cast<size_t>(topology.NodeSlots()) * sizeof(Node);
        const size_t listBytes =
            static_cast<size_t>(topology.ChildListSlots()) * kChildCount * sizeof(int);
        if (!CopyToGlBuffer(m_brickNodeBuffer,
                            m_brickNodeCapacity,
                            m_brickInterop,
                            topology.Nodes(),
                            nodeBytes) ||
            !CopyToGlBuffer(m_childListBuffer,
                            m_childListCapacity,
                            m_childListInterop,
                            topology.ChildLists(),
                            listBytes))
            return;
        m_brickInstanceCount = topology.NodeSlots();
        m_treeReady = true;
    }

    // Copies the atlas level-set channel into the GL 3D texture with the same layout, so the
    // shader can use the node buffer's brick slots directly. The texture follows the atlas size.
    void FluidSim::PublishLevelSet()
    {
        m_surfaceReady = false;
        if (!surfaceView || !m_levelSetValid || !m_treeReady)
            return;

        const BrickAtlas& atlas = m_sparse.Atlas();
        const int3 dims = atlas.TexelDims();
        if (dims.x != m_levelSetDims.x || dims.y != m_levelSetDims.y || dims.z != m_levelSetDims.z)
        {
            m_levelSetInterop.Unregister();
            if (m_levelSetTexture)
                glDeleteTextures(1, &m_levelSetTexture);
            glGenTextures(1, &m_levelSetTexture);
            glBindTexture(GL_TEXTURE_3D, m_levelSetTexture);
            glTexImage3D(GL_TEXTURE_3D,
                         0,
                         GL_R32F,
                         dims.x,
                         dims.y,
                         dims.z,
                         0,
                         GL_RED,
                         GL_FLOAT,
                         nullptr);
            glTexParameteri(GL_TEXTURE_3D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
            glTexParameteri(GL_TEXTURE_3D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
            glTexParameteri(GL_TEXTURE_3D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
            glTexParameteri(GL_TEXTURE_3D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
            glTexParameteri(GL_TEXTURE_3D, GL_TEXTURE_WRAP_R, GL_CLAMP_TO_EDGE);
            glBindTexture(GL_TEXTURE_3D, 0);
            m_levelSetDims = make_int3(0, 0, 0);
            if (!m_levelSetInterop.Register(m_levelSetTexture,
                                            GL_TEXTURE_3D,
                                            cudaGraphicsRegisterFlagsWriteDiscard))
                return;
            m_levelSetDims = dims;
        }

        cudaArray_t destination = m_levelSetInterop.Map();
        if (!destination)
            return;
        m_timer.Begin("Level set to GL");
        cudaMemcpy3DParms copy{};
        copy.srcArray = atlas.Array(AtlasChannel::LevelSet);
        copy.dstArray = destination;
        copy.extent = make_cudaExtent(dims.x, dims.y, dims.z);
        copy.kind = cudaMemcpyDeviceToDevice;
        const bool copied = CUDA_CHECK(cudaMemcpy3DAsync(&copy));
        m_timer.End();
        m_levelSetInterop.Unmap();
        m_surfaceReady = copied;
    }

    bool FluidSim::Seed()
    {
        int count = 0;
        if (!SeedParticles(m_grid, m_seeds, m_obstacles, m_positions, m_velocities, count))
            return false;
        m_particleCount = count;
        for (int i = 0; i < m_emitters.count; ++i)
            m_emitTravel[i] = EmitterLength(m_emitters.items[i]);
        m_emitSalt = 1;
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
}  // namespace Fluid
