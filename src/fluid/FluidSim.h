#pragma once
#include <cstddef>
#include <string>

#include "fluid/CudaContext.h"
#include "fluid/GlInterop.h"
#include "fluid/cuda/DenseFlip.h"
#include "fluid/cuda/DeviceBuffer.h"
#include "fluid/cuda/FluidTypes.h"
#include "fluid/cuda/GpuTimer.h"
#include "fluid/cuda/SparseFlip.h"
#include "scene/Scene.h"

namespace Fluid
{
    enum class Backend
    {
        Dense,
        Sparse
    };

    class FluidSim
    {
       public:
        FluidSim() = default;

        FluidSim(const FluidSim&) = delete;
        FluidSim& operator=(const FluidSim&) = delete;

        bool InitDevice();
        bool Load(const FluidBinding& binding);
        void Unload();
        void Shutdown();
        void Reset();
        void Update();

        bool IsDeviceReady() const { return m_deviceReady; }
        bool IsLoaded() const { return m_loaded; }
        const CudaDeviceInfo& Device() const { return m_device; }
        const std::string& Status() const { return m_status; }
        const FluidBinding& Binding() const { return m_binding; }
        const GridInfo& Grid() const { return m_grid; }
        const StepStats& LastStats() const { return m_lastStats; }
        const GpuTimer& Timer() const { return m_timer; }
        unsigned int RenderVAO() const { return m_renderVAO; }
        int ParticleCount() const { return m_particleCount; }
        int MaxParticles() const { return m_maxParticles; }
        int EmitterCount() const { return m_emitters.count; }
        bool DenseAvailable() const { return m_denseAvailable; }
        const SparseTopology& Topology() const { return m_sparse.Topology(); }
        const BrickAtlas& Atlas() const { return m_sparse.Atlas(); }
        const SubcellLists& Subcells() const { return m_sparse.Subcells(); }
        int Uncovered() const { return m_uncovered; }
        unsigned int BrickVAO() const { return m_brickVAO; }
        int BrickInstanceCount() const { return m_brickInstanceCount; }
        unsigned int NodeBuffer() const { return m_brickNodeBuffer; }
        unsigned int ChildListBuffer() const { return m_childListBuffer; }
        unsigned int LevelSetTexture() const { return m_levelSetTexture; }
        int3 LevelSetDims() const { return m_levelSetDims; }
        bool SurfaceReady() const { return m_surfaceReady; }

        StepSettings settings;
        Backend backend = Backend::Sparse;
        bool incrementalTopology = true;
        bool surfaceView = true;
        bool playing = true;
        bool stepRequested = false;

       private:
        bool CreateRenderBuffer(int count);
        void DestroyRenderBuffer();
        bool CreateTreeBuffers();
        void DestroyTreeBuffers();
        bool Seed();
        bool Emit(float dt);
        bool DenseSubstep(float dt, CgResult& cg);
        bool SparseSubstep(float dt, bool countUncovered, CgResult& cg);
        void RefreshGrid();
        void PublishPositions();
        void PublishTree();
        void PublishLevelSet();

        CudaDeviceInfo m_device;
        bool m_deviceReady = false;
        bool m_loaded = false;
        std::string m_status;
        FluidBinding m_binding;

        GridInfo m_grid;
        ShapeList m_seeds;
        ShapeList m_obstacles;
        ShapeList m_emitters;
        float m_emitTravel[kMaxShapes] = {};
        unsigned int m_emitSalt = 1;
        bool m_denseAvailable = false;
        DenseFlip m_solver;
        SparseFlip m_sparse;
        StepStats m_lastStats;
        Backend m_timedBackend = Backend::Sparse;

        int m_particleCount = 0;
        int m_maxParticles = 0;
        DeviceBuffer<float4> m_positions;
        DeviceBuffer<float4> m_velocities;
        DeviceBuffer<unsigned int> m_speedScratch;

        unsigned int m_renderBuffer = 0;
        unsigned int m_renderVAO = 0;
        InteropBuffer m_renderInterop;
        bool m_renderDirty = false;

        bool m_topologyValid = false;
        bool m_levelSetValid = false;
        bool m_treeReady = false;
        bool m_surfaceReady = false;
        bool m_lastSurfaceView = true;
        int m_uncovered = 0;
        unsigned int m_brickVAO = 0;
        unsigned int m_brickCornerBuffer = 0;
        unsigned int m_brickNodeBuffer = 0;
        size_t m_brickNodeCapacity = 0;
        int m_brickInstanceCount = 0;
        InteropBuffer m_brickInterop;
        unsigned int m_childListBuffer = 0;
        size_t m_childListCapacity = 0;
        InteropBuffer m_childListInterop;
        unsigned int m_levelSetTexture = 0;
        int3 m_levelSetDims{0, 0, 0};
        InteropImage m_levelSetInterop;

        GpuTimer m_timer;
    };
}
