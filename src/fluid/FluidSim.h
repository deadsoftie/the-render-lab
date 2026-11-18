#pragma once
#include <string>

#include "fluid/CudaContext.h"
#include "fluid/GlInterop.h"
#include "fluid/cuda/BrickAtlas.h"
#include "fluid/cuda/DenseFlip.h"
#include "fluid/cuda/DeviceBuffer.h"
#include "fluid/cuda/FluidTypes.h"
#include "fluid/cuda/GpuTimer.h"
#include "fluid/cuda/SparseTopology.h"
#include "scene/Scene.h"

namespace Fluid
{
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
        const SparseTopology& Topology() const { return m_topology; }
        const BrickAtlas& Atlas() const { return m_atlas; }
        int Uncovered() const { return m_uncovered; }
        unsigned int BrickVAO() const { return m_brickVAO; }
        int BrickInstanceCount() const { return m_brickInstanceCount; }

        StepSettings settings;
        bool incrementalTopology = true;
        bool playing = true;
        bool stepRequested = false;

       private:
        bool CreateRenderBuffer(int count);
        void DestroyRenderBuffer();
        bool CreateBrickBuffers();
        void DestroyBrickBuffers();
        bool Seed();
        void PublishPositions();
        void RebuildTopology();
        void PublishNodes();

        CudaDeviceInfo m_device;
        bool m_deviceReady = false;
        bool m_loaded = false;
        std::string m_status;
        FluidBinding m_binding;

        GridInfo m_grid;
        ShapeList m_seeds;
        ShapeList m_obstacles;
        DenseFlip m_solver;
        StepStats m_lastStats;

        int m_particleCount = 0;
        DeviceBuffer<float4> m_positions;
        DeviceBuffer<float4> m_velocities;

        unsigned int m_renderBuffer = 0;
        unsigned int m_renderVAO = 0;
        InteropBuffer m_renderInterop;
        bool m_renderDirty = false;

        SparseTopology m_topology;
        BrickAtlas m_atlas;
        int m_uncovered = 0;
        unsigned int m_brickVAO = 0;
        unsigned int m_brickCornerBuffer = 0;
        unsigned int m_brickNodeBuffer = 0;
        int m_brickNodeCapacity = 0;
        int m_brickInstanceCount = 0;
        InteropBuffer m_brickInterop;

        GpuTimer m_timer;
    };
}
