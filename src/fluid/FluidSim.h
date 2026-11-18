#pragma once
#include <string>

#include "fluid/CudaContext.h"
#include "fluid/GlInterop.h"
#include "fluid/cuda/DeviceBuffer.h"
#include "fluid/cuda/GpuTimer.h"
#include "scene/Scene.h"

namespace Fluid
{
    struct SolverSettings
    {
        float flipRatio = 0.95f;
        float cgTolerance = 1e-4f;
        int cgMaxIterations = 200;
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
        void Update(float deltaSeconds);

        bool IsDeviceReady() const { return m_deviceReady; }
        bool IsLoaded() const { return m_loaded; }
        const CudaDeviceInfo& Device() const { return m_device; }
        const std::string& Status() const { return m_status; }
        const FluidBinding& Binding() const { return m_binding; }
        const GpuTimer& Timer() const { return m_timer; }
        unsigned int RenderVAO() const { return m_renderVAO; }
        int ParticleCount() const { return m_particleCount; }

        SolverSettings settings;
        bool playing = true;
        bool stepRequested = false;

       private:
        bool CreateRenderBuffer(int count);
        void DestroyRenderBuffer();
        bool Seed();
        void PublishPositions();

        CudaDeviceInfo m_device;
        bool m_deviceReady = false;
        bool m_loaded = false;
        std::string m_status;
        FluidBinding m_binding;
        int m_particleCount = 0;

        DeviceBuffer<float4> m_positions;
        DeviceBuffer<float4> m_velocities;

        unsigned int m_renderBuffer = 0;
        unsigned int m_renderVAO = 0;
        InteropBuffer m_renderInterop;
        bool m_renderDirty = false;

        GpuTimer m_timer;
    };
}
