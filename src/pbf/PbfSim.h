#pragma once
#include <string>
#include <vector>

#include <glm/glm.hpp>

#include "graphics/resources/GpuBuffer.h"
#include "graphics/resources/GpuQueryTimer.h"
#include "pbf/PbfParams.h"
#include "pbf/PbfSeeding.h"
#include "scene/Scene.h"

namespace Pbf
{
    float RestLatticeDensitySum(float radius, float smoothing);

    class PbfSim
    {
       public:
        PbfSim() = default;

        PbfSim(const PbfSim&) = delete;
        PbfSim& operator=(const PbfSim&) = delete;

        bool Load(const PbfBinding& binding);
        void Unload();
        void Reset();
        void Update(float deltaSeconds);
        void Shutdown();

        bool IsLoaded() const { return m_loaded; }
        const std::string& Status() const { return m_status; }
        const PbfBinding& Binding() const { return m_binding; }
        const ParamsGpu& Params() const { return m_params; }
        const GpuQueryTimer& Timer() const { return m_timer; }
        int ParticleCount() const { return m_count; }
        int MaxParticles() const { return m_maxParticles; }
        float Radius() const { return m_binding.particleRadius; }
        float SmoothingRadius() const { return m_h; }
        unsigned int ParticleVAO() const { return m_vao; }
        unsigned int PositionBuffer() const { return m_positions.ID(); }

        Settings settings;
        bool paused = false;

       private:
        void Seed();
        void UploadParams();
        void BuildVao();
        void DestroyVao();

        PbfBinding m_binding;
        ParamsGpu m_params;
        GpuBuffer m_paramsBuffer;
        GpuBuffer m_positions;
        GpuBuffer m_predicted;
        GpuBuffer m_velocities;
        GpuBuffer m_sortedPositions;
        GpuBuffer m_sortedPredicted;
        GpuBuffer m_sortedVelocities;
        GpuQueryTimer m_timer;
        unsigned int m_vao = 0;
        int m_count = 0;
        int m_maxParticles = 0;
        float m_h = 0.0f;
        bool m_loaded = false;
        std::string m_status;
    };
}
