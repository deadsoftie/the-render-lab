#pragma once
#include <string>
#include <vector>

#include <glm/glm.hpp>

#include "graphics/resources/GpuBuffer.h"
#include "graphics/resources/GpuQueryTimer.h"
#include "graphics/resources/Shader.h"
#include "pbf/GpuScan.h"
#include "pbf/PbfParams.h"
#include "pbf/PbfSeeding.h"
#include "scene/Scene.h"

namespace Pbf
{
    constexpr unsigned int kCellCountBinding = 6;
    constexpr unsigned int kCellStartBinding = 7;
    constexpr unsigned int kParticleCellBinding = 8;
    constexpr unsigned int kParticleRankBinding = 9;
    constexpr unsigned int kStatsBinding = 10;
    constexpr unsigned int kNeighborCountBinding = 14;

    float RestLatticeDensitySum(float radius, float smoothing);
    float RestLatticeGradientSum(float radius, float smoothing);

    struct NeighborStats
    {
        float average = 0.0f;
        unsigned int maximum = 0;
        float densityAverage = 0.0f;
        float densityMax = 0.0f;
    };

    class PbfSim
    {
       public:
        PbfSim() = default;

        PbfSim(const PbfSim&) = delete;
        PbfSim& operator=(const PbfSim&) = delete;

        void SetShaderDir(const std::string& dir) { m_shaderDir = dir; }
        bool Load(const PbfBinding& binding);
        void Unload();
        void Reset();
        void Update(float deltaSeconds);
        void Shutdown();

        void BuildGrid();
        void Predict();
        void ComputeLambda();
        void ApplyDelta();
        void UpdateVelocities();
        void Substep();
        void StepFrame();
        void ComputeNeighborStats();
        void UploadParams();
        void UploadParticles(const std::vector<Particle>& particles);
        void DownloadPredicted(std::vector<float>& out) const;
        void DownloadPositions(std::vector<float>& out) const;
        void DownloadNeighborCounts(std::vector<unsigned int>& out) const;
        void DownloadCellCounts(std::vector<unsigned int>& out) const;
        void DownloadCellStarts(std::vector<unsigned int>& out) const;

        void BindParticleVao() const;
        void UnbindParticleVao() const;

        bool IsLoaded() const { return m_loaded; }
        const std::string& Status() const { return m_status; }
        const PbfBinding& Binding() const { return m_binding; }
        const ParamsGpu& Params() const { return m_params; }
        const GpuQueryTimer& Timer() const { return m_timer; }
        const NeighborStats& Neighbors() const { return m_neighborStats; }
        int ParticleCount() const { return m_count; }
        int MaxParticles() const { return m_maxParticles; }
        int CellCount() const { return m_params.gridDims[3]; }
        float Radius() const { return m_binding.particleRadius; }
        float SmoothingRadius() const { return m_h; }
        float TimeStep() const { return m_params.gravity[3]; }
        float SimulatedSeconds() const { return m_simSeconds; }

        Settings settings;
        bool paused = false;
        bool collectNeighborStats = true;

       private:
        enum Attribute
        {
            kPosition = 0,
            kPredicted = 1,
            kVelocity = 2,
            kAttributeCount = 3
        };

        bool LoadShaders();
        void Seed();
        void UploadShapes();
        void BindParticles(int set, unsigned int firstBinding) const;
        void BindCommon() const;
        void DispatchParticles(const Shader& shader) const;
        void DestroyVao();

        PbfBinding m_binding;
        ParamsGpu m_params;
        std::string m_shaderDir = "assets/shaders/";
        GpuBuffer m_paramsBuffer;
        GpuBuffer m_shapesBuffer;
        GpuBuffer m_particles[2][kAttributeCount];
        int m_current = 0;
        GpuBuffer m_cellCounts;
        GpuBuffer m_cellStarts;
        GpuBuffer m_particleCells;
        GpuBuffer m_particleRanks;
        GpuBuffer m_stats;
        GpuBuffer m_neighborCounts;
        GpuScan m_scan;
        Shader m_cellCountShader;
        Shader m_reorderShader;
        Shader m_neighborStatsShader;
        Shader m_predictShader;
        Shader m_lambdaShader;
        Shader m_deltaShader;
        Shader m_applyShader;
        Shader m_updateShader;
        float m_accumulator = 0.0f;
        float m_simSeconds = 0.0f;
        bool m_shadersLoaded = false;
        GpuQueryTimer m_timer;
        NeighborStats m_neighborStats;
        int m_statsFrame = 0;
        unsigned int m_vao = 0;
        int m_count = 0;
        int m_maxParticles = 0;
        float m_h = 0.0f;
        bool m_loaded = false;
        std::string m_status;
    };
}
