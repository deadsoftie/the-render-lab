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

    // Rest lattice sums of W and |grad W|^2, for mass and D0.
    float RestLatticeDensitySum(float radius, float smoothing);
    float RestLatticeGradientSum(float radius, float smoothing);

    struct NeighborStats
    {
        float average = 0.0f;
        unsigned int maximum = 0;
        float densityAverage = 0.0f;
        float densityMax = 0.0f;
        unsigned int listDropped = 0;
    };

    // Shift + drag impulse near the camera ray.
    struct MouseForce
    {
        bool active = false;
        glm::vec3 origin{0.0f};
        glm::vec3 direction{0.0f, 0.0f, -1.0f};
        glm::vec3 impulse{0.0f};
        float radius = 0.15f;
    };

    // A plank as the fluid sees it.
    struct CouplingBody
    {
        glm::vec3 com{0.0f};
        glm::vec3 cObj{0.0f};
        glm::mat3 rotation{1.0f};
        glm::vec3 halfExtents{0.1f};
        glm::vec3 velocity{0.0f};
        glm::vec3 omega{0.0f};
    };

    struct BodyForce
    {
        glm::vec3 force{0.0f};
        glm::vec3 torque{0.0f};
    };

    struct CouplingSettings
    {
        bool twoWay = true;  // impulse exchange
        float friction = 0.1f;
        bool waitForForces = true;  // up to 4 ms
    };

    // PBF (Macklin and Mueller 2013) on GL compute. Fixed 60 Hz frames; coupling data comes back
    // through a fenced 3-slot ring without stalling.
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
        // Up to 2 fixed frames, then queues the coupling readback.
        void Update(float deltaSeconds);
        void Shutdown();

        // Public for the GPU tests.
        void BuildGrid();
        void Predict();
        void ComputeLambda();
        void ApplyDelta();
        void UpdateVelocities();
        void ApplyVelocityPasses();
        void ApplyMouseForce(float scale = 1.0f);
        int EmitFrame(float seconds);
        void Substep();
        void StepFrame();
        // Sync (tests) and fenced (panel).
        void ComputeNeighborStats();
        void RequestNeighborStats();
        void PollNeighborStats();
        void BuildNeighborList();
        bool SetWorkgroupSize(int size);
        int WorkgroupSize() const { return m_groupSize; }
        void UploadParams();
        void UploadParticles(const std::vector<Particle>& particles);
        void DownloadPredicted(std::vector<float>& out) const;
        void DownloadPositions(std::vector<float>& out) const;
        void DownloadVelocities(std::vector<float>& out) const;
        void DownloadVorticity(std::vector<float>& out) const;
        void DownloadNeighborCounts(std::vector<unsigned int>& out) const;
        void DownloadCellCounts(std::vector<unsigned int>& out) const;
        void DownloadCellStarts(std::vector<unsigned int>& out) const;

        // Up to kMaxBodies.
        void SetBodies(const std::vector<CouplingBody>& bodies);
        void ClearBodyImpulses();
        void DownloadBodyImpulses(std::vector<BodyForce>& out) const;
        // Reads the newest finished slot; false if none.
        bool PollBodyForces();
        // On reload and reset.
        void DiscardBodyFeedback();
        const std::vector<BodyForce>& BodyForces() const { return m_bodyForces; }
        int BodyCount() const { return m_bodyCount; }
        void SetBodyTime(float seconds) { m_bodyTime = seconds; }

        // Count and mean velocity per grid column.
        void ComputeColumns();
        void DownloadColumns(std::vector<float>& out) const;
        bool HasColumns() const { return m_columnsValid; }
        const std::vector<float>& Columns() const { return m_columnData; }
        int ColumnsX() const { return m_columnsX; }
        int ColumnsZ() const { return m_columnsZ; }
        float ParticleVolume() const
        {
            float spacing = 2.0f * m_binding.particleRadius;
            return spacing * spacing * spacing;
        }

        void BindParticleVao() const;
        void UnbindParticleVao() const;

        bool IsLoaded() const { return m_loaded; }
        const std::string& Status() const { return m_status; }
        const PbfBinding& Binding() const { return m_binding; }
        PbfBinding& BindingMutable() { return m_binding; }
        const ParamsGpu& Params() const { return m_params; }
        const GpuQueryTimer& Timer() const { return m_timer; }
        const NeighborStats& Neighbors() const { return m_neighborStats; }
        int ParticleCount() const { return m_count; }
        int MaxParticles() const { return m_maxParticles; }
        int EmitterCount() const { return static_cast<int>(m_emitters.size()); }
        int CellCount() const { return m_params.gridDims[3]; }
        float Radius() const { return m_binding.particleRadius; }
        float SmoothingRadius() const { return m_h; }
        float TimeStep() const { return m_params.gravity[3]; }
        float SimulatedSeconds() const { return m_simSeconds; }

        Settings settings;
        bool paused = false;
        bool collectNeighborStats = true;
        bool emittersEnabled = true;
        MouseForce mouse;
        CouplingSettings coupling;
        bool columnsEnabled = false;

       private:
        enum Attribute
        {
            kPosition = 0,
            kPredicted = 1,
            kVelocity = 2,
            kAttributeCount = 3
        };

        bool LoadShaders();
        struct Emitter
        {
            std::vector<Particle> layer;
            float interval = 0.0f;
            float timer = 0.0f;
        };

        void Seed();
        void UploadShapes();
        void BuildEmitters();
        void AppendParticles(const std::vector<Particle>& particles);
        void BindParticles(int set, unsigned int firstBinding) const;
        void BindCommon() const;
        void DispatchParticles(const Shader& shader) const;
        void DestroyVao();
        void SubmitReadback(float seconds, bool impulses, bool columns);
        void DropImpulseReadbacks();
        void ZeroBodyForces();
        void DispatchWithBodyTime(const Shader& shader) const;
        bool UseNeighborList() const;
        void DispatchNeighborStats();
        void ReadNeighborStats(const unsigned int* values, int count);
        void DropStatsReadback();

        struct ImpulseSlot
        {
            GpuBuffer buffer;
            GpuBuffer columns;
            bool hasImpulses = false;
            bool hasColumns = false;
            void* fence = nullptr;
            float seconds = 0.0f;
            int bodies = 0;
            long long serial = 0;
        };

        PbfBinding m_binding;
        ParamsGpu m_params;
        std::string m_shaderDir = "assets/shaders/";
        GpuBuffer m_paramsBuffer;
        GpuBuffer m_shapesBuffer;
        GpuBuffer m_bodiesBuffer;
        GpuBuffer m_bodyImpulses;
        ImpulseSlot m_impulseSlots[3];  // readback ring
        int m_nextSlot = 0;
        long long m_impulseSerial = 0;
        BodiesGpu m_bodiesGpu;
        int m_bodyCount = 0;
        float m_bodyTime = 0.0f;  // <= 0, relative to upload
        float m_bodyFrameOffset = 0.0f;
        std::vector<BodyForce> m_bodyForces;
        GpuBuffer m_particles[2][kAttributeCount];  // ping-pong
        int m_current = 0;
        GpuBuffer m_cellCounts;
        GpuBuffer m_cellStarts;
        GpuBuffer m_particleCells;
        GpuBuffer m_particleRanks;
        GpuBuffer m_stats;
        GpuBuffer m_neighborCounts;
        GpuBuffer m_statsReadback;
        GpuBuffer m_neighborList;
        void* m_statsFence = nullptr;
        int m_statsCount = 0;
        bool m_listValid = false;
        int m_groupSize = 256;
        GpuScan m_scan;
        Shader m_cellCountShader;
        Shader m_reorderShader;
        Shader m_neighborStatsShader;
        Shader m_predictShader;
        Shader m_lambdaShader;
        Shader m_lambdaListShader;
        Shader m_deltaShader;
        Shader m_deltaListShader;
        Shader m_neighborsShader;
        Shader m_applyShader;
        Shader m_updateShader;
        Shader m_vorticityShader;
        Shader m_vorticityListShader;
        Shader m_velocityShader;
        Shader m_velocityListShader;
        Shader m_mouseShader;
        Shader m_columnsShader;
        GpuBuffer m_columns;
        std::vector<float> m_columnData;
        bool m_columnsValid = false;
        int m_columnsX = 0;
        int m_columnsZ = 0;
        std::vector<Emitter> m_emitters;
        float m_accumulator = 0.0f;
        float m_mouseSeconds = 0.0f;
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
