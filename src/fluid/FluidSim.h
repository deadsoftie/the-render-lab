#pragma once
#include <cstddef>
#include <string>
#include <vector>

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

    struct BackendSnapshot
    {
        bool valid = false;
        std::vector<GpuTimer::Stage> stages;
        StepStats stats;
    };

    struct MemoryReport
    {
        size_t particles = 0;
        size_t topology = 0;
        size_t atlas = 0;
        size_t subcells = 0;
        size_t dense = 0;
        long long cells = 0;
    };

    // Owns the particles and both solvers, runs the frame loop and publishes particles, the tree
    // and the level set to OpenGL through CUDA interop. All calls need the GL context current.
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

        void StartBake(int frames);
        void StopBake();
        void ClearBake();
        void SetPlaybackFrame(int frame);
        void SetBakeFile(const std::string& path);
        bool SaveBake();
        bool LoadBake();
        const std::string& BakeFile() const { return m_bakeFile; }
        bool HasSavedBake() const { return m_hasSavedBake; }
        bool IsBaking() const { return m_bakeTarget > 0; }
        int BakedFrames() const { return static_cast<int>(m_bakedFrames.size()); }
        int BakeTarget() const { return m_bakeTarget; }
        size_t BakeBytes() const { return m_bakeBytes; }
        int PlaybackFrame() const { return m_playbackFrame; }
        const std::string& BakeStatus() const { return m_bakeStatus; }
        int RenderCount() const { return m_renderCount; }

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
        const std::string& DenseStatus() const { return m_denseStatus; }
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
        const BackendSnapshot& Snapshot(Backend which) const
        {
            return m_snapshots[static_cast<int>(which)];
        }
        MemoryReport Memory() const;

        StepSettings settings;
        Backend backend = Backend::Sparse;
        bool incrementalTopology = true;
        bool surfaceView = true;
        bool overlayView = true;
        bool trackCoverage = false;
        bool playbackMode = false;
        bool loopPlayback = true;
        bool playing = true;
        bool stepRequested = false;

       private:
        struct BakedFrame
        {
            int count = 0;
            std::vector<unsigned short> positions;
        };

        bool StepFrame();
        bool BakeFrames();
        bool RecordFrame();
        void AdvancePlayback(float deltaSeconds);
        bool LoadBakedFrame(int index);
        void PublishFrame(const DeviceBuffer<float4>& positions,
                          const DeviceBuffer<float4>& velocities,
                          int count,
                          bool topologyFresh);
        bool CreateRenderBuffer(int count);
        void DestroyRenderBuffer();
        bool CreateTreeBuffers();
        void DestroyTreeBuffers();
        bool Seed();
        bool Emit(float dt);
        bool EnsureDense();
        bool DenseSubstep(float dt, CgResult& cg);
        bool SparseSubstep(float dt, CgResult& cg);
        bool RebuildSparseGrid(const float4* positions,
                               const float4* velocities,
                               int count,
                               bool gather);
        void PublishPositions(const DeviceBuffer<float4>& positions, int count);
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
        bool m_denseReady = false;
        std::string m_denseStatus;
        DenseFlip m_solver;
        SparseFlip m_sparse;
        StepStats m_lastStats;
        Backend m_timedBackend = Backend::Sparse;
        BackendSnapshot m_snapshots[2];

        int m_particleCount = 0;
        int m_maxParticles = 0;
        DeviceBuffer<float4> m_positions;
        DeviceBuffer<float4> m_velocities;
        DeviceBuffer<unsigned int> m_speedScratch;

        unsigned int m_renderBuffer = 0;
        unsigned int m_renderVAO = 0;
        InteropBuffer m_renderInterop;
        bool m_renderDirty = false;
        int m_renderCount = 0;

        std::vector<BakedFrame> m_bakedFrames;
        int m_bakeTarget = 0;
        size_t m_bakeBytes = 0;
        std::string m_bakeStatus;
        std::string m_bakeFile;
        bool m_hasSavedBake = false;
        DeviceBuffer<unsigned short> m_quantized;
        DeviceBuffer<float4> m_playbackPositions;
        DeviceBuffer<float4> m_playbackVelocities;
        int m_playbackFrame = 0;
        int m_shownFrame = -1;
        float m_playbackTime = 0.0f;
        bool m_lastPlaybackMode = false;

        bool m_topologyValid = false;
        bool m_levelSetValid = false;
        // The sparse tree and the gathered channels describe the live particles as they are now,
        // so the next sparse substep can skip its rebuild and gather.
        bool m_gridMatchesLive = false;
        bool m_treeReady = false;
        bool m_surfaceReady = false;
        bool m_lastSurfaceView = true;
        bool m_lastOverlayView = true;
        int m_uncovered = -1;
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
}  // namespace Fluid
