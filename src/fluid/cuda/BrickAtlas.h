#pragma once
#include <cstddef>
#include <vector>

#include <cuda_runtime.h>

#include "fluid/cuda/DeviceBuffer.h"
#include "fluid/cuda/SparseTopology.h"

namespace Fluid
{
    enum class AtlasChannel : int
    {
        Velocity = 0,
        VelocityOld,
        Marker,
        Divergence,
        Pressure,
        R,
        D,
        Q,
        LevelSet,
        Count
    };

    constexpr int kAtlasChannelCount = static_cast<int>(AtlasChannel::Count);

    // Per-channel brick storage (paper section 2.2). Every channel is a CUDA 3D array with a
    // surface object for kernel writes and a texture object for hardware-filtered reads. Brick n
    // belongs to the leaf whose brick field is n; BuildBrickTable maps it back after each topology
    // update, and kernels skip slots without a leaf.
    //
    // Velocity stores the three MAC face velocities on each voxel's min side, plus the gather
    // density in the fourth component.
    class BrickAtlas
    {
       public:
        BrickAtlas() = default;
        ~BrickAtlas();

        BrickAtlas(const BrickAtlas&) = delete;
        BrickAtlas& operator=(const BrickAtlas&) = delete;

        bool Reserve(int brickCapacity);
        void Release();

        bool BuildBrickTable(const SparseTopology& topology);
        bool ClearActive();
        // Refreshes every brick's apron from its neighbors (paper section 3). Apron texels with no
        // neighbor brick get the boundary value on float channels, zero velocity, or air for the
        // marker.
        bool UpdateApron(AtlasChannel channel,
                         const SparseTopology& topology,
                         float boundary = 0.0f,
                         cudaStream_t stream = 0);
        bool CopyChannel(AtlasChannel source, AtlasChannel destination);

        bool Upload(AtlasChannel channel, const std::vector<unsigned char>& bytes);
        bool Download(AtlasChannel channel, std::vector<unsigned char>& bytes) const;

        int Capacity() const { return m_layers * kAtlasBricksPerLayer; }
        int3 TexelDims() const;
        size_t MemoryBytes() const;
        cudaSurfaceObject_t Surface(AtlasChannel channel) const;
        cudaTextureObject_t Texture(AtlasChannel channel) const;
        cudaArray_t Array(AtlasChannel channel) const
        {
            return m_channels[static_cast<int>(channel)].array;
        }
        const int* BrickNodes() const { return m_brickNodes.Data(); }

        static size_t ElementSize(AtlasChannel channel);

       private:
        struct ChannelStorage
        {
            cudaArray_t array = nullptr;
            cudaSurfaceObject_t surface = 0;
            cudaTextureObject_t texture = 0;
        };

        static bool Allocate(int layers, ChannelStorage* channels);
        static void Destroy(ChannelStorage& channel);

        int m_layers = 0;
        ChannelStorage m_channels[kAtlasChannelCount];
        DeviceBuffer<int> m_brickNodes;
    };
}  // namespace Fluid
