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
        bool UpdateApron(AtlasChannel channel, const SparseTopology& topology, float boundary = 0.0f);

        bool Upload(AtlasChannel channel, const std::vector<unsigned char>& bytes);
        bool Download(AtlasChannel channel, std::vector<unsigned char>& bytes) const;

        int Capacity() const { return m_layers * kAtlasBricksPerLayer; }
        int3 TexelDims() const;
        size_t MemoryBytes() const;
        cudaSurfaceObject_t Surface(AtlasChannel channel) const;
        cudaTextureObject_t Texture(AtlasChannel channel) const;
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
}
