#include "fluid/cuda/BrickAtlas.h"

#include <algorithm>

#include "fluid/cuda/CudaCheck.h"
#include "fluid/cuda/SparseGrid.cuh"

namespace Fluid
{
    namespace
    {
        constexpr int kBlockSize = 256;

        int Blocks(int n)
        {
            return (n + kBlockSize - 1) / kBlockSize;
        }

        struct ChannelSurfaces
        {
            cudaSurfaceObject_t surfaces[kAtlasChannelCount];
        };

        cudaChannelFormatDesc ChannelDesc(AtlasChannel channel)
        {
            switch (channel)
            {
                case AtlasChannel::Velocity:
                case AtlasChannel::VelocityOld:
                    return cudaCreateChannelDesc<float4>();
                case AtlasChannel::Marker:
                    return cudaCreateChannelDesc<unsigned char>();
                default:
                    return cudaCreateChannelDesc<float>();
            }
        }

        template <typename T>
        __device__ T SurfaceRead(cudaSurfaceObject_t surface, int3 texel)
        {
            return surf3Dread<T>(surface, texel.x * static_cast<int>(sizeof(T)), texel.y, texel.z);
        }

        template <typename T>
        __device__ void SurfaceWrite(cudaSurfaceObject_t surface, int3 texel, T value)
        {
            surf3Dwrite(value, surface, texel.x * static_cast<int>(sizeof(T)), texel.y, texel.z);
        }

        __device__ int3 ThreadLocal()
        {
            return make_int3(static_cast<int>(threadIdx.x) - kApron,
                             static_cast<int>(threadIdx.y) - kApron,
                             static_cast<int>(threadIdx.z) - kApron);
        }

        __global__ void BrickTableKernel(const Node* nodes, int slots, int* table)
        {
            int i = blockIdx.x * blockDim.x + threadIdx.x;
            if (i >= slots || nodes[i].level != 0)
                return;
            table[nodes[i].brick] = i;
        }

        __global__ void ClearKernel(ChannelSurfaces channels, const int* table)
        {
            int brick = blockIdx.x;
            if (table[brick] == kNullNode)
                return;

            int3 texel = AtlasTexel(brick, ThreadLocal());
            for (int c = 0; c < kAtlasChannelCount; ++c)
            {
                cudaSurfaceObject_t surface = channels.surfaces[c];
                switch (static_cast<AtlasChannel>(c))
                {
                    case AtlasChannel::Velocity:
                    case AtlasChannel::VelocityOld:
                        SurfaceWrite(surface, texel, make_float4(0.0f, 0.0f, 0.0f, 0.0f));
                        break;
                    case AtlasChannel::Marker:
                        SurfaceWrite(surface, texel, static_cast<unsigned char>(0));
                        break;
                    default:
                        SurfaceWrite(surface, texel, 0.0f);
                        break;
                }
            }
        }

        template <typename T>
        __global__ void ApronKernel(cudaSurfaceObject_t surface,
                                    const int* table,
                                    const Node* nodes,
                                    const int* childLists,
                                    int rootLevel)
        {
            int brick = blockIdx.x;
            int node = table[brick];
            if (node == kNullNode)
                return;

            int3 local = ThreadLocal();
            if (local.x >= 0 && local.x < kBrickDim && local.y >= 0 && local.y < kBrickDim && local.z >= 0 &&
                local.z < kBrickDim)
                return;

            int3 voxel = BrickVoxel(nodes[node].coord, local);
            int deepest = 0;
            int neighbor = FindNode(nodes, childLists, rootLevel, voxel, 0, deepest);
            T value{};
            if (neighbor != kNullNode)
                value = SurfaceRead<T>(surface, AtlasTexel(nodes[neighbor].brick, LocalVoxel(voxel)));
            SurfaceWrite(surface, AtlasTexel(brick, local), value);
        }
    }

    BrickAtlas::~BrickAtlas()
    {
        Release();
    }

    size_t BrickAtlas::ElementSize(AtlasChannel channel)
    {
        switch (channel)
        {
            case AtlasChannel::Velocity:
            case AtlasChannel::VelocityOld:
                return sizeof(float4);
            case AtlasChannel::Marker:
                return sizeof(unsigned char);
            default:
                return sizeof(float);
        }
    }

    int3 BrickAtlas::TexelDims() const
    {
        return make_int3(kAtlasBricksXY * kBrickTexels, kAtlasBricksXY * kBrickTexels, m_layers * kBrickTexels);
    }

    size_t BrickAtlas::MemoryBytes() const
    {
        const int3 dims = TexelDims();
        const size_t texels = static_cast<size_t>(dims.x) * dims.y * dims.z;
        size_t bytes = 0;
        for (int c = 0; c < kAtlasChannelCount; ++c)
            bytes += texels * ElementSize(static_cast<AtlasChannel>(c));
        return bytes;
    }

    cudaSurfaceObject_t BrickAtlas::Surface(AtlasChannel channel) const
    {
        return m_channels[static_cast<int>(channel)].surface;
    }

    cudaTextureObject_t BrickAtlas::Texture(AtlasChannel channel) const
    {
        return m_channels[static_cast<int>(channel)].texture;
    }

    bool BrickAtlas::Reserve(int brickCapacity)
    {
        const int needed = std::max(1, (brickCapacity + kAtlasBricksPerLayer - 1) / kAtlasBricksPerLayer);
        if (needed <= m_layers)
            return m_brickNodes.Resize(Capacity());
        if (needed > kAtlasMaxLayers)
            return false;

        const int layers = std::min(kAtlasMaxLayers, std::max(needed, 2 * m_layers));
        ChannelStorage fresh[kAtlasChannelCount];
        if (!Allocate(layers, fresh))
        {
            for (ChannelStorage& channel : fresh)
                Destroy(channel);
            return false;
        }

        if (m_layers > 0)
        {
            const int3 dims = TexelDims();
            for (int c = 0; c < kAtlasChannelCount; ++c)
            {
                cudaMemcpy3DParms copy{};
                copy.srcArray = m_channels[c].array;
                copy.dstArray = fresh[c].array;
                copy.extent = make_cudaExtent(dims.x, dims.y, dims.z);
                copy.kind = cudaMemcpyDeviceToDevice;
                if (!CUDA_CHECK(cudaMemcpy3D(&copy)))
                {
                    for (ChannelStorage& channel : fresh)
                        Destroy(channel);
                    return false;
                }
            }
        }

        for (int c = 0; c < kAtlasChannelCount; ++c)
        {
            Destroy(m_channels[c]);
            m_channels[c] = fresh[c];
        }
        m_layers = layers;
        return m_brickNodes.Resize(Capacity());
    }

    void BrickAtlas::Release()
    {
        for (ChannelStorage& channel : m_channels)
            Destroy(channel);
        m_layers = 0;
        m_brickNodes.Free();
    }

    bool BrickAtlas::BuildBrickTable(const SparseTopology& topology)
    {
        if (m_layers == 0 || topology.BrickCapacity() > Capacity())
            return false;
        if (!CUDA_CHECK(cudaMemset(m_brickNodes.Data(), 0xff, m_brickNodes.Bytes())))
            return false;
        const int slots = topology.NodeSlots();
        if (slots == 0)
            return true;
        BrickTableKernel<<<Blocks(slots), kBlockSize>>>(topology.Nodes(), slots, m_brickNodes.Data());
        return CUDA_CHECK(cudaGetLastError());
    }

    bool BrickAtlas::ClearActive()
    {
        if (m_layers == 0)
            return false;
        ChannelSurfaces channels;
        for (int c = 0; c < kAtlasChannelCount; ++c)
            channels.surfaces[c] = m_channels[c].surface;
        ClearKernel<<<Capacity(), dim3(kBrickTexels, kBrickTexels, kBrickTexels)>>>(channels, m_brickNodes.Data());
        return CUDA_CHECK(cudaGetLastError());
    }

    bool BrickAtlas::UpdateApron(AtlasChannel channel, const SparseTopology& topology)
    {
        if (m_layers == 0 || topology.NodeSlots() == 0)
            return false;
        const dim3 block(kBrickTexels, kBrickTexels, kBrickTexels);
        const cudaSurfaceObject_t surface = Surface(channel);
        switch (channel)
        {
            case AtlasChannel::Velocity:
            case AtlasChannel::VelocityOld:
                ApronKernel<float4><<<Capacity(), block>>>(
                    surface, m_brickNodes.Data(), topology.Nodes(), topology.ChildLists(), topology.RootLevel());
                break;
            case AtlasChannel::Marker:
                ApronKernel<unsigned char><<<Capacity(), block>>>(
                    surface, m_brickNodes.Data(), topology.Nodes(), topology.ChildLists(), topology.RootLevel());
                break;
            default:
                ApronKernel<float><<<Capacity(), block>>>(
                    surface, m_brickNodes.Data(), topology.Nodes(), topology.ChildLists(), topology.RootLevel());
                break;
        }
        return CUDA_CHECK(cudaGetLastError());
    }

    bool BrickAtlas::Upload(AtlasChannel channel, const std::vector<unsigned char>& bytes)
    {
        const int3 dims = TexelDims();
        const size_t element = ElementSize(channel);
        if (m_layers == 0 || bytes.size() != static_cast<size_t>(dims.x) * dims.y * dims.z * element)
            return false;

        cudaMemcpy3DParms copy{};
        copy.srcPtr = make_cudaPitchedPtr(const_cast<unsigned char*>(bytes.data()), dims.x * element, dims.x, dims.y);
        copy.dstArray = m_channels[static_cast<int>(channel)].array;
        copy.extent = make_cudaExtent(dims.x, dims.y, dims.z);
        copy.kind = cudaMemcpyHostToDevice;
        return CUDA_CHECK(cudaMemcpy3D(&copy));
    }

    bool BrickAtlas::Download(AtlasChannel channel, std::vector<unsigned char>& bytes) const
    {
        if (m_layers == 0)
            return false;
        const int3 dims = TexelDims();
        const size_t element = ElementSize(channel);
        bytes.resize(static_cast<size_t>(dims.x) * dims.y * dims.z * element);

        cudaMemcpy3DParms copy{};
        copy.srcArray = m_channels[static_cast<int>(channel)].array;
        copy.dstPtr = make_cudaPitchedPtr(bytes.data(), dims.x * element, dims.x, dims.y);
        copy.extent = make_cudaExtent(dims.x, dims.y, dims.z);
        copy.kind = cudaMemcpyDeviceToHost;
        return CUDA_CHECK(cudaMemcpy3D(&copy));
    }

    bool BrickAtlas::Allocate(int layers, ChannelStorage* channels)
    {
        const cudaExtent extent =
            make_cudaExtent(kAtlasBricksXY * kBrickTexels, kAtlasBricksXY * kBrickTexels, layers * kBrickTexels);
        for (int c = 0; c < kAtlasChannelCount; ++c)
        {
            const AtlasChannel channel = static_cast<AtlasChannel>(c);
            const cudaChannelFormatDesc desc = ChannelDesc(channel);
            ChannelStorage& storage = channels[c];
            if (!CUDA_CHECK(cudaMalloc3DArray(&storage.array, &desc, extent, cudaArraySurfaceLoadStore)))
            {
                storage.array = nullptr;
                return false;
            }

            cudaResourceDesc resource{};
            resource.resType = cudaResourceTypeArray;
            resource.res.array.array = storage.array;

            cudaTextureDesc texture{};
            texture.addressMode[0] = cudaAddressModeClamp;
            texture.addressMode[1] = cudaAddressModeClamp;
            texture.addressMode[2] = cudaAddressModeClamp;
            texture.filterMode = channel == AtlasChannel::Marker ? cudaFilterModePoint : cudaFilterModeLinear;
            texture.readMode = cudaReadModeElementType;
            texture.normalizedCoords = 0;

            if (!CUDA_CHECK(cudaCreateSurfaceObject(&storage.surface, &resource)) ||
                !CUDA_CHECK(cudaCreateTextureObject(&storage.texture, &resource, &texture, nullptr)))
                return false;
        }
        return true;
    }

    void BrickAtlas::Destroy(ChannelStorage& channel)
    {
        if (channel.texture)
            CUDA_CHECK(cudaDestroyTextureObject(channel.texture));
        if (channel.surface)
            CUDA_CHECK(cudaDestroySurfaceObject(channel.surface));
        if (channel.array)
            CUDA_CHECK(cudaFreeArray(channel.array));
        channel = ChannelStorage{};
    }
}
