#include "fluid/cuda/SparseTopology.h"

#include <algorithm>
#include <cfloat>

#include <cub/device/device_radix_sort.cuh>
#include <cub/device/device_reduce.cuh>
#include <cub/device/device_scan.cuh>

#include "fluid/cuda/CudaCheck.h"
#include "fluid/cuda/SparseGrid.cuh"

namespace Fluid
{
    namespace
    {
        constexpr int kBlockSize = 256;
        constexpr int kAppendCounter = 0;
        constexpr int kLeafCounter = 1;
        constexpr int kInteriorCounter = 2;
        constexpr int kUncoveredCounter = 3;
        constexpr int kCounterCount = 4;
        constexpr int kMaxCoord = 1 << 20;

        int Blocks(int n)
        {
            return (n + kBlockSize - 1) / kBlockSize;
        }

        struct Float4Min
        {
            __host__ __device__ float4 operator()(const float4& a, const float4& b) const
            {
                return make_float4(fminf(a.x, b.x), fminf(a.y, b.y), fminf(a.z, b.z), 0.0f);
            }
        };

        struct Float4Max
        {
            __host__ __device__ float4 operator()(const float4& a, const float4& b) const
            {
                return make_float4(fmaxf(a.x, b.x), fmaxf(a.y, b.y), fmaxf(a.z, b.z), 0.0f);
            }
        };

        __global__ void LevelKeysKernel(const float4* positions,
                                        int count,
                                        VoxelSpace space,
                                        int levels,
                                        uint64_t* keys)
        {
            int p = blockIdx.x * blockDim.x + threadIdx.x;
            if (p >= count)
                return;
            int3 voxel = ToVoxel(space, XYZ(positions[p]));
            for (int level = 0; level < levels; ++level)
                keys[p * levels + level] = PackKey(level, NodeCoord(voxel, level));
        }

        __global__ void FlagKernel(const uint64_t* keys, int n, unsigned int* flags)
        {
            int i = blockIdx.x * blockDim.x + threadIdx.x;
            if (i >= n)
                return;
            flags[i] = (i == 0 || keys[i] != keys[i - 1]) ? 1u : 0u;
        }

        __global__ void ScatterKernel(const uint64_t* keys,
                                      const unsigned int* flags,
                                      const unsigned int* offsets,
                                      int n,
                                      uint64_t* compact)
        {
            int i = blockIdx.x * blockDim.x + threadIdx.x;
            if (i >= n || flags[i] == 0u)
                return;
            compact[offsets[i]] = keys[i];
        }

        __global__ void InitNodesKernel(const uint64_t* keys, int n, int base, Node* nodes, int* counters)
        {
            int i = blockIdx.x * blockDim.x + threadIdx.x;
            if (i >= n)
                return;
            Node node;
            node.level = KeyLevel(keys[i]);
            node.coord = KeyCoord(keys[i]);
            if (node.level == 0)
                node.brick = atomicAdd(counters + kLeafCounter, 1);
            else
                node.childList = atomicAdd(counters + kInteriorCounter, 1);
            nodes[base + i] = node;
        }

        __global__ void LinkKernel(Node* nodes, int* childLists, int rootLevel, int begin, int end, int level)
        {
            int i = begin + blockIdx.x * blockDim.x + threadIdx.x;
            if (i >= end || nodes[i].level != level)
                return;
            int3 coord = nodes[i].coord;
            int deepest = 0;
            int parent = FindNode(nodes, childLists, rootLevel, ShiftUp(coord, LevelShift(level)), level + 1, deepest);
            if (parent == kNullNode)
                return;
            childLists[nodes[parent].childList * kChildCount + ChildSlot(coord)] = i;
            nodes[i].parent = parent;
        }

        __device__ bool SeenBefore(const int3* seen, int seenCount, int3 leaf)
        {
            for (int s = 0; s < seenCount; ++s)
                if (seen[s].x == leaf.x && seen[s].y == leaf.y && seen[s].z == leaf.z)
                    return true;
            return false;
        }

        __global__ void CornerKernel(const float4* positions,
                                     int count,
                                     VoxelSpace space,
                                     float influence,
                                     const Node* nodes,
                                     const int* childLists,
                                     int rootLevel,
                                     uint64_t* keys,
                                     int capacity,
                                     int* counters)
        {
            int p = blockIdx.x * blockDim.x + threadIdx.x;
            if (p >= count)
                return;

            float3 x = XYZ(positions[p]);
            int3 seen[8];
            int seenCount = 0;
            for (int corner = 0; corner < 8; ++corner)
            {
                int3 voxel = ToVoxel(space, Corner(x, influence, corner));
                int3 leaf = NodeCoord(voxel, 0);
                if (SeenBefore(seen, seenCount, leaf))
                    continue;
                seen[seenCount++] = leaf;

                int deepest = rootLevel;
                if (FindNode(nodes, childLists, rootLevel, voxel, 0, deepest) != kNullNode)
                    continue;
                for (int level = deepest - 1; level >= 0; --level)
                {
                    int slot = atomicAdd(counters + kAppendCounter, 1);
                    if (slot < capacity)
                        keys[slot] = PackKey(level, NodeCoord(voxel, level));
                }
            }
        }

        __global__ void CoverageKernel(const float4* positions,
                                       int count,
                                       VoxelSpace space,
                                       float influence,
                                       const Node* nodes,
                                       const int* childLists,
                                       int rootLevel,
                                       int* counters)
        {
            int p = blockIdx.x * blockDim.x + threadIdx.x;
            if (p >= count)
                return;

            float3 x = XYZ(positions[p]);
            for (int corner = 0; corner < 8; ++corner)
            {
                int deepest = rootLevel;
                int3 voxel = ToVoxel(space, Corner(x, influence, corner));
                if (FindNode(nodes, childLists, rootLevel, voxel, 0, deepest) == kNullNode)
                {
                    atomicAdd(counters + kUncoveredCounter, 1);
                    return;
                }
            }
        }
    }

    bool SparseTopology::FullRebuild(const float4* positions,
                                     int count,
                                     const VoxelSpace& space,
                                     float influence,
                                     GpuTimer* timer)
    {
        if (count <= 0)
            return false;
        m_space = space;
        m_nodeCount = 0;
        m_leafCount = 0;
        m_interiorCount = 0;

        if (timer)
            timer->Begin("Topology");

        bool ok = [&]()
        {
            int3 rootCoord;
            if (!m_counters.Resize(kCounterCount) || !ComputeRoot(positions, count, influence, rootCoord))
                return false;

            const int keyCount = count * m_rootLevel;
            int unique = 0;
            if (!m_keys.Resize(keyCount))
                return false;
            LevelKeysKernel<<<Blocks(count), kBlockSize>>>(positions, count, m_space, m_rootLevel, m_keys.Data());
            if (!CUDA_CHECK(cudaGetLastError()) || !Compact(keyCount, unique))
                return false;

            Node root;
            root.level = m_rootLevel;
            root.coord = rootCoord;
            root.childList = 0;
            const int initialCounters[kCounterCount] = {0, 0, 1, 0};
            if (!m_nodes.Resize(1) || !m_childLists.Resize(kChildCount) ||
                !CUDA_CHECK(cudaMemcpy(m_nodes.Data(), &root, sizeof(Node), cudaMemcpyHostToDevice)) ||
                !CUDA_CHECK(cudaMemset(m_childLists.Data(), 0xff, m_childLists.Bytes())) ||
                !CUDA_CHECK(cudaMemcpy(m_counters.Data(),
                                       initialCounters,
                                       sizeof(initialCounters),
                                       cudaMemcpyHostToDevice)))
                return false;
            m_nodeCount = 1;
            if (!AddNodes(unique))
                return false;

            int attempted = 0;
            for (int pass = 0; pass < 2; ++pass)
            {
                const int limit = static_cast<int>(m_keys.Size());
                if (!CUDA_CHECK(cudaMemset(m_counters.Data() + kAppendCounter, 0, sizeof(int))))
                    return false;
                CornerKernel<<<Blocks(count), kBlockSize>>>(positions,
                                                            count,
                                                            m_space,
                                                            influence,
                                                            m_nodes.Data(),
                                                            m_childLists.Data(),
                                                            m_rootLevel,
                                                            m_keys.Data(),
                                                            limit,
                                                            m_counters.Data());
                if (!CUDA_CHECK(cudaGetLastError()) ||
                    !CUDA_CHECK(cudaMemcpy(&attempted,
                                           m_counters.Data() + kAppendCounter,
                                           sizeof(int),
                                           cudaMemcpyDeviceToHost)))
                    return false;
                if (attempted <= limit)
                    break;
                if (!m_keys.Resize(attempted))
                    return false;
            }
            if (attempted > static_cast<int>(m_keys.Size()))
                return false;

            return Compact(attempted, unique) && AddNodes(unique) && ReadCounters();
        }();

        if (timer)
            timer->End();
        if (!ok)
            m_nodeCount = 0;
        return ok;
    }

    void SparseTopology::Release()
    {
        m_nodes.Free();
        m_childLists.Free();
        m_keys.Free();
        m_sortedKeys.Free();
        m_compact.Free();
        m_flags.Free();
        m_offsets.Free();
        m_temp.Free();
        m_bounds.Free();
        m_counters.Free();
        m_nodeCount = 0;
        m_leafCount = 0;
        m_interiorCount = 0;
        m_rootLevel = 0;
    }

    int SparseTopology::CountUncovered(const float4* positions, int count, float influence)
    {
        if (m_nodeCount == 0 || count <= 0)
            return count;
        if (!CUDA_CHECK(cudaMemset(m_counters.Data() + kUncoveredCounter, 0, sizeof(int))))
            return count;
        CoverageKernel<<<Blocks(count), kBlockSize>>>(positions,
                                                      count,
                                                      m_space,
                                                      influence,
                                                      m_nodes.Data(),
                                                      m_childLists.Data(),
                                                      m_rootLevel,
                                                      m_counters.Data());
        int uncovered = count;
        if (!CUDA_CHECK(cudaGetLastError()) ||
            !CUDA_CHECK(cudaMemcpy(&uncovered,
                                   m_counters.Data() + kUncoveredCounter,
                                   sizeof(int),
                                   cudaMemcpyDeviceToHost)))
            return count;
        return uncovered;
    }

    bool SparseTopology::DownloadNodes(std::vector<Node>& nodes) const
    {
        return m_nodes.Download(nodes);
    }

    bool SparseTopology::DownloadChildLists(std::vector<int>& childLists) const
    {
        return m_childLists.Download(childLists);
    }

    bool SparseTopology::ComputeRoot(const float4* positions, int count, float influence, int3& rootCoord)
    {
        size_t bytes = 0;
        const float4 lowest = make_float4(FLT_MAX, FLT_MAX, FLT_MAX, 0.0f);
        const float4 highest = make_float4(-FLT_MAX, -FLT_MAX, -FLT_MAX, 0.0f);
        if (!m_bounds.Resize(2) ||
            !CUDA_CHECK(cub::DeviceReduce::Reduce(
                nullptr, bytes, positions, m_bounds.Data(), count, Float4Min{}, lowest)) ||
            !m_temp.Resize(bytes) ||
            !CUDA_CHECK(cub::DeviceReduce::Reduce(
                m_temp.Data(), bytes, positions, m_bounds.Data(), count, Float4Min{}, lowest)) ||
            !CUDA_CHECK(cub::DeviceReduce::Reduce(
                m_temp.Data(), bytes, positions, m_bounds.Data() + 1, count, Float4Max{}, highest)))
            return false;

        std::vector<float4> bounds;
        if (!m_bounds.Download(bounds))
            return false;

        const float3 pad = make_float3(influence, influence, influence);
        const int3 lo = ToVoxel(m_space, XYZ(bounds[0]) - pad);
        const int3 hi = ToVoxel(m_space, XYZ(bounds[1]) + pad);
        if (lo.x < 0 || lo.y < 0 || lo.z < 0 || hi.x >= kMaxCoord || hi.y >= kMaxCoord || hi.z >= kMaxCoord)
            return false;

        for (int level = 1; level < kMaxLevels; ++level)
        {
            const int3 a = NodeCoord(lo, level);
            const int3 b = NodeCoord(hi, level);
            if (a.x == b.x && a.y == b.y && a.z == b.z)
            {
                m_rootLevel = level;
                rootCoord = a;
                return true;
            }
        }
        return false;
    }

    bool SparseTopology::Compact(int keyCount, int& unique)
    {
        unique = 0;
        if (keyCount == 0)
            return true;

        size_t sortBytes = 0;
        size_t scanBytes = 0;
        if (!m_sortedKeys.Resize(keyCount) || !m_flags.Resize(keyCount) || !m_offsets.Resize(keyCount) ||
            !CUDA_CHECK(cub::DeviceRadixSort::SortKeys(
                nullptr, sortBytes, m_keys.Data(), m_sortedKeys.Data(), keyCount)) ||
            !CUDA_CHECK(cub::DeviceScan::ExclusiveSum(
                nullptr, scanBytes, m_flags.Data(), m_offsets.Data(), keyCount)) ||
            !m_temp.Resize(std::max(sortBytes, scanBytes)) ||
            !CUDA_CHECK(cub::DeviceRadixSort::SortKeys(
                m_temp.Data(), sortBytes, m_keys.Data(), m_sortedKeys.Data(), keyCount)))
            return false;

        FlagKernel<<<Blocks(keyCount), kBlockSize>>>(m_sortedKeys.Data(), keyCount, m_flags.Data());
        if (!CUDA_CHECK(cudaGetLastError()) ||
            !CUDA_CHECK(cub::DeviceScan::ExclusiveSum(
                m_temp.Data(), scanBytes, m_flags.Data(), m_offsets.Data(), keyCount)))
            return false;

        unsigned int lastOffset = 0;
        unsigned int lastFlag = 0;
        if (!CUDA_CHECK(cudaMemcpy(&lastOffset,
                                   m_offsets.Data() + keyCount - 1,
                                   sizeof(unsigned int),
                                   cudaMemcpyDeviceToHost)) ||
            !CUDA_CHECK(cudaMemcpy(
                &lastFlag, m_flags.Data() + keyCount - 1, sizeof(unsigned int), cudaMemcpyDeviceToHost)))
            return false;

        unique = static_cast<int>(lastOffset + lastFlag);
        if (!m_compact.Resize(unique))
            return false;
        ScatterKernel<<<Blocks(keyCount), kBlockSize>>>(
            m_sortedKeys.Data(), m_flags.Data(), m_offsets.Data(), keyCount, m_compact.Data());
        return CUDA_CHECK(cudaGetLastError());
    }

    bool SparseTopology::AddNodes(int unique)
    {
        if (unique == 0)
            return true;
        if (!ReadCounters())
            return false;

        const size_t oldLists = m_childLists.Size();
        const size_t neededLists = static_cast<size_t>(m_interiorCount + unique) * kChildCount;
        if (neededLists > oldLists)
        {
            if (!m_childLists.Grow(neededLists) ||
                !CUDA_CHECK(cudaMemset(m_childLists.Data() + oldLists, 0xff, (neededLists - oldLists) * sizeof(int))))
                return false;
        }
        if (!m_nodes.Grow(static_cast<size_t>(m_nodeCount + unique)))
            return false;

        InitNodesKernel<<<Blocks(unique), kBlockSize>>>(
            m_compact.Data(), unique, m_nodeCount, m_nodes.Data(), m_counters.Data());
        for (int level = m_rootLevel - 1; level >= 0; --level)
            LinkKernel<<<Blocks(unique), kBlockSize>>>(
                m_nodes.Data(), m_childLists.Data(), m_rootLevel, m_nodeCount, m_nodeCount + unique, level);

        m_nodeCount += unique;
        return CUDA_CHECK(cudaGetLastError());
    }

    bool SparseTopology::ReadCounters()
    {
        std::vector<int> counters;
        if (!m_counters.Download(counters))
            return false;
        m_leafCount = counters[kLeafCounter];
        m_interiorCount = counters[kInteriorCounter];
        return true;
    }
}
