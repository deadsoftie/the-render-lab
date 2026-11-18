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
        constexpr int kUncoveredCounter = 1;
        constexpr int kFreeNodeCounter = 2;
        constexpr int kFreeBrickCounter = 3;
        constexpr int kFreeListCounter = 4;
        constexpr int kLeafCounter = 5;
        constexpr int kCounterCount = 6;
        constexpr int kMaxCoord = 1 << 20;
        constexpr int kMinAppendCapacity = 4096;

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

        // Paper section 5.1 step 2: one key (level and node coordinate) per particle per level
        // below the root.
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

        __global__ void CountLeavesKernel(const uint64_t* keys, int n, int* counters)
        {
            int i = blockIdx.x * blockDim.x + threadIdx.x;
            if (i < n && KeyLevel(keys[i]) == 0)
                atomicAdd(counters + kLeafCounter, 1);
        }

        // The rank-th new slot: free-list entries first (section 5.2), then fresh slots past the end.
        __device__ int Allocate(int rank, const int* freeList, int freeCount, int end)
        {
            return rank < freeCount ? freeList[freeCount - 1 - rank] : end + (rank - freeCount);
        }

        // Section 5.1 step 4. Keys are sorted with the level in the top bits, so the first newLeaves keys
        // are leaves (brick ranks) and the rest are interior nodes (child list ranks).
        __global__ void InitNodesKernel(const uint64_t* keys,
                                        int n,
                                        int newLeaves,
                                        const int* freeNodes,
                                        int freeNodeCount,
                                        int nodeEnd,
                                        const int* freeBricks,
                                        int freeBrickCount,
                                        int brickEnd,
                                        const int* freeLists,
                                        int freeListCount,
                                        int listEnd,
                                        Node* nodes,
                                        int* newIndices)
        {
            int i = blockIdx.x * blockDim.x + threadIdx.x;
            if (i >= n)
                return;
            Node node;
            node.level = KeyLevel(keys[i]);
            node.coord = KeyCoord(keys[i]);
            if (node.level == 0)
                node.brick = Allocate(i, freeBricks, freeBrickCount, brickEnd);
            else
                node.childList = Allocate(i - newLeaves, freeLists, freeListCount, listEnd);
            int index = Allocate(i, freeNodes, freeNodeCount, nodeEnd);
            nodes[index] = node;
            newIndices[i] = index;
        }

        __global__ void ClearListsKernel(const int* newIndices,
                                         int newLeaves,
                                         int newInterior,
                                         const Node* nodes,
                                         int* childLists)
        {
            int t = blockIdx.x * blockDim.x + threadIdx.x;
            if (t >= newInterior * kChildCount)
                return;
            int node = newIndices[newLeaves + t / kChildCount];
            childLists[nodes[node].childList * kChildCount + t % kChildCount] = kNullNode;
        }

        // Section 5.1 step 5, launched top-down one level at a time so every parent is linked before its
        // children look it up.
        __global__ void LinkKernel(const int* newIndices, int n, Node* nodes, int* childLists, int rootLevel, int level)
        {
            int t = blockIdx.x * blockDim.x + threadIdx.x;
            if (t >= n)
                return;
            int i = newIndices[t];
            if (nodes[i].level != level)
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

        // Section 5.1.2 second pass, also the incremental pass (section 5.2): checks the leaves
        // under the 8 corners of each particle's influence box and appends keys for every missing
        // level. With marks set it also flags the nodes that are still hit, so the rest can be
        // removed.
        __global__ void CornerKernel(const float4* positions,
                                     int count,
                                     VoxelSpace space,
                                     float influence,
                                     const Node* nodes,
                                     const int* childLists,
                                     int rootLevel,
                                     unsigned char* marks,
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
                if (FindNode(nodes, childLists, rootLevel, voxel, 0, deepest, marks) != kNullNode)
                    continue;
                for (int level = deepest - 1; level >= 0; --level)
                {
                    int slot = atomicAdd(counters + kAppendCounter, 1);
                    if (slot < capacity)
                        keys[slot] = PackKey(level, NodeCoord(voxel, level));
                }
            }
        }

        // Frees unmarked nodes of one level (run bottom-up): unlinks them from their parent and
        // pushes the node, brick or child list onto the free lists. Node 0 is the root and stays.
        __global__ void RemoveKernel(Node* nodes,
                                     int* childLists,
                                     const unsigned char* marks,
                                     int nodeEnd,
                                     int level,
                                     int* freeNodes,
                                     int* freeBricks,
                                     int* freeLists,
                                     int* counters)
        {
            int i = 1 + blockIdx.x * blockDim.x + threadIdx.x;
            if (i >= nodeEnd || marks[i] != 0)
                return;
            Node node = nodes[i];
            if (node.level != level)
                return;

            if (node.parent != kNullNode)
                childLists[nodes[node.parent].childList * kChildCount + ChildSlot(node.coord)] = kNullNode;
            freeNodes[atomicAdd(counters + kFreeNodeCounter, 1)] = i;
            if (level == 0)
                freeBricks[atomicAdd(counters + kFreeBrickCounter, 1)] = node.brick;
            else
                freeLists[atomicAdd(counters + kFreeListCounter, 1)] = node.childList;
            nodes[i].level = kDeadLevel;
            nodes[i].parent = kNullNode;
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

        bool SameSpace(const VoxelSpace& a, const VoxelSpace& b)
        {
            return a.h == b.h && a.origin.x == b.origin.x && a.origin.y == b.origin.y && a.origin.z == b.origin.z;
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
        m_lastWasFull = true;

        if (timer)
            timer->Begin("Topology (full)");

        bool ok = [&]()
        {
            int level = 0;
            int3 rootCoord;
            if (!ComputeRoot(positions, count, influence, level, rootCoord) || !ResetTree(level, rootCoord))
                return false;

            const int keyCount = count * m_rootLevel;
            int unique = 0;
            if (!m_keys.Resize(keyCount))
                return false;
            LevelKeysKernel<<<Blocks(count), kBlockSize>>>(positions, count, m_space, m_rootLevel, m_keys.Data());
            if (!CUDA_CHECK(cudaGetLastError()) || !Compact(keyCount, unique) || !AddNodes(unique))
                return false;

            // The particle keys only cover the particles' own voxels; the corner pass adds the
            // neighbors their influence boxes reach.
            int appended = 0;
            return AppendMissing(positions, count, influence, false, appended) && Compact(appended, unique) &&
                   AddNodes(unique);
        }();

        if (timer)
            timer->End();
        if (!ok)
            m_nodeEnd = 0;
        return ok;
    }

    bool SparseTopology::Update(const float4* positions,
                                int count,
                                const VoxelSpace& space,
                                float influence,
                                GpuTimer* timer)
    {
        if (count <= 0)
            return false;

        // The full rebuild's per-particle key buffers are large; free them once it is done (section 9.3).
        auto fullRebuild = [&]()
        {
            bool ok = FullRebuild(positions, count, space, influence, timer);
            ReleaseScratch();
            return ok;
        };
        if (m_nodeEnd == 0 || !SameSpace(space, m_space))
            return fullRebuild();

        if (timer)
            timer->Begin("Topology (incremental)");

        int level = 0;
        int3 rootCoord;
        if (!ComputeRoot(positions, count, influence, level, rootCoord) || level != m_rootLevel ||
            rootCoord.x != m_rootCoord.x || rootCoord.y != m_rootCoord.y || rootCoord.z != m_rootCoord.z)
        {
            if (timer)
                timer->End();
            return fullRebuild();
        }
        m_lastWasFull = false;

        bool ok = [&]()
        {
            const unsigned char rootMark = 1;
            if (!m_marks.Resize(m_nodeEnd) || !m_marks.Clear() ||
                !CUDA_CHECK(cudaMemcpy(m_marks.Data(), &rootMark, 1, cudaMemcpyHostToDevice)))
                return false;
            if (static_cast<int>(m_keys.Size()) < std::max(kMinAppendCapacity, count / 16) &&
                !m_keys.Resize(std::max(kMinAppendCapacity, count / 16)))
                return false;

            // Mark what is still hit and collect what is missing, free the rest bottom-up, then add
            // the missing nodes into the freed slots first.
            int appended = 0;
            int unique = 0;
            return AppendMissing(positions, count, influence, true, appended) && RemoveUnmarked() &&
                   Compact(appended, unique) && AddNodes(unique);
        }();

        if (timer)
            timer->End();
        if (!ok)
            return fullRebuild();
        return true;
    }

    void SparseTopology::Release()
    {
        ReleaseScratch();
        m_nodes.Free();
        m_childLists.Free();
        m_marks.Free();
        m_freeNodes.Free();
        m_freeBricks.Free();
        m_freeLists.Free();
        m_newIndices.Free();
        m_bounds.Free();
        m_counters.Free();
        m_rootLevel = 0;
        m_nodeEnd = 0;
        m_brickEnd = 0;
        m_listEnd = 0;
        m_freeNodeCount = 0;
        m_freeBrickCount = 0;
        m_freeListCount = 0;
    }

    size_t SparseTopology::MemoryBytes() const
    {
        return m_nodes.Bytes() + m_childLists.Bytes() + m_marks.Bytes() + m_freeNodes.Bytes() +
               m_freeBricks.Bytes() + m_freeLists.Bytes() + m_newIndices.Bytes() + m_keys.Bytes() +
               m_sortedKeys.Bytes() + m_compact.Bytes() + m_flags.Bytes() + m_offsets.Bytes() + m_temp.Bytes() +
               m_bounds.Bytes() + m_counters.Bytes();
    }

    void SparseTopology::ReleaseScratch()
    {
        m_keys.Free();
        m_sortedKeys.Free();
        m_compact.Free();
        m_flags.Free();
        m_offsets.Free();
        m_temp.Free();
    }

    int SparseTopology::CountUncovered(const float4* positions, int count, float influence)
    {
        if (m_nodeEnd == 0 || count <= 0)
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

    bool SparseTopology::ComputeRoot(const float4* positions, int count, float influence, int& level, int3& rootCoord)
    {
        size_t bytes = 0;
        const float4 lowest = make_float4(FLT_MAX, FLT_MAX, FLT_MAX, 0.0f);
        const float4 highest = make_float4(-FLT_MAX, -FLT_MAX, -FLT_MAX, 0.0f);
        if (!m_bounds.Resize(2) ||
            !CUDA_CHECK(cub::DeviceReduce::Reduce(
                nullptr, bytes, positions, m_bounds.Data(), count, Float4Min{}, lowest)) ||
            !m_temp.Resize(std::max(bytes, m_temp.Size())) ||
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

        for (int candidate = 1; candidate < kMaxLevels; ++candidate)
        {
            const int3 a = NodeCoord(lo, candidate);
            const int3 b = NodeCoord(hi, candidate);
            if (a.x == b.x && a.y == b.y && a.z == b.z)
            {
                level = candidate;
                rootCoord = a;
                return true;
            }
        }
        return false;
    }

    bool SparseTopology::ResetTree(int rootLevel, int3 rootCoord)
    {
        m_rootLevel = rootLevel;
        m_rootCoord = rootCoord;
        m_nodeEnd = 1;
        m_brickEnd = 0;
        m_listEnd = 1;
        m_freeNodeCount = 0;
        m_freeBrickCount = 0;
        m_freeListCount = 0;

        Node root;
        root.level = rootLevel;
        root.coord = rootCoord;
        root.childList = 0;
        return m_nodes.Resize(1) && m_childLists.Resize(kChildCount) && m_counters.Resize(kCounterCount) &&
               m_counters.Clear() &&
               CUDA_CHECK(cudaMemcpy(m_nodes.Data(), &root, sizeof(Node), cudaMemcpyHostToDevice)) &&
               CUDA_CHECK(cudaMemset(m_childLists.Data(), 0xff, m_childLists.Bytes()));
    }

    bool SparseTopology::AppendMissing(const float4* positions, int count, float influence, bool mark, int& appended)
    {
        appended = 0;
        for (int pass = 0; pass < 2; ++pass)
        {
            const int capacity = static_cast<int>(m_keys.Size());
            if (!CUDA_CHECK(cudaMemset(m_counters.Data() + kAppendCounter, 0, sizeof(int))))
                return false;
            CornerKernel<<<Blocks(count), kBlockSize>>>(positions,
                                                        count,
                                                        m_space,
                                                        influence,
                                                        m_nodes.Data(),
                                                        m_childLists.Data(),
                                                        m_rootLevel,
                                                        mark ? m_marks.Data() : nullptr,
                                                        m_keys.Data(),
                                                        capacity,
                                                        m_counters.Data());
            if (!CUDA_CHECK(cudaGetLastError()) ||
                !CUDA_CHECK(cudaMemcpy(
                    &appended, m_counters.Data() + kAppendCounter, sizeof(int), cudaMemcpyDeviceToHost)))
                return false;
            if (appended <= capacity)
                return true;
            if (!m_keys.Resize(appended))
                return false;
        }
        return false;
    }

    bool SparseTopology::RemoveUnmarked()
    {
        if (m_nodeEnd <= 1)
            return true;
        for (int level = 0; level < m_rootLevel; ++level)
            RemoveKernel<<<Blocks(m_nodeEnd - 1), kBlockSize>>>(m_nodes.Data(),
                                                                m_childLists.Data(),
                                                                m_marks.Data(),
                                                                m_nodeEnd,
                                                                level,
                                                                m_freeNodes.Data(),
                                                                m_freeBricks.Data(),
                                                                m_freeLists.Data(),
                                                                m_counters.Data());
        return CUDA_CHECK(cudaGetLastError()) && ReadFreeCounts();
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
            !m_temp.Resize(std::max({sortBytes, scanBytes, m_temp.Size()})) ||
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

        int newLeaves = 0;
        if (!CUDA_CHECK(cudaMemset(m_counters.Data() + kLeafCounter, 0, sizeof(int))))
            return false;
        CountLeavesKernel<<<Blocks(unique), kBlockSize>>>(m_compact.Data(), unique, m_counters.Data());
        if (!CUDA_CHECK(cudaGetLastError()) ||
            !CUDA_CHECK(cudaMemcpy(&newLeaves, m_counters.Data() + kLeafCounter, sizeof(int), cudaMemcpyDeviceToHost)))
            return false;
        const int newInterior = unique - newLeaves;

        const int nodeEnd = m_nodeEnd + std::max(0, unique - m_freeNodeCount);
        const int brickEnd = m_brickEnd + std::max(0, newLeaves - m_freeBrickCount);
        const int listEnd = m_listEnd + std::max(0, newInterior - m_freeListCount);
        if (!m_nodes.Grow(nodeEnd) || !m_childLists.Grow(static_cast<size_t>(listEnd) * kChildCount) ||
            !m_freeNodes.Grow(nodeEnd) || !m_freeBricks.Grow(std::max(brickEnd, 1)) ||
            !m_freeLists.Grow(listEnd) || !m_newIndices.Resize(unique))
            return false;

        InitNodesKernel<<<Blocks(unique), kBlockSize>>>(m_compact.Data(),
                                                        unique,
                                                        newLeaves,
                                                        m_freeNodes.Data(),
                                                        m_freeNodeCount,
                                                        m_nodeEnd,
                                                        m_freeBricks.Data(),
                                                        m_freeBrickCount,
                                                        m_brickEnd,
                                                        m_freeLists.Data(),
                                                        m_freeListCount,
                                                        m_listEnd,
                                                        m_nodes.Data(),
                                                        m_newIndices.Data());
        if (newInterior > 0)
            ClearListsKernel<<<Blocks(newInterior * kChildCount), kBlockSize>>>(
                m_newIndices.Data(), newLeaves, newInterior, m_nodes.Data(), m_childLists.Data());
        for (int level = m_rootLevel - 1; level >= 0; --level)
            LinkKernel<<<Blocks(unique), kBlockSize>>>(
                m_newIndices.Data(), unique, m_nodes.Data(), m_childLists.Data(), m_rootLevel, level);

        m_nodeEnd = nodeEnd;
        m_brickEnd = brickEnd;
        m_listEnd = listEnd;
        m_freeNodeCount = std::max(0, m_freeNodeCount - unique);
        m_freeBrickCount = std::max(0, m_freeBrickCount - newLeaves);
        m_freeListCount = std::max(0, m_freeListCount - newInterior);
        return CUDA_CHECK(cudaGetLastError()) && WriteFreeCounts();
    }

    bool SparseTopology::ReadFreeCounts()
    {
        int counts[3] = {};
        if (!CUDA_CHECK(cudaMemcpy(
                counts, m_counters.Data() + kFreeNodeCounter, sizeof(counts), cudaMemcpyDeviceToHost)))
            return false;
        m_freeNodeCount = counts[0];
        m_freeBrickCount = counts[1];
        m_freeListCount = counts[2];
        return true;
    }

    bool SparseTopology::WriteFreeCounts()
    {
        const int counts[3] = {m_freeNodeCount, m_freeBrickCount, m_freeListCount};
        return CUDA_CHECK(
            cudaMemcpy(m_counters.Data() + kFreeNodeCounter, counts, sizeof(counts), cudaMemcpyHostToDevice));
    }
}
