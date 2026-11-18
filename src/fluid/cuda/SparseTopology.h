#pragma once
#include <cstdint>
#include <vector>

#include "fluid/cuda/DeviceBuffer.h"
#include "fluid/cuda/GpuTimer.h"
#include "fluid/cuda/SparseTypes.h"

namespace Fluid
{
    // The sparse tree (paper section 5): node pool and child-list pool on the device. Leaves are
    // bricks covering every particle's influence box (one influence distance each way). FullRebuild
    // builds it from scratch (section 5.1); Update edits the previous tree (section 5.2) and falls
    // back to a full rebuild when the root changes.
    class SparseTopology
    {
       public:
        bool FullRebuild(const float4* positions,
                         int count,
                         const VoxelSpace& space,
                         float influence,
                         GpuTimer* timer = nullptr);
        bool Update(const float4* positions,
                    int count,
                    const VoxelSpace& space,
                    float influence,
                    GpuTimer* timer = nullptr);
        void Release();

        int CountUncovered(const float4* positions, int count, float influence);
        bool DownloadNodes(std::vector<Node>& nodes) const;
        bool DownloadChildLists(std::vector<int>& childLists) const;

        const VoxelSpace& Space() const { return m_space; }
        int RootLevel() const { return m_rootLevel; }
        int NodeSlots() const { return m_nodeEnd; }
        int NodeCount() const { return m_nodeEnd - m_freeNodeCount; }
        int LeafCount() const { return m_brickEnd - m_freeBrickCount; }
        int InteriorCount() const { return m_listEnd - m_freeListCount; }
        int ChildListSlots() const { return m_listEnd; }
        size_t MemoryBytes() const;
        int BrickCapacity() const { return m_brickEnd; }
        bool LastWasFull() const { return m_lastWasFull; }
        const Node* Nodes() const { return m_nodes.Data(); }
        const int* ChildLists() const { return m_childLists.Data(); }

       private:
        bool ComputeRoot(const float4* positions, int count, float influence, int& level, int3& rootCoord);
        bool ResetTree(int rootLevel, int3 rootCoord);
        bool AppendMissing(const float4* positions, int count, float influence, bool mark, int& appended);
        bool RemoveUnmarked();
        bool Compact(int keyCount, int& unique);
        bool AddNodes(int unique);
        bool ReadFreeCounts();
        bool WriteFreeCounts();
        void ReleaseScratch();

        VoxelSpace m_space;
        int m_rootLevel = 0;
        int3 m_rootCoord{0, 0, 0};
        bool m_lastWasFull = true;

        int m_nodeEnd = 0;
        int m_brickEnd = 0;
        int m_listEnd = 0;
        int m_freeNodeCount = 0;
        int m_freeBrickCount = 0;
        int m_freeListCount = 0;

        DeviceBuffer<Node> m_nodes;
        DeviceBuffer<int> m_childLists;
        DeviceBuffer<unsigned char> m_marks;
        DeviceBuffer<int> m_freeNodes;
        DeviceBuffer<int> m_freeBricks;
        DeviceBuffer<int> m_freeLists;
        DeviceBuffer<int> m_newIndices;

        DeviceBuffer<uint64_t> m_keys;
        DeviceBuffer<uint64_t> m_sortedKeys;
        DeviceBuffer<uint64_t> m_compact;
        DeviceBuffer<unsigned int> m_flags;
        DeviceBuffer<unsigned int> m_offsets;
        DeviceBuffer<unsigned char> m_temp;
        DeviceBuffer<float4> m_bounds;
        DeviceBuffer<int> m_counters;
    };
}
