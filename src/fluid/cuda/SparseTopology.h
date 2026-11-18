#pragma once
#include <cstdint>
#include <vector>

#include "fluid/cuda/DeviceBuffer.h"
#include "fluid/cuda/GpuTimer.h"
#include "fluid/cuda/SparseTypes.h"

namespace Fluid
{
    class SparseTopology
    {
       public:
        bool FullRebuild(const float4* positions,
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
        int NodeCount() const { return m_nodeCount; }
        int LeafCount() const { return m_leafCount; }
        int InteriorCount() const { return m_interiorCount; }
        const Node* Nodes() const { return m_nodes.Data(); }
        const int* ChildLists() const { return m_childLists.Data(); }

       private:
        bool ComputeRoot(const float4* positions, int count, float influence, int3& rootCoord);
        bool Compact(int keyCount, int& unique);
        bool AddNodes(int unique);
        bool ReadCounters();

        VoxelSpace m_space;
        int m_rootLevel = 0;
        int m_nodeCount = 0;
        int m_leafCount = 0;
        int m_interiorCount = 0;

        DeviceBuffer<Node> m_nodes;
        DeviceBuffer<int> m_childLists;

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
