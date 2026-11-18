#pragma once
#include <cstddef>
#include <vector>

#include "fluid/cuda/DeviceBuffer.h"
#include "fluid/cuda/GpuTimer.h"
#include "fluid/cuda/SparseTopology.h"

namespace Fluid
{
    // Per-subcell particle lists (paper section 6). The 8 subcells of atlas brick n are stored
    // consecutively starting at index 8 times n, so the lists line up with the atlas. Entry order
    // inside a list is not deterministic.
    class SubcellLists
    {
       public:
        bool Build(const float4* positions,
                   const float4* velocities,
                   int count,
                   const SparseTopology& topology,
                   float influence,
                   GpuTimer* timer = nullptr);
        void Release();

        int SubcellCount() const { return m_subcellCount; }
        int EntryCount() const { return m_entryCount; }
        size_t MemoryBytes() const;

        const unsigned int* Counts() const { return m_counts.Data(); }
        const unsigned int* Offsets() const { return m_offsets.Data(); }
        const float4* Positions() const { return m_positions.Data(); }
        const float4* Velocities() const { return m_velocities.Data(); }

        bool Download(std::vector<unsigned int>& counts,
                      std::vector<unsigned int>& offsets,
                      std::vector<float4>& positions,
                      std::vector<float4>& velocities) const;

       private:
        int m_subcellCount = 0;
        int m_entryCount = 0;

        DeviceBuffer<unsigned int> m_counts;
        DeviceBuffer<unsigned int> m_offsets;
        DeviceBuffer<unsigned int> m_cursors;
        DeviceBuffer<float4> m_positions;
        DeviceBuffer<float4> m_velocities;
        DeviceBuffer<unsigned char> m_temp;
    };
}
