#include "fluid/cuda/SubcellLists.h"

#include <algorithm>

#include <cub/device/device_scan.cuh>

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

        // Paper section 6: rasterizes each particle's influence box into subcells (at most 2 per
        // axis). The count pass sizes the lists; after the scan, the fill pass writes copies of the
        // particle into each list, so the gather reads contiguous memory per subcell.
        template <bool Fill>
        __global__ void RasterizeKernel(const float4* positions,
                                        const float4* velocities,
                                        int count,
                                        VoxelSpace space,
                                        float influence,
                                        const Node* nodes,
                                        const int* childLists,
                                        int rootLevel,
                                        unsigned int* counts,
                                        const unsigned int* offsets,
                                        unsigned int* cursors,
                                        float4* listPositions,
                                        float4* listVelocities)
        {
            int p = blockIdx.x * blockDim.x + threadIdx.x;
            if (p >= count)
                return;

            float4 x = positions[p];
            float3 center = XYZ(x);
            float3 pad = make_float3(influence, influence, influence);
            int3 lo = ShiftDown(ToVoxel(space, center - pad), kLog2SubcellDim);
            int3 hi = ShiftDown(ToVoxel(space, center + pad), kLog2SubcellDim);

            for (int sz = lo.z; sz <= hi.z; ++sz)
                for (int sy = lo.y; sy <= hi.y; ++sy)
                    for (int sx = lo.x; sx <= hi.x; ++sx)
                    {
                        int3 subcell = make_int3(sx, sy, sz);
                        int deepest = 0;
                        int leaf = FindNode(
                            nodes, childLists, rootLevel, ShiftUp(subcell, kLog2SubcellDim), 0, deepest);
                        if (leaf == kNullNode)
                            continue;
                        int index = SubcellIndex(nodes[leaf].brick, subcell);
                        if (!Fill)
                        {
                            atomicAdd(counts + index, 1u);
                            continue;
                        }
                        unsigned int slot = offsets[index] + atomicAdd(cursors + index, 1u);
                        listPositions[slot] = x;
                        listVelocities[slot] = velocities[p];
                    }
        }
    }

    bool SubcellLists::Build(const float4* positions,
                             const float4* velocities,
                             int count,
                             const SparseTopology& topology,
                             float influence,
                             GpuTimer* timer)
    {
        if (count <= 0 || topology.NodeSlots() == 0)
            return false;

        if (timer)
            timer->Begin("Subcells");

        bool ok = [&]()
        {
            m_subcellCount = std::max(1, topology.BrickCapacity()) * kSubcellsPerBrick;
            m_entryCount = 0;
            if (!m_counts.Resize(m_subcellCount) || !m_offsets.Resize(m_subcellCount) ||
                !m_cursors.Resize(m_subcellCount) || !m_counts.Clear() || !m_cursors.Clear())
                return false;

            RasterizeKernel<false><<<Blocks(count), kBlockSize>>>(positions,
                                                                  velocities,
                                                                  count,
                                                                  topology.Space(),
                                                                  influence,
                                                                  topology.Nodes(),
                                                                  topology.ChildLists(),
                                                                  topology.RootLevel(),
                                                                  m_counts.Data(),
                                                                  nullptr,
                                                                  nullptr,
                                                                  nullptr,
                                                                  nullptr);

            size_t scanBytes = 0;
            if (!CUDA_CHECK(cudaGetLastError()) ||
                !CUDA_CHECK(cub::DeviceScan::ExclusiveSum(
                    nullptr, scanBytes, m_counts.Data(), m_offsets.Data(), m_subcellCount)) ||
                !m_temp.Resize(scanBytes) ||
                !CUDA_CHECK(cub::DeviceScan::ExclusiveSum(
                    m_temp.Data(), scanBytes, m_counts.Data(), m_offsets.Data(), m_subcellCount)))
                return false;

            unsigned int lastOffset = 0;
            unsigned int lastCount = 0;
            if (!CUDA_CHECK(cudaMemcpy(&lastOffset,
                                       m_offsets.Data() + m_subcellCount - 1,
                                       sizeof(unsigned int),
                                       cudaMemcpyDeviceToHost)) ||
                !CUDA_CHECK(cudaMemcpy(&lastCount,
                                       m_counts.Data() + m_subcellCount - 1,
                                       sizeof(unsigned int),
                                       cudaMemcpyDeviceToHost)))
                return false;

            m_entryCount = static_cast<int>(lastOffset + lastCount);
            if (!m_positions.Resize(m_entryCount) || !m_velocities.Resize(m_entryCount))
                return false;
            if (m_entryCount == 0)
                return true;

            RasterizeKernel<true><<<Blocks(count), kBlockSize>>>(positions,
                                                                 velocities,
                                                                 count,
                                                                 topology.Space(),
                                                                 influence,
                                                                 topology.Nodes(),
                                                                 topology.ChildLists(),
                                                                 topology.RootLevel(),
                                                                 nullptr,
                                                                 m_offsets.Data(),
                                                                 m_cursors.Data(),
                                                                 m_positions.Data(),
                                                                 m_velocities.Data());
            return CUDA_CHECK(cudaGetLastError());
        }();

        if (timer)
            timer->End();
        if (!ok)
            m_entryCount = 0;
        return ok;
    }

    void SubcellLists::Release()
    {
        m_counts.Free();
        m_offsets.Free();
        m_cursors.Free();
        m_positions.Free();
        m_velocities.Free();
        m_temp.Free();
        m_subcellCount = 0;
        m_entryCount = 0;
    }

    size_t SubcellLists::MemoryBytes() const
    {
        return m_counts.Bytes() + m_offsets.Bytes() + m_cursors.Bytes() + m_positions.Bytes() +
               m_velocities.Bytes();
    }

    bool SubcellLists::Download(std::vector<unsigned int>& counts,
                                std::vector<unsigned int>& offsets,
                                std::vector<float4>& positions,
                                std::vector<float4>& velocities) const
    {
        return m_counts.Download(counts) && m_offsets.Download(offsets) && m_positions.Download(positions) &&
               m_velocities.Download(velocities);
    }
}
