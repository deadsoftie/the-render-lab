#pragma once
#include <cuda_runtime.h>

#include "fluid/cuda/BrickAtlas.h"
#include "fluid/cuda/DeviceBuffer.h"
#include "fluid/cuda/FluidTypes.h"
#include "fluid/cuda/GpuTimer.h"
#include "fluid/cuda/SparseTopology.h"

namespace Fluid
{
    class SparseCg
    {
       public:
        SparseCg() = default;
        ~SparseCg();

        SparseCg(const SparseCg&) = delete;
        SparseCg& operator=(const SparseCg&) = delete;

        CgResult Solve(BrickAtlas& atlas, const SparseTopology& topology, float tolerance, int maxIterations);
        void Release();

        static constexpr int kIterationsPerGraph = 10;

       private:
        bool Prepare(int subcellCount);
        bool EnqueueIteration(BrickAtlas& atlas, const SparseTopology& topology, int subcellCount);
        bool EnqueueSum(int subcellCount, int slot);
        bool ReadScalar(int slot, double& value);
        bool BuildGraph(BrickAtlas& atlas, const SparseTopology& topology, int subcellCount);

        cudaStream_t m_stream = nullptr;
        cudaGraphExec_t m_graph = nullptr;
        DeviceBuffer<double> m_partials;
        DeviceBuffer<double> m_scalars;
        DeviceBuffer<unsigned char> m_temp;
        size_t m_tempBytes = 0;
    };

    CgResult SparseProject(BrickAtlas& atlas,
                           const SparseTopology& topology,
                           SparseCg& cg,
                           float3 gravity,
                           float dt,
                           float tolerance,
                           int maxIterations,
                           GpuTimer* timer = nullptr);
}
