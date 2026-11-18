#pragma once
#include "fluid/cuda/DeviceBuffer.h"
#include "fluid/cuda/FluidTypes.h"

namespace Fluid
{
    // Matrix-free CG for the pressure system on the dense grid (right-hand side: minus h squared
    // times the divergence), the baseline for SparseCg. Dot products are fused into the SpMV and
    // the x and r updates and accumulated as doubles; alpha and beta never leave the device, and
    // the residual is read back every 10 iterations.
    class DenseCg
    {
       public:
        CgResult Solve(const GridInfo& grid,
                       const unsigned char* markers,
                       const float* b,
                       float* x,
                       float tolerance,
                       int maxIterations,
                       cudaStream_t stream = 0);
        void Release();
        size_t MemoryBytes() const { return m_r.Bytes() + m_d.Bytes() + m_q.Bytes() + m_scalars.Bytes(); }

       private:
        bool Reserve(int cells);
        bool ReadScalar(int index, double& value, cudaStream_t stream) const;

        DeviceBuffer<float> m_r;
        DeviceBuffer<float> m_d;
        DeviceBuffer<float> m_q;
        DeviceBuffer<double> m_scalars;
    };
}
