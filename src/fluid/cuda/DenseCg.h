#pragma once
#include "fluid/cuda/DeviceBuffer.h"
#include "fluid/cuda/FluidTypes.h"

namespace Fluid
{
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

       private:
        bool Reserve(int cells);
        bool ReadScalar(int index, double& value, cudaStream_t stream) const;

        DeviceBuffer<float> m_r;
        DeviceBuffer<float> m_d;
        DeviceBuffer<float> m_q;
        DeviceBuffer<double> m_scalars;
    };
}
