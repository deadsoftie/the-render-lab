#pragma once
#include "fluid/cuda/DeviceBuffer.h"
#include "fluid/cuda/FluidTypes.h"

namespace Fluid
{
    constexpr int kParticlesPerCellAxis = 2;

    bool SeedParticles(const GridInfo& grid,
                       const ShapeList& seeds,
                       const ShapeList& obstacles,
                       DeviceBuffer<float4>& positions,
                       DeviceBuffer<float4>& velocities,
                       int& count);
}
