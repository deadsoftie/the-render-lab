#pragma once
#include "fluid/cuda/DeviceBuffer.h"
#include "fluid/cuda/FluidTypes.h"

namespace Fluid
{
    float MaxParticleSpeed(const float4* velocities, int count, DeviceBuffer<unsigned int>& scratch);
    int SubstepCount(float maxSpeed, float h, float3 gravity, float frameSeconds);

    // Positions stored as three 16-bit fractions of the domain extent per particle, for baking.
    // The step is the domain size divided by 65535 on each axis.
    bool QuantizePositions(const float4* positions,
                           int count,
                           const GridInfo& grid,
                           DeviceBuffer<unsigned short>& quantized);
    bool DequantizePositions(const DeviceBuffer<unsigned short>& quantized,
                             int count,
                             const GridInfo& grid,
                             DeviceBuffer<float4>& positions);
}
