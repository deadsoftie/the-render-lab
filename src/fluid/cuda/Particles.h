#pragma once
#include "fluid/cuda/DeviceBuffer.h"
#include "fluid/cuda/FluidTypes.h"

namespace Fluid
{
    float MaxParticleSpeed(const float4* velocities, int count, DeviceBuffer<unsigned int>& scratch);
    int SubstepCount(float maxSpeed, float h, float3 gravity, float frameSeconds);
}
