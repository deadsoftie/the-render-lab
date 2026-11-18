#pragma once
#include <cuda_runtime.h>

namespace Fluid
{
    struct Box
    {
        float3 lo;
        float3 hi;
    };

    bool LaunchSeedParticles(float4* positions,
                             float4* velocities,
                             int count,
                             const Box& seed,
                             cudaStream_t stream = 0);

    bool LaunchBallisticStep(float4* positions,
                             float4* velocities,
                             int count,
                             const Box& bounds,
                             float3 gravity,
                             float dt,
                             float restitution,
                             cudaStream_t stream = 0);
}
