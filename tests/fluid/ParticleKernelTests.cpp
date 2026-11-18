#include "FluidTests.h"
#include "TestFramework.h"
#include "fluid/cuda/DeviceBuffer.h"
#include "fluid/cuda/ParticleKernels.h"

#include <algorithm>
#include <vector>

using Fluid::Box;
using Fluid::DeviceBuffer;

namespace
{
    bool Inside(const float4& p, const Box& box)
    {
        return p.x >= box.lo.x && p.x <= box.hi.x && p.y >= box.lo.y && p.y <= box.hi.y &&
               p.z >= box.lo.z && p.z <= box.hi.z;
    }

    float MeanHeight(const std::vector<float4>& positions)
    {
        double sum = 0.0;
        for (const float4& p : positions)
            sum += p.y;
        return static_cast<float>(sum / static_cast<double>(positions.size()));
    }
}

void RunParticleKernelTests()
{
    constexpr int kCount = 10000;
    const Box seed{make_float3(-0.5f, 0.2f, -0.25f), make_float3(0.1f, 0.8f, 0.25f)};
    const Box bounds{make_float3(-1.0f, 0.0f, -0.5f), make_float3(1.0f, 1.0f, 0.5f)};

    DeviceBuffer<float4> positions(kCount);
    DeviceBuffer<float4> velocities(kCount);
    CHECK(Fluid::LaunchSeedParticles(positions.Data(), velocities.Data(), kCount, seed));

    std::vector<float4> p;
    std::vector<float4> v;
    CHECK(positions.Download(p));
    CHECK(velocities.Download(v));
    CHECK(std::all_of(p.begin(), p.end(), [&](const float4& q) { return Inside(q, seed); }));
    CHECK(std::all_of(v.begin(), v.end(), [](const float4& q) {
        return q.x == 0.0f && q.y == 0.0f && q.z == 0.0f;
    }));

    const float seededHeight = MeanHeight(p);
    bool stepped = true;
    for (int i = 0; i < 240; ++i)
        stepped &= Fluid::LaunchBallisticStep(positions.Data(),
                                              velocities.Data(),
                                              kCount,
                                              bounds,
                                              make_float3(0.0f, -9.81f, 0.0f),
                                              1.0f / 120.0f,
                                              0.3f);
    CHECK(stepped);

    CHECK(positions.Download(p));
    CHECK(std::all_of(p.begin(), p.end(), [&](const float4& q) { return Inside(q, bounds); }));
    CHECK(MeanHeight(p) < seededHeight);
}
