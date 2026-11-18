#include "FluidTests.h"
#include "TestFramework.h"
#include "fluid/cuda/DeviceBuffer.h"
#include "fluid/cuda/GpuTimer.h"
#include "fluid/cuda/ParticleKernels.h"

using Fluid::DeviceBuffer;

void RunGpuTimerTests()
{
    constexpr int kCount = 1 << 16;
    const Fluid::Box seed{make_float3(0.0f, 0.0f, 0.0f), make_float3(1.0f, 1.0f, 1.0f)};
    DeviceBuffer<float4> positions(kCount);
    DeviceBuffer<float4> velocities(kCount);

    Fluid::GpuTimer timer;
    timer.Begin("Seed");
    CHECK(Fluid::LaunchSeedParticles(positions.Data(), velocities.Data(), kCount, seed));
    timer.End();
    CHECK(cudaDeviceSynchronize() == cudaSuccess);

    timer.BeginFrame();
    CHECK(timer.Stages().size() == 1);
    CHECK(timer.Stages()[0].name == "Seed");
    CHECK(timer.Stages()[0].valid);
    CHECK(timer.Stages()[0].ms >= 0.0f);
    CHECK(timer.TotalMs() == timer.Stages()[0].ms);

    timer.Reset();
    timer.BeginFrame();
    CHECK(timer.Stages().empty());
}
