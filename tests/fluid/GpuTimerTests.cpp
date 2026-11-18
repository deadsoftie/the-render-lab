#include "FluidTests.h"
#include "TestFramework.h"
#include "fluid/cuda/DeviceBuffer.h"
#include "fluid/cuda/GpuTimer.h"

void RunGpuTimerTests()
{
    Fluid::DeviceBuffer<float4> buffer(1 << 20);

    Fluid::GpuTimer timer;
    timer.Begin("Clear");
    CHECK(buffer.Clear());
    timer.End();
    CHECK(cudaDeviceSynchronize() == cudaSuccess);

    timer.BeginFrame();
    CHECK(timer.Stages().size() == 1);
    CHECK(timer.Stages()[0].name == "Clear");
    CHECK(timer.Stages()[0].valid);
    CHECK(timer.Stages()[0].ms >= 0.0f);
    CHECK(timer.TotalMs() == timer.Stages()[0].ms);

    timer.Reset();
    timer.BeginFrame();
    CHECK(timer.Stages().empty());
}
