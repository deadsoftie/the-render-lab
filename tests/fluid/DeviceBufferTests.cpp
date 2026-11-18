#include "FluidTests.h"
#include "TestFramework.h"
#include "fluid/cuda/DeviceBuffer.h"

#include <algorithm>
#include <numeric>
#include <vector>

using Fluid::DeviceBuffer;

void RunDeviceBufferTests()
{
    std::vector<int> host(1000);
    std::iota(host.begin(), host.end(), 1);

    DeviceBuffer<int> buffer;
    CHECK(buffer.Upload(host));
    CHECK(buffer.Size() == host.size());
    CHECK(buffer.Bytes() == host.size() * sizeof(int));

    std::vector<int> back;
    CHECK(buffer.Download(back));
    CHECK(back == host);

    int* before = buffer.Data();
    CHECK(buffer.Resize(10));
    CHECK(buffer.Data() == before);
    CHECK(buffer.Size() == 10);
    CHECK(buffer.Download(back));
    CHECK(back.size() == 10 && back.front() == 1 && back.back() == 10);

    CHECK(buffer.Clear());
    CHECK(buffer.Download(back));
    CHECK(std::all_of(back.begin(), back.end(), [](int v) { return v == 0; }));

    DeviceBuffer<int> moved = std::move(buffer);
    CHECK(moved.Data() == before);
    CHECK(moved.Size() == 10);
    CHECK(buffer.Data() == nullptr);
    CHECK(buffer.Size() == 0);

    DeviceBuffer<int> empty;
    CHECK(empty.Upload({}));
    CHECK(empty.Clear());
    CHECK(empty.Download(back));
    CHECK(back.empty());
}
