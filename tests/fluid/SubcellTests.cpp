#include "FluidTests.h"
#include "TestFramework.h"
#include "fluid/cuda/DeviceBuffer.h"
#include "fluid/cuda/SparseGrid.cuh"
#include "fluid/cuda/SparseTopology.h"
#include "fluid/cuda/SubcellLists.h"

#include <algorithm>
#include <array>
#include <vector>

using namespace Fluid;

namespace
{
    using Entry = std::array<float, 8>;

    Entry MakeEntry(const float4& p, const float4& v)
    {
        return {p.x, p.y, p.z, p.w, v.x, v.y, v.z, v.w};
    }

    void MakeParticles(std::vector<float4>& positions, std::vector<float4>& velocities)
    {
        unsigned int seed = 99u;
        auto next = [&seed]()
        {
            seed = seed * 1664525u + 1013904223u;
            return static_cast<float>(seed >> 8) / 16777216.0f * 2.0f - 1.0f;
        };
        auto blob = [&](float3 center, float radius, int count)
        {
            int added = 0;
            while (added < count)
            {
                float3 p = make_float3(next(), next(), next());
                if (Dot(p, p) > 1.0f)
                    continue;
                p = center + p * radius;
                positions.push_back(make_float4(p.x, p.y, p.z, 1.0f));
                velocities.push_back(make_float4(next(), next(), next(), 0.0f));
                ++added;
            }
        };
        blob(make_float3(0.5f, 0.5f, 0.5f), 0.2f, 4000);
        blob(make_float3(1.0f, 0.15f, 0.4f), 0.05f, 400);
    }
}

void RunSubcellTests()
{
    VoxelSpace space;
    space.origin = make_float3(0.0f, 0.0f, 0.0f);
    space.h = 1.0f / 32.0f;
    const float influence = space.h;

    std::vector<float4> hostPositions;
    std::vector<float4> hostVelocities;
    MakeParticles(hostPositions, hostVelocities);
    const int count = static_cast<int>(hostPositions.size());

    DeviceBuffer<float4> positions;
    DeviceBuffer<float4> velocities;
    CHECK(positions.Upload(hostPositions));
    CHECK(velocities.Upload(hostVelocities));

    SparseTopology topology;
    CHECK(topology.FullRebuild(positions.Data(), count, space, influence));

    SubcellLists lists;
    CHECK(lists.Build(positions.Data(), velocities.Data(), count, topology, influence));
    CHECK(lists.SubcellCount() == topology.BrickCapacity() * kSubcellsPerBrick);
    CHECK(lists.EntryCount() >= count);

    std::vector<unsigned int> counts;
    std::vector<unsigned int> offsets;
    std::vector<float4> listPositions;
    std::vector<float4> listVelocities;
    CHECK(lists.Download(counts, offsets, listPositions, listVelocities));
    CHECK(static_cast<int>(listPositions.size()) == lists.EntryCount());

    bool scanned = true;
    unsigned int running = 0;
    for (size_t s = 0; s < counts.size(); ++s)
    {
        scanned &= offsets[s] == running;
        running += counts[s];
    }
    CHECK(scanned);
    CHECK(static_cast<int>(running) == lists.EntryCount());

    std::vector<Node> nodes;
    CHECK(topology.DownloadNodes(nodes));
    std::vector<std::vector<Entry>> expected(counts.size());
    std::vector<bool> active(counts.size(), false);
    for (const Node& node : nodes)
    {
        if (node.level != 0)
            continue;
        for (int s = 0; s < kSubcellsPerBrick; ++s)
            active[node.brick * kSubcellsPerBrick + s] = true;
    }

    for (int p = 0; p < count; ++p)
    {
        const float3 x = XYZ(hostPositions[p]);
        const float3 pad = make_float3(influence, influence, influence);
        const int3 lo = ToVoxel(space, x - pad);
        const int3 hi = ToVoxel(space, x + pad);
        for (const Node& node : nodes)
        {
            if (node.level != 0)
                continue;
            for (int s = 0; s < kSubcellsPerBrick; ++s)
            {
                const int3 sub = make_int3(s % kSubcellsPerAxis,
                                           (s / kSubcellsPerAxis) % kSubcellsPerAxis,
                                           s / (kSubcellsPerAxis * kSubcellsPerAxis));
                const int3 first = BrickVoxel(
                    node.coord, make_int3(sub.x * kSubcellDim, sub.y * kSubcellDim, sub.z * kSubcellDim));
                const bool overlaps = lo.x <= first.x + kSubcellDim - 1 && hi.x >= first.x &&
                                      lo.y <= first.y + kSubcellDim - 1 && hi.y >= first.y &&
                                      lo.z <= first.z + kSubcellDim - 1 && hi.z >= first.z;
                if (overlaps)
                    expected[node.brick * kSubcellsPerBrick + s].push_back(
                        MakeEntry(hostPositions[p], hostVelocities[p]));
            }
        }
    }

    bool matches = true;
    bool inactiveEmpty = true;
    for (size_t s = 0; s < counts.size(); ++s)
    {
        if (!active[s])
        {
            inactiveEmpty &= counts[s] == 0;
            continue;
        }
        std::vector<Entry> actual;
        for (unsigned int e = offsets[s]; e < offsets[s] + counts[s]; ++e)
            actual.push_back(MakeEntry(listPositions[e], listVelocities[e]));
        std::sort(actual.begin(), actual.end());
        std::sort(expected[s].begin(), expected[s].end());
        matches &= actual == expected[s];
    }
    CHECK(matches);
    CHECK(inactiveEmpty);
}
