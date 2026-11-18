#include "FluidTests.h"
#include "TestFramework.h"
#include "fluid/cuda/DeviceBuffer.h"
#include "fluid/cuda/SparseGrid.cuh"
#include "fluid/cuda/SparseTopology.h"

#include <algorithm>
#include <cfloat>
#include <set>
#include <vector>

using namespace Fluid;

namespace
{
    std::vector<float4> MakeParticles()
    {
        std::vector<float4> particles;
        unsigned int seed = 777u;
        auto next = [&seed]()
        {
            seed = seed * 1664525u + 1013904223u;
            return static_cast<float>(seed >> 8) / 16777216.0f;
        };

        while (particles.size() < 20000)
        {
            float3 p = make_float3(next() * 2.0f - 1.0f, next() * 2.0f - 1.0f, next() * 2.0f - 1.0f);
            if (Dot(p, p) > 1.0f)
                continue;
            p = make_float3(0.5f, 0.4f, 0.5f) + p * 0.3f;
            particles.push_back(make_float4(p.x, p.y, p.z, 1.0f));
        }
        for (int i = 0; i < 4000; ++i)
            particles.push_back(make_float4(next() * 1.5f, 0.01f + next() * 0.03f, next(), 1.0f));
        return particles;
    }

    std::vector<float4> Shifted(const std::vector<float4>& particles, float3 offset)
    {
        std::vector<float4> moved = particles;
        for (float4& p : moved)
        {
            p.x += offset.x;
            p.y += offset.y;
            p.z += offset.z;
        }
        return moved;
    }

    std::set<uint64_t> ExpectedKeys(const std::vector<float4>& particles,
                                    const VoxelSpace& space,
                                    float influence,
                                    int& rootLevel)
    {
        float3 lo = make_float3(FLT_MAX, FLT_MAX, FLT_MAX);
        float3 hi = make_float3(-FLT_MAX, -FLT_MAX, -FLT_MAX);
        for (const float4& p : particles)
        {
            lo = make_float3(std::min(lo.x, p.x), std::min(lo.y, p.y), std::min(lo.z, p.z));
            hi = make_float3(std::max(hi.x, p.x), std::max(hi.y, p.y), std::max(hi.z, p.z));
        }
        const float3 pad = make_float3(influence, influence, influence);
        const int3 vlo = ToVoxel(space, lo - pad);
        const int3 vhi = ToVoxel(space, hi + pad);

        rootLevel = -1;
        for (int level = 1; level < kMaxLevels && rootLevel < 0; ++level)
        {
            int3 a = NodeCoord(vlo, level);
            int3 b = NodeCoord(vhi, level);
            if (a.x == b.x && a.y == b.y && a.z == b.z)
                rootLevel = level;
        }

        std::set<uint64_t> keys;
        if (rootLevel < 0)
            return keys;
        keys.insert(PackKey(rootLevel, NodeCoord(vlo, rootLevel)));
        for (const float4& p : particles)
        {
            const float3 x = XYZ(p);
            std::vector<int3> voxels{ToVoxel(space, x)};
            for (int corner = 0; corner < 8; ++corner)
                voxels.push_back(ToVoxel(space, Corner(x, influence, corner)));
            for (const int3& voxel : voxels)
                for (int level = 0; level < rootLevel; ++level)
                    keys.insert(PackKey(level, NodeCoord(voxel, level)));
        }
        return keys;
    }

    void CheckTree(const SparseTopology& topology,
                   const std::vector<float4>& particles,
                   const VoxelSpace& space,
                   float influence)
    {
        int expectedRoot = -1;
        const std::set<uint64_t> expected = ExpectedKeys(particles, space, influence, expectedRoot);
        CHECK(expectedRoot >= 1);
        CHECK(topology.RootLevel() == expectedRoot);

        std::vector<Node> nodes;
        std::vector<int> childLists;
        CHECK(topology.DownloadNodes(nodes));
        CHECK(topology.DownloadChildLists(childLists));
        CHECK(static_cast<int>(nodes.size()) == topology.NodeSlots());
        if (nodes.empty())
            return;

        std::set<uint64_t> actual;
        std::set<int> bricks;
        std::set<int> lists;
        int live = 0;
        int leaves = 0;
        int interior = 0;
        bool linked = nodes[0].level == expectedRoot && nodes[0].parent == kNullNode;
        for (int i = 0; i < static_cast<int>(nodes.size()); ++i)
        {
            const Node& node = nodes[i];
            if (node.level == kDeadLevel)
                continue;
            ++live;
            actual.insert(PackKey(node.level, node.coord));
            if (node.level == 0)
            {
                ++leaves;
                linked &= node.childList == kNullNode && node.brick >= 0 &&
                          node.brick < topology.BrickCapacity() && bricks.insert(node.brick).second;
            }
            else
            {
                ++interior;
                linked &= node.brick == kNullNode && node.childList >= 0 && lists.insert(node.childList).second;
            }
            if (i == 0)
                continue;
            const int parent = node.parent;
            linked &= parent >= 0 && parent < static_cast<int>(nodes.size()) &&
                      nodes[parent].level == node.level + 1 &&
                      childLists[nodes[parent].childList * kChildCount + ChildSlot(node.coord)] == i;
        }

        for (int i = 0; i < static_cast<int>(nodes.size()); ++i)
        {
            if (nodes[i].level <= 0)
                continue;
            for (int slot = 0; slot < kChildCount; ++slot)
            {
                const int child = childLists[nodes[i].childList * kChildCount + slot];
                linked &= child == kNullNode ||
                          (child > 0 && child < static_cast<int>(nodes.size()) && nodes[child].parent == i);
            }
        }

        CHECK(linked);
        CHECK(static_cast<int>(actual.size()) == live);
        CHECK(actual == expected);
        CHECK(live == topology.NodeCount());
        CHECK(leaves == topology.LeafCount());
        CHECK(interior == topology.InteriorCount());
    }
}

void RunTopologyTests()
{
    VoxelSpace space;
    space.origin = make_float3(0.0f, 0.0f, 0.0f);
    space.h = 1.0f / 64.0f;
    const float influence = space.h;

    const std::vector<float4> particles = MakeParticles();
    const int count = static_cast<int>(particles.size());
    DeviceBuffer<float4> positions;
    CHECK(positions.Upload(particles));

    SparseTopology topology;
    CHECK(topology.FullRebuild(positions.Data(), count, space, influence));
    CHECK(topology.LastWasFull());
    CHECK(topology.CountUncovered(positions.Data(), count, influence) == 0);
    CheckTree(topology, particles, space, influence);

    std::vector<Node> fullNodes;
    CHECK(topology.DownloadNodes(fullNodes));
    std::vector<int> bricks;
    for (const Node& node : fullNodes)
        if (node.level == 0)
            bricks.push_back(node.brick);
    std::sort(bricks.begin(), bricks.end());
    bool contiguous = true;
    for (int i = 0; i < static_cast<int>(bricks.size()); ++i)
        contiguous &= bricks[i] == i;
    CHECK(contiguous);

    const std::vector<float4> moved = Shifted(particles, make_float3(0.11f, -0.004f, 0.07f));
    CHECK(positions.Upload(moved));
    CHECK(topology.Update(positions.Data(), count, space, influence));
    CHECK(!topology.LastWasFull());
    CHECK(topology.CountUncovered(positions.Data(), count, influence) == 0);
    CheckTree(topology, moved, space, influence);

    CHECK(positions.Upload(particles));
    CHECK(topology.Update(positions.Data(), count, space, influence));
    CHECK(!topology.LastWasFull());
    CheckTree(topology, particles, space, influence);

    const std::vector<float4> far = Shifted(particles, make_float3(7.0f, 0.0f, 0.0f));
    CHECK(positions.Upload(far));
    CHECK(topology.Update(positions.Data(), count, space, influence));
    CHECK(topology.LastWasFull());
    CheckTree(topology, far, space, influence);

    SparseTopology fresh;
    CHECK(fresh.Update(positions.Data(), count, space, influence));
    CHECK(fresh.LastWasFull());
    CHECK(fresh.NodeCount() == topology.NodeCount());

    const std::vector<float4> single{make_float4(0.3f, 0.3f, 0.3f, 1.0f)};
    CHECK(positions.Upload(single));
    CHECK(topology.FullRebuild(positions.Data(), 1, space, influence));
    CHECK(topology.CountUncovered(positions.Data(), 1, influence) == 0);
    CHECK(topology.RootLevel() >= 1);
    CHECK(topology.LeafCount() >= 1 && topology.LeafCount() <= 8);
}
