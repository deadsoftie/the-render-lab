#include "FluidTests.h"
#include "TestFramework.h"
#include "fluid/cuda/BrickAtlas.h"
#include "fluid/cuda/DeviceBuffer.h"
#include "fluid/cuda/SparseGrid.cuh"
#include "fluid/cuda/SparseTopology.h"

#include <cstring>
#include <vector>

using namespace Fluid;

namespace
{
    float Field(float3 p)
    {
        return 1.0f + 2.0f * p.x - 3.0f * p.y + 0.5f * p.z;
    }

    float4 VectorField(float3 p)
    {
        const float f = Field(p);
        return make_float4(f, 2.0f * f, -f, 1.0f);
    }

    size_t TexelIndex(int3 dims, int3 t)
    {
        return static_cast<size_t>(t.x) +
               static_cast<size_t>(dims.x) * (static_cast<size_t>(t.y) + static_cast<size_t>(dims.y) * t.z);
    }

    template <typename T>
    T Get(const std::vector<unsigned char>& bytes, int3 dims, int3 t)
    {
        T value;
        std::memcpy(&value, bytes.data() + TexelIndex(dims, t) * sizeof(T), sizeof(T));
        return value;
    }

    template <typename T>
    void Set(std::vector<unsigned char>& bytes, int3 dims, int3 t, T value)
    {
        std::memcpy(bytes.data() + TexelIndex(dims, t) * sizeof(T), &value, sizeof(T));
    }

    bool SameFloat4(float4 a, float4 b)
    {
        return a.x == b.x && a.y == b.y && a.z == b.z && a.w == b.w;
    }

    bool Interior(int3 local)
    {
        return local.x >= 0 && local.x < kBrickDim && local.y >= 0 && local.y < kBrickDim && local.z >= 0 &&
               local.z < kBrickDim;
    }

    std::vector<float4> MakeParticles()
    {
        std::vector<float4> particles;
        unsigned int seed = 4242u;
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
                particles.push_back(make_float4(p.x, p.y, p.z, 1.0f));
                ++added;
            }
        };
        blob(make_float3(0.5f, 0.5f, 0.5f), 0.25f, 3000);
        blob(make_float3(1.1f, 0.2f, 0.3f), 0.05f, 300);
        return particles;
    }
}

void RunAtlasTests()
{
    VoxelSpace space;
    space.origin = make_float3(0.0f, 0.0f, 0.0f);
    space.h = 1.0f / 32.0f;

    const std::vector<float4> particles = MakeParticles();
    DeviceBuffer<float4> positions;
    CHECK(positions.Upload(particles));

    SparseTopology topology;
    CHECK(topology.FullRebuild(positions.Data(), static_cast<int>(particles.size()), space, space.h));

    BrickAtlas atlas;
    CHECK(atlas.Reserve(topology.BrickCapacity()));
    CHECK(atlas.Capacity() >= topology.BrickCapacity());
    CHECK(atlas.BuildBrickTable(topology));
    CHECK(atlas.ClearActive());

    std::vector<Node> nodes;
    std::vector<int> childLists;
    CHECK(topology.DownloadNodes(nodes));
    CHECK(topology.DownloadChildLists(childLists));

    const int3 dims = atlas.TexelDims();
    const size_t texels = static_cast<size_t>(dims.x) * dims.y * dims.z;
    std::vector<unsigned char> pressure(texels * sizeof(float), 0);
    std::vector<unsigned char> velocity(texels * sizeof(float4), 0);
    for (const Node& node : nodes)
    {
        if (node.level != 0)
            continue;
        for (int z = 0; z < kBrickDim; ++z)
            for (int y = 0; y < kBrickDim; ++y)
                for (int x = 0; x < kBrickDim; ++x)
                {
                    const int3 local = make_int3(x, y, z);
                    const float3 center = VoxelCenter(space, BrickVoxel(node.coord, local));
                    const int3 t = AtlasTexel(node.brick, local);
                    Set<float>(pressure, dims, t, Field(center));
                    Set<float4>(velocity, dims, t, VectorField(center));
                }
    }

    CHECK(atlas.Upload(AtlasChannel::Pressure, pressure));
    CHECK(atlas.Upload(AtlasChannel::Velocity, velocity));
    CHECK(atlas.UpdateApron(AtlasChannel::Pressure, topology));
    CHECK(atlas.UpdateApron(AtlasChannel::Velocity, topology));

    std::vector<unsigned char> pressureOut;
    std::vector<unsigned char> velocityOut;
    CHECK(atlas.Download(AtlasChannel::Pressure, pressureOut));
    CHECK(atlas.Download(AtlasChannel::Velocity, velocityOut));

    bool interiorKept = true;
    bool apronCorrect = true;
    int apronFilled = 0;
    int apronEmpty = 0;
    for (const Node& node : nodes)
    {
        if (node.level != 0)
            continue;
        for (int z = -kApron; z < kBrickDim + kApron; ++z)
            for (int y = -kApron; y < kBrickDim + kApron; ++y)
                for (int x = -kApron; x < kBrickDim + kApron; ++x)
                {
                    const int3 local = make_int3(x, y, z);
                    const int3 voxel = BrickVoxel(node.coord, local);
                    const int3 t = AtlasTexel(node.brick, local);
                    const float p = Get<float>(pressureOut, dims, t);
                    const float4 v = Get<float4>(velocityOut, dims, t);
                    const float3 center = VoxelCenter(space, voxel);
                    if (Interior(local))
                    {
                        interiorKept &= p == Field(center) && SameFloat4(v, VectorField(center));
                        continue;
                    }

                    int deepest = 0;
                    const int neighbor =
                        FindNode(nodes.data(), childLists.data(), topology.RootLevel(), voxel, 0, deepest);
                    if (neighbor == kNullNode)
                    {
                        ++apronEmpty;
                        apronCorrect &= p == 0.0f && SameFloat4(v, make_float4(0.0f, 0.0f, 0.0f, 0.0f));
                    }
                    else
                    {
                        ++apronFilled;
                        apronCorrect &= p == Field(center) && SameFloat4(v, VectorField(center));
                    }
                }
    }
    CHECK(interiorKept);
    CHECK(apronCorrect);
    CHECK(apronFilled > 0);
    CHECK(apronEmpty > 0);

    const int oldCapacity = atlas.Capacity();
    CHECK(atlas.Reserve(oldCapacity + 1));
    CHECK(atlas.Capacity() > oldCapacity);
    const int3 grownDims = atlas.TexelDims();
    CHECK(grownDims.x == dims.x && grownDims.y == dims.y && grownDims.z > dims.z);

    std::vector<unsigned char> grown;
    CHECK(atlas.Download(AtlasChannel::Pressure, grown));
    bool preserved = true;
    for (const Node& node : nodes)
    {
        if (node.level != 0)
            continue;
        for (int z = -kApron; z < kBrickDim + kApron; ++z)
            for (int y = -kApron; y < kBrickDim + kApron; ++y)
                for (int x = -kApron; x < kBrickDim + kApron; ++x)
                {
                    const int3 t = AtlasTexel(node.brick, make_int3(x, y, z));
                    preserved &= Get<float>(grown, grownDims, t) == Get<float>(pressureOut, dims, t);
                }
    }
    CHECK(preserved);

    CHECK(atlas.BuildBrickTable(topology));
    CHECK(atlas.ClearActive());
    std::vector<unsigned char> cleared;
    CHECK(atlas.Download(AtlasChannel::Pressure, cleared));
    bool zeroed = true;
    for (const Node& node : nodes)
    {
        if (node.level != 0)
            continue;
        for (int z = -kApron; z < kBrickDim + kApron; ++z)
            for (int y = -kApron; y < kBrickDim + kApron; ++y)
                for (int x = -kApron; x < kBrickDim + kApron; ++x)
                    zeroed &= Get<float>(cleared, grownDims, AtlasTexel(node.brick, make_int3(x, y, z))) == 0.0f;
    }
    CHECK(zeroed);
}
