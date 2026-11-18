#include "FluidTests.h"
#include "TestFramework.h"
#include "fluid/cuda/BrickAtlas.h"
#include "fluid/cuda/DenseFlip.h"
#include "fluid/cuda/DeviceBuffer.h"
#include "fluid/cuda/GridMath.cuh"
#include "fluid/cuda/Seeding.h"
#include "fluid/cuda/SparseGather.h"
#include "fluid/cuda/SparseGrid.cuh"
#include "fluid/cuda/SparseTopology.h"
#include "fluid/cuda/SubcellLists.h"

#include <cmath>
#include <cstring>
#include <unordered_map>
#include <vector>

using namespace Fluid;

namespace
{
    template <typename T>
    T AtlasValue(const std::vector<unsigned char>& bytes, int3 dims, int3 t)
    {
        T value;
        const size_t index =
            static_cast<size_t>(t.x) + static_cast<size_t>(dims.x) * (static_cast<size_t>(t.y) + static_cast<size_t>(dims.y) * t.z);
        std::memcpy(&value, bytes.data() + index * sizeof(T), sizeof(T));
        return value;
    }

    bool Near(float a, float b)
    {
        return std::fabs(a - b) <= 1e-4f * (1.0f + std::fabs(b));
    }

    struct LevelAccum
    {
        double weight = 0.0;
        double x = 0.0;
        double y = 0.0;
        double z = 0.0;
    };
}

void RunSparseGatherTests()
{
    GridInfo grid;
    grid.dims = make_int3(16, 16, 16);
    grid.origin = make_float3(0.0f, 0.0f, 0.0f);
    grid.h = 1.0f / 16.0f;

    ShapeList seeds;
    Shape& pool = seeds.items[seeds.count++];
    pool.type = ShapeType::Box;
    pool.lo = make_float3(0.0f, 0.0f, 0.0f);
    pool.hi = make_float3(1.0f, 0.25f, 1.0f);

    ShapeList obstacles;
    Shape& ball = obstacles.items[obstacles.count++];
    ball.type = ShapeType::Sphere;
    ball.center = make_float3(0.5f, 0.2f, 0.5f);
    ball.radius = 0.15f;

    DeviceBuffer<float4> positions;
    DeviceBuffer<float4> velocities;
    int count = 0;
    CHECK(SeedParticles(grid, seeds, obstacles, positions, velocities, count));
    CHECK(count > 0);

    std::vector<float4> hostVelocities(count);
    unsigned int seed = 31337u;
    for (float4& v : hostVelocities)
    {
        auto next = [&seed]()
        {
            seed = seed * 1664525u + 1013904223u;
            return static_cast<float>(seed >> 8) / 16777216.0f * 2.0f - 1.0f;
        };
        v = make_float4(next(), next(), next(), 0.0f);
    }
    CHECK(velocities.Upload(hostVelocities));

    DenseFlip dense;
    CHECK(dense.Init(grid, obstacles, count));
    CHECK(dense.Transfer(positions, velocities, count, 1.0f / 60.0f, make_float3(0.0f, 0.0f, 0.0f)));
    std::vector<float> du, dv, dw;
    std::vector<unsigned char> denseMarkers;
    CHECK(dense.UOld().Download(du) && dense.VOld().Download(dv) && dense.WOld().Download(dw));
    CHECK(dense.Markers().Download(denseMarkers));

    VoxelSpace space;
    space.origin = grid.origin;
    space.h = grid.h;
    const float influence = grid.h;
    const float levelSetRadius = kLevelSetRadiusScale * grid.h;

    SparseTopology topology;
    BrickAtlas atlas;
    SubcellLists lists;
    CHECK(topology.FullRebuild(positions.Data(), count, space, influence));
    CHECK(atlas.Reserve(topology.BrickCapacity()) && atlas.BuildBrickTable(topology) && atlas.ClearActive());
    CHECK(lists.Build(positions.Data(), velocities.Data(), count, topology, influence));
    CHECK(SparseParticleToGrid(lists, atlas, topology, obstacles, levelSetRadius));

    std::vector<unsigned char> velocityBytes, markerBytes, levelBytes;
    CHECK(atlas.Download(AtlasChannel::Velocity, velocityBytes));
    CHECK(atlas.Download(AtlasChannel::Marker, markerBytes));
    CHECK(atlas.Download(AtlasChannel::LevelSet, levelBytes));
    std::vector<Node> nodes;
    std::vector<int> childLists;
    std::vector<float4> hostPositions;
    CHECK(topology.DownloadNodes(nodes) && topology.DownloadChildLists(childLists));
    CHECK(positions.Download(hostPositions));

    const float kernelRadius = 2.0f * levelSetRadius;
    std::unordered_map<uint64_t, LevelAccum> level;
    for (const float4& p : hostPositions)
    {
        const int3 lo = ToVoxel(space, XYZ(p) - make_float3(kernelRadius, kernelRadius, kernelRadius));
        const int3 hi = ToVoxel(space, XYZ(p) + make_float3(kernelRadius, kernelRadius, kernelRadius));
        for (int z = lo.z; z <= hi.z; ++z)
            for (int y = lo.y; y <= hi.y; ++y)
                for (int x = lo.x; x <= hi.x; ++x)
                {
                    const float3 c = VoxelCenter(space, make_int3(x, y, z));
                    const double dx = p.x - c.x;
                    const double dy = p.y - c.y;
                    const double dz = p.z - c.z;
                    const double s2 = (dx * dx + dy * dy + dz * dz) / (kernelRadius * kernelRadius);
                    if (s2 >= 1.0)
                        continue;
                    const double w = (1.0 - s2) * (1.0 - s2) * (1.0 - s2);
                    LevelAccum& a = level[PackKey(0, make_int3(x, y, z))];
                    a.weight += w;
                    a.x += w * p.x;
                    a.y += w * p.y;
                    a.z += w * p.z;
                }
    }

    const int3 dims = atlas.TexelDims();
    const int3 fu = FaceDims(grid, 0);
    const int3 fv = FaceDims(grid, 1);
    const int3 fw = FaceDims(grid, 2);
    int compared = 0;
    int fluid = 0;
    bool velocityMatch = true;
    bool markerMatch = true;
    bool levelMatch = true;
    bool apronFar = true;
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
                    const int3 texel = AtlasTexel(node.brick, local);
                    const float phi = AtlasValue<float>(levelBytes, dims, texel);
                    const bool interior = x >= 0 && x < kBrickDim && y >= 0 && y < kBrickDim && z >= 0 &&
                                          z < kBrickDim;
                    if (!interior)
                    {
                        int deepest = 0;
                        if (FindNode(nodes.data(), childLists.data(), topology.RootLevel(), voxel, 0, deepest) ==
                            kNullNode)
                        {
                            ++apronEmpty;
                            apronFar &= phi == space.h;
                        }
                        continue;
                    }

                    float expectedPhi = space.h;
                    auto found = level.find(PackKey(0, voxel));
                    if (found != level.end() && found->second.weight < 1e-6)
                        continue;
                    if (found != level.end())
                    {
                        const LevelAccum& a = found->second;
                        const float3 c = VoxelCenter(space, voxel);
                        const double mx = a.x / a.weight - c.x;
                        const double my = a.y / a.weight - c.y;
                        const double mz = a.z / a.weight - c.z;
                        expectedPhi = static_cast<float>(
                            std::fmin(std::sqrt(mx * mx + my * my + mz * mz) - levelSetRadius, space.h));
                    }
                    levelMatch &= std::fabs(phi - expectedPhi) <= 1e-5f;

                    const int i = voxel.x - kCoordOffset;
                    const int j = voxel.y - kCoordOffset;
                    const int k = voxel.z - kCoordOffset;
                    if (!InGrid(grid, i, j, k))
                        continue;
                    const float4 v = AtlasValue<float4>(velocityBytes, dims, texel);
                    const unsigned char m = AtlasValue<unsigned char>(markerBytes, dims, texel);
                    velocityMatch &= Near(v.x, du[FaceIndex(fu, i, j, k)]) && Near(v.y, dv[FaceIndex(fv, i, j, k)]) &&
                                     Near(v.z, dw[FaceIndex(fw, i, j, k)]);
                    markerMatch &= m == denseMarkers[CellIndex(grid, i, j, k)];
                    fluid += m == kCellFluid ? 1 : 0;
                    ++compared;
                }
    }

    CHECK(compared > 0);
    CHECK(fluid > 0);
    CHECK(velocityMatch);
    CHECK(markerMatch);
    CHECK(levelMatch);
    CHECK(apronEmpty > 0);
    CHECK(apronFar);
}
