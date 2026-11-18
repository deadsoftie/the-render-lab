#include "FluidTests.h"
#include "TestFramework.h"
#include "fluid/cuda/BrickAtlas.h"
#include "fluid/cuda/DenseFlip.h"
#include "fluid/cuda/DeviceBuffer.h"
#include "fluid/cuda/GridMath.cuh"
#include "fluid/cuda/Seeding.h"
#include "fluid/cuda/SparseGather.h"
#include "fluid/cuda/SparseGrid.cuh"
#include "fluid/cuda/SparseProject.h"
#include "fluid/cuda/SparseTopology.h"
#include "fluid/cuda/SubcellLists.h"

#include <algorithm>
#include <cmath>
#include <cstring>
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

    int3 Step(int3 t, int axis)
    {
        return make_int3(t.x + (axis == 0 ? 1 : 0), t.y + (axis == 1 ? 1 : 0), t.z + (axis == 2 ? 1 : 0));
    }

    float Component(float4 v, int axis)
    {
        return axis == 0 ? v.x : (axis == 1 ? v.y : v.z);
    }
}

void RunSparseProjectTests()
{
    GridInfo grid;
    grid.dims = make_int3(16, 16, 16);
    grid.origin = make_float3(0.0f, 0.0f, 0.0f);
    grid.h = 1.0f / 16.0f;
    const float dt = 1.0f / 60.0f;
    const float3 gravity = make_float3(0.0f, -9.81f, 0.0f);
    const float tolerance = 1e-6f;
    const int maxIterations = 4000;

    ShapeList seeds;
    Shape& pool = seeds.items[seeds.count++];
    pool.type = ShapeType::Box;
    pool.lo = make_float3(0.0f, 0.0f, 0.0f);
    pool.hi = make_float3(1.0f, 0.375f, 1.0f);

    ShapeList obstacles;
    Shape& ball = obstacles.items[obstacles.count++];
    ball.type = ShapeType::Sphere;
    ball.center = make_float3(0.5f, 0.2f, 0.5f);
    ball.radius = 0.15f;

    DeviceBuffer<float4> positions;
    DeviceBuffer<float4> velocities;
    int count = 0;
    CHECK(SeedParticles(grid, seeds, obstacles, positions, velocities, count));

    std::vector<float4> hostVelocities(count);
    unsigned int seed = 2718u;
    auto next = [&seed]()
    {
        seed = seed * 1664525u + 1013904223u;
        return static_cast<float>(seed >> 8) / 16777216.0f * 2.0f - 1.0f;
    };
    for (float4& v : hostVelocities)
        v = make_float4(next(), next(), next(), 0.0f);
    CHECK(velocities.Upload(hostVelocities));

    DenseFlip dense;
    CHECK(dense.Init(grid, obstacles, count));
    CHECK(dense.Transfer(positions, velocities, count, dt, gravity));
    const CgResult denseCg = dense.Project(tolerance, maxIterations);
    CHECK(denseCg.converged);
    std::vector<float> du, dv, dw, dp;
    std::vector<unsigned char> denseMarkers;
    CHECK(dense.U().Download(du) && dense.V().Download(dv) && dense.W().Download(dw));
    CHECK(dense.Pressure().Download(dp) && dense.Markers().Download(denseMarkers));

    VoxelSpace space;
    space.origin = grid.origin;
    space.h = grid.h;
    SparseTopology topology;
    BrickAtlas atlas;
    SubcellLists lists;
    SparseCg cg;
    CHECK(topology.FullRebuild(positions.Data(), count, space, grid.h));
    CHECK(atlas.Reserve(topology.BrickCapacity()) && atlas.BuildBrickTable(topology) && atlas.ClearActive());
    CHECK(lists.Build(positions.Data(), velocities.Data(), count, topology, grid.h));
    CHECK(SparseParticleToGrid(
        lists, atlas, topology, obstacles, DomainVoxels(grid), kLevelSetRadiusScale * grid.h));
    const CgResult sparseCg = SparseProject(atlas, topology, cg, gravity, dt, tolerance, maxIterations);
    CHECK(sparseCg.converged);
    CHECK(sparseCg.iterations > 0 && sparseCg.iterations % SparseCg::kIterationsPerGraph == 0);

    std::vector<unsigned char> pressureBytes, velocityBytes, markerBytes, divergenceBytes;
    CHECK(atlas.Download(AtlasChannel::Pressure, pressureBytes));
    CHECK(atlas.Download(AtlasChannel::Velocity, velocityBytes));
    CHECK(atlas.Download(AtlasChannel::Marker, markerBytes));
    CHECK(atlas.Download(AtlasChannel::Divergence, divergenceBytes));
    std::vector<Node> nodes;
    CHECK(topology.DownloadNodes(nodes));

    float maxPressure = 0.0f;
    float maxVelocity = 0.0f;
    for (float p : dp)
        maxPressure = std::max(maxPressure, std::fabs(p));
    for (const std::vector<float>* faces : {&du, &dv, &dw})
        for (float f : *faces)
            maxVelocity = std::max(maxVelocity, std::fabs(f));

    const int3 dims = atlas.TexelDims();
    const int3 faceDims[3] = {FaceDims(grid, 0), FaceDims(grid, 1), FaceDims(grid, 2)};
    const std::vector<float>* denseFaces[3] = {&du, &dv, &dw};
    float pressureError = 0.0f;
    float velocityError = 0.0f;
    bool markersMatch = true;
    double before = 0.0;
    double after = 0.0;
    int fluidCells = 0;
    for (const Node& node : nodes)
    {
        if (node.level != 0)
            continue;
        for (int z = 0; z < kBrickDim; ++z)
            for (int y = 0; y < kBrickDim; ++y)
                for (int x = 0; x < kBrickDim; ++x)
                {
                    const int3 voxel = BrickVoxel(node.coord, make_int3(x, y, z));
                    const int3 texel = AtlasTexel(node.brick, make_int3(x, y, z));
                    const int i = voxel.x - kCoordOffset;
                    const int j = voxel.y - kCoordOffset;
                    const int k = voxel.z - kCoordOffset;
                    if (!InGrid(grid, i, j, k))
                        continue;

                    const unsigned char marker = AtlasValue<unsigned char>(markerBytes, dims, texel);
                    markersMatch &= marker == denseMarkers[CellIndex(grid, i, j, k)];
                    const float4 v = AtlasValue<float4>(velocityBytes, dims, texel);
                    for (int axis = 0; axis < 3; ++axis)
                        velocityError = std::max(
                            velocityError,
                            std::fabs(Component(v, axis) - (*denseFaces[axis])[FaceIndex(faceDims[axis], i, j, k)]));

                    if (marker != kCellFluid)
                        continue;
                    ++fluidCells;
                    pressureError = std::max(
                        pressureError,
                        std::fabs(AtlasValue<float>(pressureBytes, dims, texel) - dp[CellIndex(grid, i, j, k)]));

                    const double b = AtlasValue<float>(divergenceBytes, dims, texel);
                    before += (b / grid.h) * (b / grid.h);
                    double sum = 0.0;
                    for (int axis = 0; axis < 3; ++axis)
                        sum += Component(AtlasValue<float4>(velocityBytes, dims, Step(texel, axis)), axis) -
                               Component(v, axis);
                    after += sum * sum;
                }
    }

    CHECK(fluidCells > 0);
    CHECK(markersMatch);
    CHECK(maxPressure > 0.0f);
    CHECK(pressureError <= 1e-3f * maxPressure);
    CHECK(velocityError <= 1e-3f * (1.0f + maxVelocity));
    CHECK(before > 0.0);
    CHECK(std::sqrt(after) <= 1e-3 * std::sqrt(before));
}
