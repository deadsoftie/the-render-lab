#include "FluidTests.h"
#include "TestFramework.h"
#include "fluid/cuda/DenseFlip.h"
#include "fluid/cuda/DeviceBuffer.h"
#include "fluid/cuda/GridMath.cuh"
#include "fluid/cuda/Seeding.h"
#include "fluid/cuda/Shapes.cuh"

#include <algorithm>
#include <cmath>
#include <vector>

using namespace Fluid;

namespace
{
    double DivergenceRms(const DenseFlip& flip)
    {
        const GridInfo& grid = flip.Grid();
        std::vector<float> u, v, w;
        std::vector<unsigned char> markers;
        if (!flip.U().Download(u) || !flip.V().Download(v) || !flip.W().Download(w) ||
            !flip.Markers().Download(markers))
            return -1.0;

        const int3 du = FaceDims(grid, 0);
        const int3 dv = FaceDims(grid, 1);
        const int3 dw = FaceDims(grid, 2);
        double sum = 0.0;
        int fluidCells = 0;
        for (int c = 0; c < CellCount(grid); ++c)
        {
            if (markers[c] != kCellFluid)
                continue;
            int3 q = CellCoord(grid, c);
            double divergence = u[FaceIndex(du, q.x + 1, q.y, q.z)] - u[FaceIndex(du, q.x, q.y, q.z)] +
                                v[FaceIndex(dv, q.x, q.y + 1, q.z)] - v[FaceIndex(dv, q.x, q.y, q.z)] +
                                w[FaceIndex(dw, q.x, q.y, q.z + 1)] - w[FaceIndex(dw, q.x, q.y, q.z)];
            sum += divergence * divergence;
            ++fluidCells;
        }
        return fluidCells > 0 ? std::sqrt(sum / fluidCells) / grid.h : 0.0;
    }
}

void RunDenseFlipTests()
{
    GridInfo grid;
    grid.dims = make_int3(16, 16, 16);
    grid.origin = make_float3(0.0f, 0.0f, 0.0f);
    grid.h = 1.0f / 16.0f;

    ShapeList seeds;
    Shape& pool = seeds.items[seeds.count++];
    pool.type = ShapeType::Box;
    pool.lo = make_float3(0.0f, 0.0f, 0.0f);
    pool.hi = make_float3(1.0f, 0.5f, 1.0f);
    pool.velocity = make_float3(0.5f, 0.2f, -0.3f);

    ShapeList obstacles;
    Shape& ball = obstacles.items[obstacles.count++];
    ball.type = ShapeType::Sphere;
    ball.center = make_float3(0.5f, 0.25f, 0.5f);
    ball.radius = 0.15f;

    DeviceBuffer<float4> positions;
    DeviceBuffer<float4> velocities;
    int count = 0;
    CHECK(SeedParticles(grid, seeds, obstacles, positions, velocities, count));
    CHECK(count > 0);

    DenseFlip flip;
    CHECK(flip.Init(grid, obstacles, count));
    CHECK(flip.Transfer(positions, velocities, count, 1.0f / 60.0f, make_float3(0.0f, -9.81f, 0.0f)));
    const double before = DivergenceRms(flip);

    CgResult cg = flip.Project(1e-5f, 1000);
    const double after = DivergenceRms(flip);
    CHECK(cg.converged);
    CHECK(before > 1e-2);
    CHECK(after >= 0.0 && after <= 1e-3 * before);

    CHECK(flip.Advect(positions, velocities, count, 1.0f / 60.0f, 0.95f));
    std::vector<float4> p;
    CHECK(positions.Download(p));
    CHECK(static_cast<int>(p.size()) == count);
    CHECK(std::all_of(p.begin(), p.end(), [&](const float4& q) {
        return q.x >= 0.0f && q.x <= 1.0f && q.y >= 0.0f && q.y <= 1.0f && q.z >= 0.0f && q.z <= 1.0f &&
               ShapeSdf(ball, XYZ(q)) >= -1e-4f;
    }));

    StepSettings settings;
    settings.cgTolerance = 1e-4f;
    settings.cgMaxIterations = 500;
    StepStats stats = flip.Step(positions, velocities, count, 1.0f / 60.0f, settings);
    CHECK(stats.substeps >= 1);
    CHECK(stats.cg.converged);
    CHECK(positions.Download(p));
    CHECK(std::all_of(p.begin(), p.end(), [](const float4& q) {
        return std::isfinite(q.x) && std::isfinite(q.y) && std::isfinite(q.z);
    }));
}
