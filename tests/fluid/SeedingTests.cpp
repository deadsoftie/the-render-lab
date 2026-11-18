#include "FluidTests.h"
#include "TestFramework.h"
#include "fluid/cuda/DeviceBuffer.h"
#include "fluid/cuda/Seeding.h"
#include "fluid/cuda/Shapes.cuh"

#include <algorithm>
#include <vector>

using namespace Fluid;

namespace
{
    Shape MakeBox(float3 lo, float3 hi, float3 velocity = make_float3(0.0f, 0.0f, 0.0f))
    {
        Shape shape;
        shape.type = ShapeType::Box;
        shape.lo = lo;
        shape.hi = hi;
        shape.velocity = velocity;
        return shape;
    }

    Shape MakeSphere(float3 center, float radius)
    {
        Shape shape;
        shape.type = ShapeType::Sphere;
        shape.center = center;
        shape.radius = radius;
        return shape;
    }
}

void RunSeedingTests()
{
    GridInfo grid;
    grid.dims = make_int3(8, 8, 8);
    grid.origin = make_float3(0.0f, 0.0f, 0.0f);
    grid.h = 0.125f;

    ShapeList seeds;
    seeds.items[seeds.count++] =
        MakeBox(make_float3(0.0f, 0.0f, 0.0f), make_float3(0.5f, 0.25f, 1.0f), make_float3(1.0f, 2.0f, 3.0f));
    ShapeList noObstacles;

    DeviceBuffer<float4> positions;
    DeviceBuffer<float4> velocities;
    int count = 0;
    CHECK(SeedParticles(grid, seeds, noObstacles, positions, velocities, count));

    const int perCell = kParticlesPerCellAxis * kParticlesPerCellAxis * kParticlesPerCellAxis;
    CHECK(count == 4 * 2 * 8 * perCell);

    std::vector<float4> p;
    std::vector<float4> v;
    CHECK(positions.Download(p));
    CHECK(velocities.Download(v));
    CHECK(static_cast<int>(p.size()) == count);
    CHECK(std::all_of(p.begin(), p.end(), [&](const float4& q) {
        return ShapeSdf(seeds.items[0], XYZ(q)) < 0.0f;
    }));
    CHECK(std::all_of(v.begin(), v.end(), [](const float4& q) {
        return q.x == 1.0f && q.y == 2.0f && q.z == 3.0f;
    }));

    ShapeList obstacles;
    obstacles.items[obstacles.count++] = MakeSphere(make_float3(0.25f, 0.125f, 0.5f), 0.2f);
    int blocked = 0;
    CHECK(SeedParticles(grid, seeds, obstacles, positions, velocities, blocked));
    CHECK(blocked > 0 && blocked < count);
    CHECK(positions.Download(p));
    CHECK(std::none_of(p.begin(), p.end(), [&](const float4& q) {
        return ShapeSdf(obstacles.items[0], XYZ(q)) < 0.0f;
    }));

    ShapeList outside;
    outside.items[outside.count++] = MakeSphere(make_float3(5.0f, 5.0f, 5.0f), 0.5f);
    int none = -1;
    CHECK(SeedParticles(grid, outside, noObstacles, positions, velocities, none));
    CHECK(none == 0);

    CHECK(SeedParticles(grid, seeds, noObstacles, positions, velocities, count));
    std::vector<float4> before;
    CHECK(positions.Download(before));
    ShapeList emitter;
    emitter.items[emitter.count++] =
        MakeBox(make_float3(0.5f, 0.5f, 0.5f), make_float3(0.75f, 0.75f, 0.75f), make_float3(0.0f, -1.0f, 0.0f));
    int appended = count;
    CHECK(AppendParticles(grid, emitter, noObstacles, positions, velocities, appended, 1 << 20, 7u));
    CHECK(appended == count + 2 * 2 * 2 * perCell);
    CHECK(positions.Download(p));
    CHECK(velocities.Download(v));
    CHECK(static_cast<int>(p.size()) == appended);
    CHECK(std::equal(before.begin(), before.end(), p.begin(), [](const float4& a, const float4& b) {
        return a.x == b.x && a.y == b.y && a.z == b.z;
    }));
    CHECK(std::all_of(p.begin() + count, p.end(), [&](const float4& q) {
        return ShapeSdf(emitter.items[0], XYZ(q)) < 0.0f;
    }));
    CHECK(std::all_of(v.begin() + count, v.end(), [](const float4& q) { return q.y == -1.0f; }));

    int capped = appended;
    CHECK(AppendParticles(grid, emitter, noObstacles, positions, velocities, capped, appended + 5, 8u));
    CHECK(capped == appended + 5);
    CHECK(AppendParticles(grid, emitter, noObstacles, positions, velocities, capped, capped, 9u));
    CHECK(capped == appended + 5);

    GridInfo huge;
    huge.dims = make_int3(1024, 512, 1024);
    huge.origin = make_float3(-8.0f, 0.0f, -8.0f);
    huge.h = 1.0f / 64.0f;
    ShapeList small;
    small.items[small.count++] = MakeBox(make_float3(0.0f, 1.0f, 0.0f), make_float3(0.125f, 1.125f, 0.125f));
    int hugeCount = 0;
    CHECK(SeedParticles(huge, small, noObstacles, positions, velocities, hugeCount));
    CHECK(hugeCount == 8 * 8 * 8 * perCell);
}
