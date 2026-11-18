#include "FluidTests.h"
#include "TestFramework.h"
#include "fluid/cuda/DenseFlip.h"
#include "fluid/cuda/DeviceBuffer.h"
#include "fluid/cuda/Particles.h"
#include "fluid/cuda/Seeding.h"
#include "fluid/cuda/SparseFlip.h"

#include <algorithm>
#include <cmath>
#include <vector>

using namespace Fluid;

namespace
{
    constexpr float kTwoPi = 6.28318530718f;

    GridInfo UnitGrid()
    {
        GridInfo grid;
        grid.dims = make_int3(16, 16, 16);
        grid.origin = make_float3(0.0f, 0.0f, 0.0f);
        grid.h = 1.0f / 16.0f;
        return grid;
    }

    ShapeList Pool(float height)
    {
        ShapeList seeds;
        Shape& pool = seeds.items[seeds.count++];
        pool.type = ShapeType::Box;
        pool.lo = make_float3(0.0f, 0.0f, 0.0f);
        pool.hi = make_float3(1.0f, height, 1.0f);
        return seeds;
    }

    float Distance(float4 a, float4 b)
    {
        const float dx = a.x - b.x;
        const float dy = a.y - b.y;
        const float dz = a.z - b.z;
        return std::sqrt(dx * dx + dy * dy + dz * dz);
    }

    void CompareWithDense()
    {
        const GridInfo grid = UnitGrid();
        const float dt = 1.0f / 60.0f;
        const float3 gravity = make_float3(0.0f, -9.81f, 0.0f);
        const float flipRatio = 0.95f;

        const ShapeList seeds = Pool(0.375f);
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

        std::vector<float4> hostPositions;
        CHECK(positions.Download(hostPositions));
        std::vector<float4> hostVelocities(count);
        for (int i = 0; i < count; ++i)
        {
            float4& p = hostPositions[i];
            p.w = static_cast<float>(i);
            hostVelocities[i] = make_float4(0.5f * std::sin(kTwoPi * p.y),
                                            0.5f * std::sin(kTwoPi * p.z),
                                            0.5f * std::sin(kTwoPi * p.x),
                                            0.0f);
        }

        DeviceBuffer<float4> densePositions;
        DeviceBuffer<float4> denseVelocities;
        CHECK(positions.Upload(hostPositions) && velocities.Upload(hostVelocities));
        CHECK(densePositions.Upload(hostPositions) && denseVelocities.Upload(hostVelocities));

        DenseFlip dense;
        CHECK(dense.Init(grid, obstacles, count));
        CHECK(dense.Transfer(densePositions, denseVelocities, count, dt, gravity));
        CHECK(dense.Project(1e-6f, 4000).converged);
        CHECK(dense.Advect(densePositions, denseVelocities, count, dt, flipRatio));

        SparseFlip sparse;
        sparse.Init(grid, obstacles);
        CHECK(sparse.Rebuild(positions.Data(), count, false));
        CHECK(sparse.CountUncovered(positions.Data(), count) == 0);
        CHECK(sparse.Transfer(positions.Data(), velocities.Data(), count));
        CHECK(sparse.Project(dt, gravity, 1e-6f, 4000).converged);
        CHECK(sparse.Advect(positions.Data(), velocities.Data(), count, dt, flipRatio));

        std::vector<float4> dp, dv, sp, sv;
        CHECK(densePositions.Download(dp) && denseVelocities.Download(dv));
        CHECK(positions.Download(sp) && velocities.Download(sv));

        std::vector<int> denseById(count, -1);
        bool idsValid = true;
        for (int i = 0; i < count; ++i)
        {
            const int id = static_cast<int>(dp[i].w);
            if (id < 0 || id >= count || denseById[id] != -1)
                idsValid = false;
            else
                denseById[id] = i;
        }
        CHECK(idsValid);
        if (!idsValid)
            return;

        float maxVelocity = 0.0f;
        for (const float4& v : dv)
            maxVelocity = std::max(maxVelocity, std::sqrt(v.x * v.x + v.y * v.y + v.z * v.z));

        float positionError = 0.0f;
        int velocityMismatches = 0;
        bool sparseIdsKept = true;
        for (int i = 0; i < count; ++i)
        {
            sparseIdsKept &= static_cast<int>(sp[i].w) == i;
            const int d = denseById[i];
            positionError = std::max(positionError, Distance(sp[i], dp[d]));
            if (Distance(sv[i], dv[d]) > 2e-2f * (1.0f + maxVelocity))
                ++velocityMismatches;
        }

        CHECK(sparseIdsKept);
        CHECK(maxVelocity > 0.0f);
        CHECK(positionError <= 5e-2f * grid.h);
        CHECK(velocityMismatches <= count / 1000);
    }

    void PoolAtRest()
    {
        const GridInfo grid = UnitGrid();
        const float dt = 1.0f / 120.0f;
        const float3 gravity = make_float3(0.0f, -9.81f, 0.0f);
        const int substeps = 30;

        DeviceBuffer<float4> positions;
        DeviceBuffer<float4> velocities;
        int count = 0;
        CHECK(SeedParticles(grid, Pool(0.375f), ShapeList{}, positions, velocities, count));

        SparseFlip sparse;
        sparse.Init(grid, ShapeList{});
        bool stepped = true;
        bool covered = true;
        bool converged = true;
        for (int s = 0; s < substeps && stepped; ++s)
        {
            stepped = sparse.Rebuild(positions.Data(), count, true) &&
                      sparse.Transfer(positions.Data(), velocities.Data(), count);
            covered &= sparse.CountUncovered(positions.Data(), count) == 0;
            converged &= sparse.Project(dt, gravity, 1e-5f, 2000).converged;
            stepped = stepped && sparse.Advect(positions.Data(), velocities.Data(), count, dt, 0.95f);
        }
        CHECK(stepped);
        CHECK(covered);
        CHECK(converged);
        CHECK(!sparse.Topology().LastWasFull());

        DeviceBuffer<unsigned int> scratch;
        CHECK(MaxParticleSpeed(velocities.Data(), count, scratch) < 0.1f);

        std::vector<float4> p;
        CHECK(positions.Download(p));
        CHECK(static_cast<int>(p.size()) == count);
        CHECK(std::all_of(p.begin(), p.end(), [](const float4& q) {
            return std::isfinite(q.x) && std::isfinite(q.y) && std::isfinite(q.z) && q.x >= 0.0f &&
                   q.x <= 1.0f && q.y >= 0.0f && q.y <= 0.5f && q.z >= 0.0f && q.z <= 1.0f;
        }));
    }
}

void RunSparseFlipTests()
{
    CompareWithDense();
    PoolAtRest();
}
