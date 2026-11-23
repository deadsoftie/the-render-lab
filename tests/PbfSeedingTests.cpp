#include <cmath>

#include "PhysicsTests.h"
#include "TestFramework.h"
#include "pbf/PbfSeeding.h"

using namespace Pbf;

namespace
{
    Shape Box(Vec3 lo, Vec3 hi)
    {
        Shape s;
        s.type = ShapeType::Box;
        s.lo = lo;
        s.hi = hi;
        return s;
    }

    Shape Sphere(Vec3 c, float r)
    {
        Shape s;
        s.type = ShapeType::Sphere;
        s.center = c;
        s.radius = r;
        return s;
    }

    Vec3 Pos(const Particle& p)
    {
        return Vec3{p.position[0], p.position[1], p.position[2]};
    }

    SeedDesc Desc(int maxParticles = 100000)
    {
        SeedDesc d;
        d.domainMin = Vec3{0, 0, 0};
        d.domainMax = Vec3{1, 1, 1};
        d.radius = 0.01f;
        d.jitter = 0.01f;
        d.maxParticles = maxParticles;
        return d;
    }

    float MinPairDistance(const std::vector<Particle>& ps)
    {
        float best = 1e30f;
        for (size_t i = 0; i < ps.size(); ++i)
            for (size_t j = i + 1; j < ps.size(); ++j)
                best = std::fmin(best, Anim::Length(Pos(ps[i]) - Pos(ps[j])));
        return best;
    }
}

void RunPbfSeedingTests()
{
    CHECK_NEAR(ShapeSdf(Box(Vec3{0, 0, 0}, Vec3{1, 1, 1}), Vec3{0.5f, 0.5f, 0.5f}), -0.5f, 1e-6f);
    CHECK_NEAR(ShapeSdf(Box(Vec3{0, 0, 0}, Vec3{1, 1, 1}), Vec3{2.0f, 0.5f, 0.5f}), 1.0f, 1e-6f);
    CHECK_NEAR(ShapeSdf(Sphere(Vec3{0, 0, 0}, 0.5f), Vec3{1, 0, 0}), 0.5f, 1e-6f);

    std::vector<Particle> out;
    std::vector<Shape> seeds = {Box(Vec3{0.1f, 0.1f, 0.1f}, Vec3{0.29f, 0.29f, 0.29f})};
    int count = SeedLattice(Desc(), seeds, {}, out);
    CHECK(count == 10 * 10 * 10);
    CHECK(static_cast<int>(out.size()) == count);
    for (const Particle& p : out)
    {
        CHECK(ShapeSdf(seeds[0], Pos(p)) < 0.001f);
        CHECK(p.position[3] == 1.0f);
    }
    float minDist = MinPairDistance(out);
    CHECK(minDist > 0.02f * 0.98f);
    CHECK(minDist < 0.02f * 1.02f);

    std::vector<Shape> overlapping = {Box(Vec3{0.1f, 0.1f, 0.1f}, Vec3{0.29f, 0.29f, 0.29f}),
                                      Box(Vec3{0.2f, 0.1f, 0.1f}, Vec3{0.39f, 0.29f, 0.29f})};
    std::vector<Particle> merged;
    SeedLattice(Desc(), overlapping, {}, merged);
    CHECK(merged.size() == 15u * 10u * 10u);

    std::vector<Particle> blocked;
    std::vector<Shape> obstacles = {Sphere(Vec3{0.2f, 0.2f, 0.2f}, 0.05f)};
    SeedLattice(Desc(), seeds, obstacles, blocked);
    CHECK(blocked.size() < out.size());
    for (const Particle& p : blocked)
        CHECK(ShapeSdf(obstacles[0], Pos(p)) >= 0.01f - 1e-3f);

    std::vector<Particle> capped;
    CHECK(SeedLattice(Desc(123), seeds, {}, capped) == 123);

    std::vector<Particle> clipped;
    SeedLattice(Desc(), {Box(Vec3{-1, -1, -1}, Vec3{0.05f, 0.05f, 0.05f})}, {}, clipped);
    for (const Particle& p : clipped)
        for (int c = 0; c < 3; ++c)
            CHECK(p.position[c] >= 0.01f - 1e-3f);

    Shape moving = Box(Vec3{0.5f, 0.5f, 0.5f}, Vec3{0.55f, 0.55f, 0.55f});
    moving.velocity = Vec3{1.0f, -2.0f, 0.5f};
    std::vector<Particle> withVelocity;
    SeedLattice(Desc(), {moving}, {}, withVelocity);
    CHECK(!withVelocity.empty());
    CHECK(withVelocity[0].velocity[1] == -2.0f);
}
