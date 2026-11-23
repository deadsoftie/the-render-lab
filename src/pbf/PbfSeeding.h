#pragma once
#include <vector>

#include "anim/Math/Vec3.h"

namespace Pbf
{
    using Anim::Vec3;

    enum class ShapeType
    {
        Box,
        Sphere
    };

    // Box or sphere. velocity and rate are for emitters.
    struct Shape
    {
        ShapeType type = ShapeType::Box;
        Vec3 lo;
        Vec3 hi;
        Vec3 center;
        float radius = 0.0f;
        Vec3 velocity;
        float rate = 0.0f;
    };

    struct Particle
    {
        float position[4] = {0.0f, 0.0f, 0.0f, 1.0f};
        float velocity[4] = {0.0f, 0.0f, 0.0f, 0.0f};
    };

    // Negative inside.
    float ShapeSdf(const Shape& shape, const Vec3& p);
    // -1 if none.
    int FirstContaining(const std::vector<Shape>& shapes, const Vec3& p, float margin);

    struct SeedDesc
    {
        Vec3 domainMin;
        Vec3 domainMax;
        float radius = 0.01f;
        float jitter = 0.01f;
        int maxParticles = 100000;
        unsigned int seed = 1u;
    };

    // Jittered 2r lattice in the seeds, skipping obstacles and earlier seeds.
    int SeedLattice(const SeedDesc& desc,
                    const std::vector<Shape>& seeds,
                    const std::vector<Shape>& obstacles,
                    std::vector<Particle>& out);
}
