#include "pbf/PbfSeeding.h"

#include <algorithm>
#include <cmath>

namespace Pbf
{
    namespace
    {
        float Jitter(unsigned int& state)
        {
            state ^= state << 13;
            state ^= state >> 17;
            state ^= state << 5;
            return static_cast<float>(state & 0xFFFFFFu) / static_cast<float>(0xFFFFFFu) * 2.0f - 1.0f;
        }

        void ShapeBounds(const Shape& shape, Vec3& lo, Vec3& hi)
        {
            if (shape.type == ShapeType::Sphere)
            {
                Vec3 r{shape.radius, shape.radius, shape.radius};
                lo = shape.center - r;
                hi = shape.center + r;
                return;
            }
            lo = shape.lo;
            hi = shape.hi;
        }
    }

    float ShapeSdf(const Shape& shape, const Vec3& p)
    {
        if (shape.type == ShapeType::Sphere)
            return Anim::Length(p - shape.center) - shape.radius;

        Vec3 center = (shape.lo + shape.hi) * 0.5f;
        Vec3 half = (shape.hi - shape.lo) * 0.5f;
        Vec3 d = p - center;
        Vec3 q{std::fabs(d.x) - half.x, std::fabs(d.y) - half.y, std::fabs(d.z) - half.z};
        Vec3 outside{std::max(q.x, 0.0f), std::max(q.y, 0.0f), std::max(q.z, 0.0f)};
        float inside = std::min(std::max(q.x, std::max(q.y, q.z)), 0.0f);
        return Anim::Length(outside) + inside;
    }

    int FirstContaining(const std::vector<Shape>& shapes, const Vec3& p, float margin)
    {
        for (size_t i = 0; i < shapes.size(); ++i)
            if (ShapeSdf(shapes[i], p) < margin)
                return static_cast<int>(i);
        return -1;
    }

    int SeedLattice(const SeedDesc& desc,
                    const std::vector<Shape>& seeds,
                    const std::vector<Shape>& obstacles,
                    std::vector<Particle>& out)
    {
        out.clear();
        float spacing = 2.0f * desc.radius;
        if (spacing <= 0.0f || desc.maxParticles <= 0)
            return 0;

        Vec3 origin = desc.domainMin + Vec3{desc.radius, desc.radius, desc.radius};
        Vec3 limit = desc.domainMax - Vec3{desc.radius, desc.radius, desc.radius};
        unsigned int state = desc.seed == 0 ? 1u : desc.seed;
        float jitter = desc.jitter * desc.radius;

        for (size_t s = 0; s < seeds.size(); ++s)
        {
            Vec3 lo, hi;
            ShapeBounds(seeds[s], lo, hi);
            lo = Vec3{std::max(lo.x, origin.x), std::max(lo.y, origin.y), std::max(lo.z, origin.z)};
            hi = Vec3{std::min(hi.x, limit.x), std::min(hi.y, limit.y), std::min(hi.z, limit.z)};
            if (lo.x > hi.x || lo.y > hi.y || lo.z > hi.z)
                continue;

            int i0 = static_cast<int>(std::ceil((lo.x - origin.x) / spacing - 1e-4f));
            int j0 = static_cast<int>(std::ceil((lo.y - origin.y) / spacing - 1e-4f));
            int k0 = static_cast<int>(std::ceil((lo.z - origin.z) / spacing - 1e-4f));
            int i1 = static_cast<int>(std::floor((hi.x - origin.x) / spacing + 1e-4f));
            int j1 = static_cast<int>(std::floor((hi.y - origin.y) / spacing + 1e-4f));
            int k1 = static_cast<int>(std::floor((hi.z - origin.z) / spacing + 1e-4f));

            for (int j = j0; j <= j1; ++j)
            {
                for (int k = k0; k <= k1; ++k)
                {
                    for (int i = i0; i <= i1; ++i)
                    {
                        Vec3 p = origin + Vec3{i * spacing, j * spacing, k * spacing};
                        if (ShapeSdf(seeds[s], p) > 1e-5f)
                            continue;
                        bool claimed = false;
                        for (size_t e = 0; e < s && !claimed; ++e)
                            claimed = ShapeSdf(seeds[e], p) <= 1e-5f;
                        if (claimed || FirstContaining(obstacles, p, desc.radius) >= 0)
                            continue;
                        if (static_cast<int>(out.size()) >= desc.maxParticles)
                            return static_cast<int>(out.size());

                        Particle particle;
                        particle.position[0] = p.x + Jitter(state) * jitter;
                        particle.position[1] = p.y + Jitter(state) * jitter;
                        particle.position[2] = p.z + Jitter(state) * jitter;
                        particle.velocity[0] = seeds[s].velocity.x;
                        particle.velocity[1] = seeds[s].velocity.y;
                        particle.velocity[2] = seeds[s].velocity.z;
                        out.push_back(particle);
                    }
                }
            }
        }
        return static_cast<int>(out.size());
    }
}
