#pragma once
#include "fluid/cuda/FluidTypes.h"
#include "fluid/cuda/Shapes.cuh"
#include "fluid/cuda/VecMath.cuh"

namespace Fluid
{
    constexpr float kWallMargin = 0.01f;
    constexpr float kNormalEpsilon = 0.05f;

    // Pushes a particle out of analytic obstacles along the SDF normal and clamps it just inside
    // the domain, removing the velocity component that points into the obstacle or wall.
    __device__ inline void CollideParticle(const GridInfo& grid, const ShapeList& obstacles, float3& x, float3& vel)
    {
        for (int s = 0; s < obstacles.count; ++s)
        {
            float d = ShapeSdf(obstacles.items[s], x);
            if (d >= 0.0f)
                continue;
            float3 n = ShapeNormal(obstacles.items[s], x, kNormalEpsilon * grid.h);
            x = x - n * d;
            float vn = Dot(vel, n);
            if (vn < 0.0f)
                vel = vel - n * vn;
        }

        float margin = kWallMargin * grid.h;
        float3 lo = grid.origin + make_float3(margin, margin, margin);
        float3 hi = grid.origin + make_float3(grid.dims.x, grid.dims.y, grid.dims.z) * grid.h -
                    make_float3(margin, margin, margin);
        if (x.x < lo.x) { x.x = lo.x; vel.x = fmaxf(vel.x, 0.0f); }
        if (x.x > hi.x) { x.x = hi.x; vel.x = fminf(vel.x, 0.0f); }
        if (x.y < lo.y) { x.y = lo.y; vel.y = fmaxf(vel.y, 0.0f); }
        if (x.y > hi.y) { x.y = hi.y; vel.y = fminf(vel.y, 0.0f); }
        if (x.z < lo.z) { x.z = lo.z; vel.z = fmaxf(vel.z, 0.0f); }
        if (x.z > hi.z) { x.z = hi.z; vel.z = fminf(vel.z, 0.0f); }
    }
}
