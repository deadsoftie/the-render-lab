#include "pbf/PbfWaterLevel.h"

#include <algorithm>
#include <cmath>

namespace Pbf
{
    namespace
    {
        constexpr int kSamplesPerSide = 4;
        constexpr float kMaxSlope = 1.0f;

        const float* Column(const ColumnField& field, int i, int k)
        {
            return field.data + (static_cast<size_t>(k) * field.nx + i) * 4;
        }
    }

    float ColumnHeight(const ColumnField& field, int i, int k)
    {
        return field.floorY + Column(field, i, k)[3] * field.particleVolume / (field.cell * field.cell);
    }

    bool SampleColumns(const ColumnField& field, float x, float z, float& height, Vec3& velocity)
    {
        if (field.data == nullptr || field.nx < 1 || field.nz < 1)
            return false;
        float maxX = field.originX + field.nx * field.cell;
        float maxZ = field.originZ + field.nz * field.cell;
        if (x < field.originX || z < field.originZ || x > maxX || z > maxZ)
            return false;

        float fx = (x - field.originX) / field.cell - 0.5f;
        float fz = (z - field.originZ) / field.cell - 0.5f;
        int i0 = static_cast<int>(std::floor(fx));
        int k0 = static_cast<int>(std::floor(fz));
        float tx = fx - static_cast<float>(i0);
        float tz = fz - static_cast<float>(k0);
        int i1 = std::clamp(i0 + 1, 0, field.nx - 1);
        int k1 = std::clamp(k0 + 1, 0, field.nz - 1);
        i0 = std::clamp(i0, 0, field.nx - 1);
        k0 = std::clamp(k0, 0, field.nz - 1);

        const int is[4] = {i0, i1, i0, i1};
        const int ks[4] = {k0, k0, k1, k1};
        const float ws[4] = {(1.0f - tx) * (1.0f - tz), tx * (1.0f - tz), (1.0f - tx) * tz, tx * tz};
        height = 0.0f;
        velocity = Vec3{};
        for (int s = 0; s < 4; ++s)
        {
            const float* c = Column(field, is[s], ks[s]);
            height += ws[s] * ColumnHeight(field, is[s], ks[s]);
            velocity = velocity + Vec3{c[0], c[1], c[2]} * ws[s];
        }
        return true;
    }

    Footprint BoxFootprint(const Vec3& center, const Vec3& axisX, const Vec3& axisY, const Vec3& axisZ, const Vec3& half)
    {
        Footprint f;
        f.centerX = center.x;
        f.centerZ = center.z;
        float ux = axisX.x;
        float uz = axisX.z;
        float len = std::sqrt(ux * ux + uz * uz);
        if (len < 0.2f)
        {
            ux = axisZ.x;
            uz = axisZ.z;
            len = std::sqrt(ux * ux + uz * uz);
        }
        if (len < 1e-6f)
        {
            ux = 1.0f;
            uz = 0.0f;
            len = 1.0f;
        }
        f.axisX = ux / len;
        f.axisZ = uz / len;
        const Vec3* axes[3] = {&axisX, &axisY, &axisZ};
        const float halves[3] = {half.x, half.y, half.z};
        for (int a = 0; a < 3; ++a)
        {
            float du = axes[a]->x * f.axisX + axes[a]->z * f.axisZ;
            float dv = -axes[a]->x * f.axisZ + axes[a]->z * f.axisX;
            f.halfU += std::fabs(du) * halves[a];
            f.halfV += std::fabs(dv) * halves[a];
        }
        return f;
    }

    bool InsideFootprint(const Footprint& f, float x, float z, float margin)
    {
        float dx = x - f.centerX;
        float dz = z - f.centerZ;
        float du = dx * f.axisX + dz * f.axisZ;
        float dv = -dx * f.axisZ + dz * f.axisX;
        return std::fabs(du) < f.halfU + margin && std::fabs(dv) < f.halfV + margin;
    }

    WaterPlane FitWaterPlane(const ColumnField& field,
                             const std::vector<Footprint>& footprints,
                             int body,
                             float margin,
                             float fallbackLevel)
    {
        WaterPlane plane;
        plane.level = fallbackLevel;
        if (body < 0 || body >= static_cast<int>(footprints.size()))
            return plane;
        const Footprint& f = footprints[body];
        plane.refX = f.centerX;
        plane.refZ = f.centerZ;

        float hu = f.halfU + margin;
        float hv = f.halfV + margin;
        const float corners[5][2] = {{hu, hv}, {-hu, hv}, {-hu, -hv}, {hu, -hv}, {hu, hv}};
        double n = 0.0, sx = 0.0, sz = 0.0, sxx = 0.0, sxz = 0.0, szz = 0.0, sh = 0.0, sxh = 0.0, szh = 0.0;
        Vec3 velocity;
        for (int side = 0; side < 4; ++side)
        {
            for (int s = 0; s < kSamplesPerSide; ++s)
            {
                float t = static_cast<float>(s) / kSamplesPerSide;
                float u = corners[side][0] + (corners[side + 1][0] - corners[side][0]) * t;
                float v = corners[side][1] + (corners[side + 1][1] - corners[side][1]) * t;
                float x = f.centerX + u * f.axisX - v * f.axisZ;
                float z = f.centerZ + u * f.axisZ + v * f.axisX;

                bool blocked = false;
                for (size_t o = 0; o < footprints.size() && !blocked; ++o)
                    blocked = static_cast<int>(o) != body && InsideFootprint(footprints[o], x, z, 0.5f * margin);
                float h = 0.0f;
                Vec3 vel;
                if (blocked || !SampleColumns(field, x, z, h, vel))
                    continue;

                double dx = x - f.centerX;
                double dz = z - f.centerZ;
                n += 1.0;
                sx += dx;
                sz += dz;
                sxx += dx * dx;
                sxz += dx * dz;
                szz += dz * dz;
                sh += h;
                sxh += dx * h;
                szh += dz * h;
                velocity = velocity + vel;
            }
        }
        if (n < 1.0)
            return plane;

        plane.valid = true;
        plane.samples = static_cast<int>(n);
        plane.velocity = velocity * static_cast<float>(1.0 / n);
        plane.level = static_cast<float>(sh / n);

        double det = n * (sxx * szz - sxz * sxz) - sx * (sx * szz - sxz * sz) + sz * (sx * sxz - sxx * sz);
        double scale = n * (sxx + szz) * (sxx + szz) + 1e-12;
        if (n < 3.0 || std::fabs(det) < 1e-6 * scale)
            return plane;

        double a = (sh * (sxx * szz - sxz * sxz) - sx * (sxh * szz - sxz * szh) + sz * (sxh * sxz - sxx * szh)) / det;
        double b = (n * (sxh * szz - sxz * szh) - sh * (sx * szz - sxz * sz) + sz * (sx * szh - sxh * sz)) / det;
        double c = (n * (sxx * szh - sxh * sxz) - sx * (sx * szh - sxh * sz) + sh * (sx * sxz - sxx * sz)) / det;
        plane.level = static_cast<float>(a);
        plane.slopeX = std::clamp(static_cast<float>(b), -kMaxSlope, kMaxSlope);
        plane.slopeZ = std::clamp(static_cast<float>(c), -kMaxSlope, kMaxSlope);
        return plane;
    }
}
