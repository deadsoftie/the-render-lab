#pragma once
#include <vector>

#include "anim/Math/Vec3.h"

// Water-level coupling: column heights and a local water plane per plank.
namespace Pbf
{
    using Anim::Vec3;

    // Per column: (mean velocity, count).
    struct ColumnField
    {
        int nx = 0;
        int nz = 0;
        float cell = 1.0f;
        float originX = 0.0f;
        float originZ = 0.0f;
        float floorY = 0.0f;
        float particleVolume = 0.0f;
        const float* data = nullptr;
    };

    // Plank rectangle on xz: centre, long axis, half sizes.
    struct Footprint
    {
        float centerX = 0.0f;
        float centerZ = 0.0f;
        float axisX = 1.0f;
        float axisZ = 0.0f;
        float halfU = 0.0f;
        float halfV = 0.0f;
    };

    // y = level + slopeX (x - refX) + slopeZ (z - refZ).
    struct WaterPlane
    {
        bool valid = false;
        float level = 0.0f;
        float slopeX = 0.0f;
        float slopeZ = 0.0f;
        float refX = 0.0f;
        float refZ = 0.0f;
        Vec3 velocity;
        int samples = 0;
    };

    // floor + count (2r)^3 / cell^2.
    float ColumnHeight(const ColumnField& field, int i, int k);
    // Bilinear; false outside.
    bool SampleColumns(const ColumnField& field, float x, float z, float& height, Vec3& velocity);
    // xz footprint of an oriented box.
    Footprint BoxFootprint(const Vec3& center, const Vec3& axisX, const Vec3& axisY, const Vec3& axisZ, const Vec3& half);
    bool InsideFootprint(const Footprint& f, float x, float z, float margin);
    // Least-squares plane through 16 samples just outside the footprint, skipping other planks.
    WaterPlane FitWaterPlane(const ColumnField& field,
                             const std::vector<Footprint>& footprints,
                             int body,
                             float margin,
                             float fallbackLevel);
}
