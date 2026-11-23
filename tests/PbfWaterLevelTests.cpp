#include <cmath>
#include <cstdio>
#include <vector>

#include "PhysicsTests.h"
#include "TestFramework.h"
#include "pbf/PbfWaterLevel.h"
#include "physics/PhysicsWorld.h"

using namespace Pbf;

namespace
{
    constexpr int kNx = 40;
    constexpr int kNz = 20;
    constexpr float kCell = 0.1f;
    constexpr float kVolume = 0.001f;

    std::vector<float> Field(float baseHeight, float slopeX, float slopeZ, Vec3 velocity)
    {
        std::vector<float> data(static_cast<size_t>(kNx) * kNz * 4);
        for (int k = 0; k < kNz; ++k)
            for (int i = 0; i < kNx; ++i)
            {
                float x = (i + 0.5f) * kCell;
                float z = (k + 0.5f) * kCell;
                float h = baseHeight + slopeX * (x - 2.0f) + slopeZ * (z - 1.0f);
                float* c = &data[(static_cast<size_t>(k) * kNx + i) * 4];
                c[0] = velocity.x;
                c[1] = velocity.y;
                c[2] = velocity.z;
                c[3] = h * kCell * kCell / kVolume;
            }
        return data;
    }

    ColumnField MakeField(const std::vector<float>& data)
    {
        ColumnField f;
        f.nx = kNx;
        f.nz = kNz;
        f.cell = kCell;
        f.particleVolume = kVolume;
        f.data = data.data();
        return f;
    }

    void TestSampling()
    {
        std::vector<float> data = Field(0.4f, 0.0f, 0.0f, Vec3{0.5f, 0.0f, -0.25f});
        ColumnField field = MakeField(data);
        float h = 0.0f;
        Vec3 v;
        CHECK(SampleColumns(field, 1.23f, 0.77f, h, v));
        CHECK_NEAR(h, 0.4f, 1e-5f);
        CHECK_NEAR(v.x, 0.5f, 1e-6f);
        CHECK_NEAR(v.z, -0.25f, 1e-6f);
        CHECK(!SampleColumns(field, -0.1f, 0.5f, h, v));
        CHECK(!SampleColumns(field, 1.0f, 2.5f, h, v));

        std::vector<float> tilted = Field(0.4f, 0.1f, 0.0f, Vec3{});
        ColumnField tiltedField = MakeField(tilted);
        CHECK(SampleColumns(tiltedField, 2.37f, 1.0f, h, v));
        CHECK_NEAR(h, 0.4f + 0.1f * 0.37f, 1e-5f);
    }

    void TestFootprint()
    {
        Vec3 half{0.2f, 0.05f, 0.3f};
        Footprint f = BoxFootprint(Vec3{2.0f, 0.4f, 1.0f}, Vec3{1, 0, 0}, Vec3{0, 1, 0}, Vec3{0, 0, 1}, half);
        CHECK_NEAR(f.halfU, 0.2f, 1e-6f);
        CHECK_NEAR(f.halfV, 0.3f, 1e-6f);
        CHECK(InsideFootprint(f, 2.15f, 1.25f, 0.0f));
        CHECK(!InsideFootprint(f, 2.25f, 1.0f, 0.0f));
        CHECK(InsideFootprint(f, 2.25f, 1.0f, 0.1f));

        float c = std::cos(0.5f);
        float s = std::sin(0.5f);
        Footprint rolled = BoxFootprint(Vec3{2.0f, 0.4f, 1.0f}, Vec3{1, 0, 0}, Vec3{0, c, s}, Vec3{0, -s, c}, half);
        CHECK_NEAR(rolled.halfV, 0.05f * s + 0.3f * c, 1e-5f);
    }

    void TestPlaneFit()
    {
        std::vector<float> data = Field(0.42f, 0.08f, -0.05f, Vec3{0.3f, 0.0f, 0.1f});
        ColumnField field = MakeField(data);
        Vec3 half{0.2f, 0.05f, 0.3f};
        std::vector<Footprint> footprints = {
            BoxFootprint(Vec3{2.0f, 0.4f, 1.0f}, Vec3{1, 0, 0}, Vec3{0, 1, 0}, Vec3{0, 0, 1}, half),
            BoxFootprint(Vec3{2.55f, 0.4f, 1.0f}, Vec3{1, 0, 0}, Vec3{0, 1, 0}, Vec3{0, 0, 1}, half)};
        WaterPlane plane = FitWaterPlane(field, footprints, 0, 0.15f, 0.0f);
        std::printf("[PBF] water plane: level %.4f slope (%.4f, %.4f) from %d samples\n",
                    plane.level,
                    plane.slopeX,
                    plane.slopeZ,
                    plane.samples);
        CHECK(plane.valid);
        CHECK(plane.samples >= 6);
        CHECK(plane.samples < 16);
        CHECK_NEAR(plane.level, 0.42f, 2e-3f);
        CHECK_NEAR(plane.slopeX, 0.08f, 5e-3f);
        CHECK_NEAR(plane.slopeZ, -0.05f, 5e-3f);
        CHECK_NEAR(plane.velocity.x, 0.3f, 1e-5f);

        std::vector<Footprint> outside = {
            BoxFootprint(Vec3{20.0f, 0.4f, 20.0f}, Vec3{1, 0, 0}, Vec3{0, 1, 0}, Vec3{0, 0, 1}, half)};
        WaterPlane fallback = FitWaterPlane(field, outside, 0, 0.15f, 0.33f);
        CHECK(!fallback.valid);
        CHECK_NEAR(fallback.level, 0.33f, 1e-6f);
    }

    void TestLocalSurfaceBuoyancy()
    {
        Phys::PhysicsWorld world;
        world.settings.integrator = Phys::IntegratorType::RK2;
        world.buoyancy.enabled = true;
        world.buoyancy.level = 0.0f;
        Phys::WaterSurface surface;
        surface.valid = true;
        surface.level = 0.3f;
        surface.reference = Vec3{0.0f, 0.3f, 0.0f};
        world.buoyancy.local.push_back(surface);
        Phys::BodyDesc desc;
        desc.halfExtents = Vec3{0.2f, 0.05f, 0.15f};
        desc.mass = 0.5f * 1000.0f * 8.0f * 0.2f * 0.05f * 0.15f;
        desc.position = Vec3{0.0f, 0.4f, 0.0f};
        world.AddBody(desc);
        world.CaptureInitialState();
        for (int i = 0; i < 1200; ++i)
            world.Advance(1.0f / 60.0f);
        CHECK_NEAR(world.State(0).c.y, 0.3f, 3e-3f);

        world.buoyancy.local[0].slopeX = 0.2f;
        Phys::ForceAccumulator tilted = world.FluidForce(0);
        std::printf("[Physics] local water: c.y %.4f, torque z on a +x slope %.3f N m\n",
                    world.State(0).c.y,
                    tilted.torque.z);
        CHECK(tilted.torque.z > 0.0f);

        world.buoyancy.local[0].slopeX = 0.0f;
        world.buoyancy.local[0].velocity = Vec3{1.0f, 0.0f, 0.0f};
        CHECK(world.FluidForce(0).force.x > 0.0f);
    }
}

void RunPbfWaterLevelTests()
{
    TestSampling();
    TestFootprint();
    TestPlaneFit();
    TestLocalSurfaceBuoyancy();
}
