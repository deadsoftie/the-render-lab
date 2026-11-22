#include <cmath>
#include <cstdio>

#include "PhysicsTests.h"
#include "TestFramework.h"
#include "physics/PhysicsWorld.h"

using namespace Phys;

namespace
{
    bool AllFinite(const PhysicsWorld& world)
    {
        for (int i = 0; i < world.BodyCount(); ++i)
        {
            const RigidBodyState& s = world.State(i);
            if (!Anim::IsFinite(s.c) || !Anim::IsFinite(s.P) || !Anim::IsFinite(s.L))
                return false;
            for (float v : s.R.m)
                if (!std::isfinite(v))
                    return false;
        }
        return true;
    }

    bool MakeChain(PhysicsWorld& world, IntegratorType type)
    {
        world.Clear();
        world.settings.integrator = type;
        world.settings.linearDrag = 0.1f;
        world.settings.angularDrag = 0.1f;
        std::string error;
        bool ok = BuildChain(ChainDesc{}, world, error);
        world.CaptureInitialState();
        return ok;
    }

    void RunSeconds(PhysicsWorld& world, float seconds)
    {
        int frames = static_cast<int>(seconds * 60.0f);
        for (int i = 0; i < frames; ++i)
            world.Advance(1.0f / 60.0f);
    }

    void TestChainLayout()
    {
        PhysicsWorld world;
        CHECK(MakeChain(world, IntegratorType::RK2));
        CHECK(world.BodyCount() == 6);
        CHECK(world.AnchorCount() == 2);
        CHECK(world.SpringCount() == 2 + 5 * 4 + 2);

        ChainDesc noDiagonals;
        noDiagonals.diagonals = false;
        PhysicsWorld plain;
        std::string error;
        CHECK(BuildChain(noDiagonals, plain, error));
        CHECK(plain.SpringCount() == 2 + 5 * 2 + 2);

        ChainDesc tooShort;
        tooShort.anchorB = Vec3{0.5f, 2.0f, 0.0f};
        PhysicsWorld bad;
        CHECK(!BuildChain(tooShort, bad, error));
        ChainDesc none;
        none.plankCount = 0;
        CHECK(!BuildChain(none, bad, error));

        for (int i = 0; i < world.BodyCount(); ++i)
        {
            CHECK_NEAR(world.State(i).c.y, 2.0f, 1e-5f);
            CHECK(world.State(i).c.x > -1.5f && world.State(i).c.x < 1.5f);
        }
        CHECK_NEAR(world.State(0).c.x, -world.State(5).c.x, 1e-5f);
    }

    void TestChainSettles()
    {
        PhysicsWorld world;
        MakeChain(world, IntegratorType::RK2);
        RunSeconds(world, 60.0f);
        CHECK(AllFinite(world));
        EnergyReport e = world.Energy();
        std::printf("[PhysicsWorld] RK2 chain after 60 s: kinetic=%.2e J, mid plank y=%.4f\n",
                    e.kinetic, world.State(2).c.y);
        CHECK(e.kinetic < 1e-3f);

        int n = world.BodyCount();
        for (int i = 0; i < n / 2; ++i)
        {
            CHECK_NEAR(world.State(i).c.y, world.State(n - 1 - i).c.y, 1e-3f);
            CHECK_NEAR(world.State(i).c.x, -world.State(n - 1 - i).c.x, 1e-3f);
        }
        CHECK(world.State(2).c.y < world.State(0).c.y);
        CHECK(world.State(0).c.y < 2.0f);
        for (int i = 0; i < n; ++i)
            CHECK(std::abs(world.State(i).c.z) < 1e-3f);
    }

    void TestIntegratorsAgreeAtRest()
    {
        const IntegratorType types[] = {IntegratorType::RK2, IntegratorType::Verlet, IntegratorType::RK4};
        float midY[3] = {};
        for (int k = 0; k < 3; ++k)
        {
            PhysicsWorld world;
            MakeChain(world, types[k]);
            RunSeconds(world, 60.0f);
            CHECK(AllFinite(world));
            midY[k] = world.State(2).c.y;
        }
        CHECK_NEAR(midY[0], midY[1], 5e-3f);
        CHECK_NEAR(midY[0], midY[2], 5e-3f);
    }

    void TestAnchorJump()
    {
        PhysicsWorld world;
        MakeChain(world, IntegratorType::RK2);
        RunSeconds(world, 20.0f);
        world.SetAnchorTarget(1, Vec3{2.0f, 2.2f, 0.3f});
        world.Advance(1.0f / 60.0f);
        float peak = 0.0f;
        for (int i = 0; i < 60; ++i)
        {
            world.Advance(1.0f / 60.0f);
            peak = std::fmax(peak, world.Energy().kinetic);
        }
        RunSeconds(world, 30.0f);
        CHECK(AllFinite(world));
        CHECK(peak > 0.0f);
        CHECK(world.Energy().kinetic < peak);
        CHECK(world.GetAnchor(1).target.x == 2.0f);
    }

    void TestPassiveEnergyDecays()
    {
        PhysicsWorld world;
        MakeChain(world, IntegratorType::RK4);
        float previous = world.Energy().Total();
        bool monotone = true;
        for (int second = 0; second < 20; ++second)
        {
            RunSeconds(world, 1.0f);
            float now = world.Energy().Total();
            monotone = monotone && now <= previous + 1e-3f;
            previous = now;
        }
        CHECK(monotone);
    }

    void TestResetAndFixedStep()
    {
        PhysicsWorld world;
        MakeChain(world, IntegratorType::RK2);
        RigidBodyState s0 = world.State(3);
        CHECK(world.Advance(1.0f / 60.0f) == 4);
        CHECK(world.Advance(1.0f / 480.0f) == 0);
        CHECK(world.Advance(1.0f / 480.0f) == 1);
        world.settings.maxStepsPerFrame = 8;
        CHECK(world.Advance(1.0f) == 8);
        world.StepOnce();
        CHECK(world.StepCount() == 4 + 1 + 8 + 1);
        world.Reset();
        CHECK(world.StepCount() == 0);
        CHECK(world.Time() == 0.0f);
        CHECK(world.State(3).c.y == s0.c.y);
        CHECK(world.State(3).P.y == s0.P.y);
    }

    PhysicsWorld SingleBody()
    {
        PhysicsWorld world;
        world.settings.gravity = Vec3{};
        world.settings.linearDrag = 0.0f;
        world.settings.angularDrag = 0.0f;
        world.settings.integrator = IntegratorType::RK4;
        BodyDesc body;
        body.name = "B";
        body.halfExtents = Vec3{0.2f, 0.025f, 0.12f};
        body.position = Vec3{0.0f, 1.0f, 0.0f};
        world.AddBody(body);
        world.CaptureInitialState();
        return world;
    }

    void TestImpulse()
    {
        PhysicsWorld world = SingleBody();
        Vec3 c = world.State(0).c;
        Vec3 J{0.0f, 2.0f, 0.0f};
        Vec3 r{0.2f, 0.0f, 0.0f};
        world.ApplyImpulse(0, J, c + r);
        CHECK_NEAR(world.State(0).P.y, 2.0f, 1e-6f);
        Vec3 expectedL = Anim::Cross(r, J);
        CHECK_NEAR(world.State(0).L.z, expectedL.z, 1e-6f);
        CHECK_NEAR(world.State(0).L.x, 0.0f, 1e-6f);
        world.ApplyImpulse(5, J, c);
        CHECK_NEAR(world.State(0).P.y, 2.0f, 1e-6f);
    }

    void TestMouseSpringPulls()
    {
        PhysicsWorld world = SingleBody();
        world.mouse.active = true;
        world.mouse.body = 0;
        world.mouse.localPoint = Vec3{0.2f, 0.0f, 0.0f};
        world.mouse.target = Vec3{0.5f, 1.5f, 0.3f};
        RunSeconds(world, 5.0f);
        Kinematics kin = world.BodyKinematics(0);
        Vec3 p = WorldPoint(kin, world.State(0), world.mouse.localPoint);
        CHECK(Anim::Length(p - world.mouse.target) < 0.02f);
        CHECK(AllFinite(world));

        world.mouse.active = false;
        Vec3 P = world.State(0).P;
        RunSeconds(world, 1.0f);
        CHECK(Anim::Length(world.State(0).P - P) < 1e-5f);
    }

    void TestSpringForceQuery()
    {
        PhysicsWorld world;
        MakeChain(world, IntegratorType::RK2);
        for (int i = 0; i < world.SpringCount(); ++i)
        {
            const Spring& s = world.GetSpring(i);
            Vec3 d = world.EndPositionNow(s.a) - world.EndPositionNow(s.b);
            float stretch = Anim::Length(d) - s.restLength;
            Vec3 f = world.SpringForceOnA(i);
            CHECK_NEAR(Anim::Length(f), std::abs(s.stiffness * stretch), 1e-3f);
            CHECK(stretch <= 0.0f || Anim::Dot(f, d) < 0.0f);
        }
    }
    void TestExternalForce()
    {
        PhysicsWorld world;
        world.settings.gravity = Vec3{};
        world.settings.linearDrag = 0.0f;
        world.settings.angularDrag = 0.0f;
        world.settings.integrator = IntegratorType::RK4;
        BodyDesc desc;
        desc.mass = 2.0f;
        world.AddBody(desc);
        world.CaptureInitialState();
        world.SetExternalForce(0, Vec3{3.0f, 0.0f, 0.0f}, Vec3{0.0f, 0.0f, 0.5f});
        CHECK_NEAR(world.FluidForce(0).force.x, 3.0f, 1e-6f);
        CHECK_NEAR(world.FluidForce(0).torque.z, 0.5f, 1e-6f);
        RunSeconds(world, 1.0f);
        CHECK_NEAR(world.State(0).P.x, 3.0f, 1e-3f);
        CHECK_NEAR(world.State(0).L.z, 0.5f, 1e-3f);
        CHECK_NEAR(world.State(0).P.y, 0.0f, 1e-6f);
        world.ClearExternalForces();
        CHECK(Anim::Length(world.FluidForce(0).force) == 0.0f);
        world.SetExternalForce(5, Vec3{1.0f, 0.0f, 0.0f}, Vec3{});
        CHECK(Anim::Length(world.FluidForce(5).force) == 0.0f);
    }

    void TestBuoyancyFloats()
    {
        const float level = 0.3f;
        const float ratios[] = {0.25f, 0.5f, 0.75f};
        for (float ratio : ratios)
        {
            PhysicsWorld world;
            world.settings.integrator = IntegratorType::RK2;
            world.buoyancy.enabled = true;
            world.buoyancy.level = level;
            world.buoyancy.drag = 2.0f;
            BodyDesc desc;
            desc.halfExtents = Vec3{0.2f, 0.05f, 0.15f};
            float volume = 8.0f * desc.halfExtents.x * desc.halfExtents.y * desc.halfExtents.z;
            desc.mass = ratio * world.buoyancy.density * volume;
            desc.position = Vec3{0.0f, 0.5f, 0.0f};
            desc.rotation = Anim::RotationFromVector(Vec3{0.0f, 0.0f, 0.26f});
            world.AddBody(desc);
            world.CaptureInitialState();
            RunSeconds(world, 20.0f);

            float expected = level - 2.0f * desc.halfExtents.y * ratio + desc.halfExtents.y;
            float upright = Anim::Column(world.State(0).R, 1).y;
            std::printf("[Physics] buoyancy density ratio %.2f: c.y %.4f (expected %.4f), up.y %.4f, lift %.2f N vs weight %.2f N\n",
                        ratio,
                        world.State(0).c.y,
                        expected,
                        upright,
                        world.FluidForce(0).force.y,
                        desc.mass * 9.81f);
            CHECK(AllFinite(world));
            CHECK_NEAR(world.State(0).c.y, expected, 3e-3f);
            CHECK(upright > 0.99f);
            CHECK_NEAR(world.FluidForce(0).force.y, desc.mass * 9.81f, 0.02f * desc.mass * 9.81f);
        }
    }
}

void RunPhysicsWorldTests()
{
    TestChainLayout();
    TestChainSettles();
    TestIntegratorsAgreeAtRest();
    TestAnchorJump();
    TestPassiveEnergyDecays();
    TestResetAndFixedStep();
    TestImpulse();
    TestMouseSpringPulls();
    TestSpringForceQuery();
    TestExternalForce();
    TestBuoyancyFloats();
}
