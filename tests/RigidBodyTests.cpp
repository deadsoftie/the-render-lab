#include "PhysicsTests.h"
#include "TestFramework.h"
#include "physics/RigidBody.h"

using namespace Phys;
using Anim::Cross;
using Anim::Dot;
using Anim::FromAxisAngle;
using Anim::Normalize;

namespace
{
    bool NearVec3(const Vec3& a, const Vec3& b, float eps)
    {
        return std::abs(a.x - b.x) <= eps && std::abs(a.y - b.y) <= eps && std::abs(a.z - b.z) <= eps;
    }

    bool NearMat3(const Mat3& a, const Mat3& b, float eps)
    {
        for (int i = 0; i < 9; ++i)
            if (std::abs(a.m[i] - b.m[i]) > eps)
                return false;
        return true;
    }

    Mat3 RotationFromAxisAngle(const Vec3& axis, float angle)
    {
        Anim::Quat q = FromAxisAngle(Normalize(axis), angle);
        return Anim::FromColumns(Anim::Rotate(q, Vec3{1, 0, 0}), Anim::Rotate(q, Vec3{0, 1, 0}), Anim::Rotate(q, Vec3{0, 0, 1}));
    }

    RigidBodyProperties PlankProperties(const Vec3& center)
    {
        return MakeProperties(ComputeMassProperties(BoxMassPoints(center, Vec3{0.3f, 0.025f, 0.125f}, 2.0f, 16)));
    }

    void TestBoxInertia()
    {
        Vec3 half{0.3f, 0.025f, 0.125f};
        float mass = 2.0f;
        MassProperties mp = ComputeMassProperties(BoxMassPoints(Vec3{0.1f, -0.2f, 0.3f}, half, mass, 16));
        CHECK_NEAR(mp.mass, mass, 1e-4f);
        CHECK(NearVec3(mp.centerOfMass, Vec3{0.1f, -0.2f, 0.3f}, 1e-5f));

        float a = 2.0f * half.x;
        float b = 2.0f * half.y;
        float c = 2.0f * half.z;
        float ixx = mass / 12.0f * (b * b + c * c);
        float iyy = mass / 12.0f * (a * a + c * c);
        float izz = mass / 12.0f * (a * a + b * b);
        CHECK_NEAR(Anim::At(mp.inertia, 0, 0), ixx, ixx * 0.01f);
        CHECK_NEAR(Anim::At(mp.inertia, 1, 1), iyy, iyy * 0.01f);
        CHECK_NEAR(Anim::At(mp.inertia, 2, 2), izz, izz * 0.01f);
        CHECK_NEAR(Anim::At(mp.inertia, 0, 1), 0.0f, 1e-6f);
        CHECK_NEAR(Anim::At(mp.inertia, 0, 2), 0.0f, 1e-6f);
        CHECK_NEAR(Anim::At(mp.inertia, 1, 2), 0.0f, 1e-6f);
    }

    void TestPointMassInertia()
    {
        std::vector<MassPoint> points = {{Vec3{1, 0, 0}, 1.0f}, {Vec3{-1, 0, 0}, 3.0f}};
        MassProperties mp = ComputeMassProperties(points);
        CHECK_NEAR(mp.mass, 4.0f, 1e-6f);
        CHECK(NearVec3(mp.centerOfMass, Vec3{-0.5f, 0, 0}, 1e-6f));
        CHECK_NEAR(Anim::At(mp.inertia, 0, 0), 0.0f, 1e-6f);
        CHECK_NEAR(Anim::At(mp.inertia, 1, 1), 3.0f, 1e-5f);
        CHECK_NEAR(Anim::At(mp.inertia, 2, 2), 3.0f, 1e-5f);

        std::vector<MassPoint> diagonal = {{Vec3{1, 1, 0}, 1.0f}, {Vec3{-1, -1, 0}, 1.0f}};
        MassProperties md = ComputeMassProperties(diagonal);
        CHECK_NEAR(Anim::At(md.inertia, 0, 1), -2.0f, 1e-5f);
        CHECK_NEAR(Anim::At(md.inertia, 1, 0), -2.0f, 1e-5f);
    }

    void TestKinematics()
    {
        RigidBodyProperties props = PlankProperties(Vec3{0.3f, 0.0f, 0.0f});
        Mat3 R0 = RotationFromAxisAngle(Vec3{1, 2, -1}, 0.8f);
        Vec3 x0{1, 2, 3};
        Vec3 v0{0.5f, -1.0f, 2.0f};
        Vec3 w0{0.3f, 1.5f, -0.7f};
        RigidBodyState s = MakeState(props, x0, R0, v0, w0);
        Kinematics kin = ComputeKinematics(props, s);

        CHECK(NearVec3(s.c, R0 * Vec3{0.3f, 0, 0} + x0, 1e-5f));
        CHECK(NearVec3(kin.x, x0, 1e-5f));
        CHECK(NearVec3(kin.cDot, v0, 1e-5f));
        CHECK(NearVec3(kin.omega, w0, 1e-4f));

        Mat3 iWorld = s.R * props.iObj * Anim::Transpose(s.R);
        CHECK(NearMat3(kin.iInv * iWorld, Mat3{}, 1e-4f));
        CHECK(NearMat3(kin.RDot, Anim::Skew(w0) * s.R, 1e-4f));

        Vec3 qLocal{0.6f, 0.025f, -0.125f};
        Vec3 p = WorldPoint(kin, s, qLocal);
        CHECK(NearVec3(p, s.R * qLocal + x0, 1e-5f));
        CHECK(NearVec3(PointVelocity(kin, s, p), kin.RDot * qLocal + kin.xDot, 1e-4f));
        CHECK(NearVec3(PointVelocity(kin, s, s.c), kin.cDot, 1e-6f));

        float linear = LinearKineticEnergy(props, s);
        CHECK_NEAR(linear, 0.5f * props.mass * Dot(v0, v0), 1e-4f);
        float angular = AngularKineticEnergy(kin, s);
        CHECK_NEAR(angular, 0.5f * Dot(w0, iWorld * w0), 1e-4f);
    }

    void TestForces()
    {
        Vec3 c{1, 2, 3};
        ForceAccumulator acc;
        acc.AddForceAtPoint(Vec3{0, -9.81f, 0}, c, c);
        CHECK(NearVec3(acc.force, Vec3{0, -9.81f, 0}, 1e-6f));
        CHECK(NearVec3(acc.torque, Vec3{}, 1e-6f));

        Vec3 r{0.5f, 0.0f, 0.2f};
        Vec3 f{0.0f, 3.0f, -1.0f};
        acc.AddForceAtPoint(f, c + r, c);
        CHECK(NearVec3(acc.force, Vec3{0, -6.81f, -1.0f}, 1e-5f));
        CHECK(NearVec3(acc.torque, Cross(r, f), 1e-6f));

        acc.AddForceAtPoint(-f, c - r, c);
        CHECK(NearVec3(acc.torque, Cross(r, f) * 2.0f, 1e-5f));

        acc.Clear();
        CHECK(NearVec3(acc.force, Vec3{}, 0.0f));
        CHECK(NearVec3(acc.torque, Vec3{}, 0.0f));
    }

    void TestTorqueFreeSpin()
    {
        RigidBodyProperties props = PlankProperties(Vec3{});
        RigidBodyState s = MakeState(props, Vec3{}, Mat3{}, Vec3{}, Vec3{0.4f, 3.0f, 0.2f});
        Vec3 L0 = s.L;
        Vec3 c0 = s.c;
        ForceAccumulator none;
        for (int i = 0; i < 10000; ++i)
        {
            Kinematics kin = ComputeKinematics(props, s);
            s = Advance(s, Derivative(kin, none), 1.0f / 240.0f);
            Renormalize(s);
        }
        CHECK(NearVec3(s.L, L0, 0.0f));
        CHECK(NearMat3(Anim::Transpose(s.R) * s.R, Mat3{}, 1e-5f));
        CHECK_NEAR(Anim::Determinant(s.R), 1.0f, 1e-5f);
        CHECK(NearVec3(s.c, c0, 0.0f));
        CHECK(NearVec3(s.c, Vec3{}, 1e-9f));
    }

    void TestGravityMomentum()
    {
        RigidBodyProperties props = PlankProperties(Vec3{});
        RigidBodyState s = MakeState(props, Vec3{0, 5, 0}, Mat3{}, Vec3{}, Vec3{});
        Vec3 g{0, -9.81f, 0};
        float dt = 1.0f / 240.0f;
        for (int i = 0; i < 240; ++i)
        {
            Kinematics kin = ComputeKinematics(props, s);
            ForceAccumulator acc;
            acc.AddForceAtPoint(g * props.mass, s.c, s.c);
            s = Advance(s, Derivative(kin, acc), dt);
        }
        CHECK(NearVec3(s.P, g * props.mass, 1e-3f));
        CHECK(NearVec3(s.L, Vec3{}, 0.0f));
        CHECK(NearMat3(s.R, Mat3{}, 0.0f));
    }

    void TestOffCenterForceSpins()
    {
        RigidBodyProperties props = PlankProperties(Vec3{});
        RigidBodyState s = MakeState(props, Vec3{}, Mat3{}, Vec3{}, Vec3{});
        Kinematics kin = ComputeKinematics(props, s);
        ForceAccumulator acc;
        acc.AddForceAtPoint(Vec3{0, 1, 0}, s.c + Vec3{0.3f, 0, 0}, s.c);
        s = Advance(s, Derivative(kin, acc), 0.01f);
        kin = ComputeKinematics(props, s);
        CHECK(kin.omega.z > 0.0f);
        CHECK_NEAR(kin.omega.x, 0.0f, 1e-6f);
        CHECK_NEAR(kin.omega.y, 0.0f, 1e-6f);
        CHECK_NEAR(kin.omega.z, 0.01f * 0.3f / Anim::At(props.iObj, 2, 2), 1e-4f);
    }
}

void RunRigidBodyTests()
{
    TestBoxInertia();
    TestPointMassInertia();
    TestKinematics();
    TestForces();
    TestTorqueFreeSpin();
    TestGravityMomentum();
    TestOffCenterForceSpins();
}
