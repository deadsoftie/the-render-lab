#pragma once
#include <vector>

#include "anim/Math/Mat3.h"
#include "anim/Math/Vec3.h"

// Rigid body from the course flowchart: state (c, R, P, L), the rest derived.
namespace Phys
{
    using Anim::Mat3;
    using Anim::Vec3;

    struct MassPoint
    {
        Vec3 position;
        float mass = 0.0f;
    };

    struct MassProperties
    {
        float mass = 0.0f;
        Vec3 centerOfMass;
        Mat3 inertia;
    };

    // M, c_obj and I = sum m_i ((r_i . r_i) 1 - r_i r_i^T).
    MassProperties ComputeMassProperties(const std::vector<MassPoint>& points);

    // n^3 equal point masses filling a box.
    std::vector<MassPoint> BoxMassPoints(const Vec3& center, const Vec3& halfExtents, float mass, int pointsPerAxis);

    // Body-space constants.
    struct RigidBodyProperties
    {
        float mass = 1.0f;
        float invMass = 1.0f;
        Vec3 cObj;
        Mat3 iObj;
        Mat3 iObjInv;
    };

    RigidBodyProperties MakeProperties(const MassProperties& massProperties);

    // Integrated state.
    struct RigidBodyState
    {
        Vec3 c;
        Mat3 R;
        Vec3 P;
        Vec3 L;
    };

    // (c_dot, R_dot, F, tau).
    struct RigidBodyDerivative
    {
        Vec3 cDot;
        Mat3 RDot = Mat3{} * 0.0f;
        Vec3 PDot;
        Vec3 LDot;
    };

    // Flowchart derived quantities: c_dot, I^-1, omega, R_dot, x, x_dot.
    struct Kinematics
    {
        Vec3 cDot;
        Mat3 iInv;
        Vec3 omega;
        Mat3 RDot;
        Vec3 x;
        Vec3 xDot;
    };

    // Force and torque about c.
    struct ForceAccumulator
    {
        Vec3 force;
        Vec3 torque;

        void AddForce(const Vec3& f);
        // F += f, T += (point - c) x f.
        void AddForceAtPoint(const Vec3& f, const Vec3& point, const Vec3& c);
        void AddTorque(const Vec3& t);
        void Clear();
    };

    // Y(0) from x0, R0, v0, omega0.
    RigidBodyState MakeState(const RigidBodyProperties& props,
                             const Vec3& x0,
                             const Mat3& R0,
                             const Vec3& linearVelocity,
                             const Vec3& angularVelocity);

    Kinematics ComputeKinematics(const RigidBodyProperties& props, const RigidBodyState& state);
    RigidBodyDerivative Derivative(const Kinematics& kin, const ForceAccumulator& forces);

    // y + dt y_dot.
    RigidBodyState Advance(const RigidBodyState& state, const RigidBodyDerivative& d, float dt);

    // Gram-Schmidt on R.
    void Renormalize(RigidBodyState& state);

    // R q + x.
    Vec3 WorldPoint(const Kinematics& kin, const RigidBodyState& state, const Vec3& qLocal);

    // c_dot + omega x (p - c).
    Vec3 PointVelocity(const Kinematics& kin, const RigidBodyState& state, const Vec3& worldPoint);

    float LinearKineticEnergy(const RigidBodyProperties& props, const RigidBodyState& state);
    float AngularKineticEnergy(const Kinematics& kin, const RigidBodyState& state);

    struct RigidBody
    {
        RigidBodyProperties props;
        RigidBodyState state;
        std::vector<Vec3> localPoints;  // for the q_i overlay

        Kinematics Evaluate() const { return ComputeKinematics(props, state); }
        void WorldPoints(std::vector<Vec3>& out) const;
    };
}
