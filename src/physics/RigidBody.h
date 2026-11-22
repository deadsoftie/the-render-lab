#pragma once
#include <vector>

#include "anim/Math/Mat3.h"
#include "anim/Math/Vec3.h"

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

    MassProperties ComputeMassProperties(const std::vector<MassPoint>& points);
    std::vector<MassPoint> BoxMassPoints(const Vec3& center, const Vec3& halfExtents, float mass, int pointsPerAxis);

    struct RigidBodyProperties
    {
        float mass = 1.0f;
        float invMass = 1.0f;
        Vec3 cObj;
        Mat3 iObj;
        Mat3 iObjInv;
    };

    RigidBodyProperties MakeProperties(const MassProperties& massProperties);

    struct RigidBodyState
    {
        Vec3 c;
        Mat3 R;
        Vec3 P;
        Vec3 L;
    };

    struct RigidBodyDerivative
    {
        Vec3 cDot;
        Mat3 RDot = Mat3{} * 0.0f;
        Vec3 PDot;
        Vec3 LDot;
    };

    struct Kinematics
    {
        Vec3 cDot;
        Mat3 iInv;
        Vec3 omega;
        Mat3 RDot;
        Vec3 x;
        Vec3 xDot;
    };

    struct ForceAccumulator
    {
        Vec3 force;
        Vec3 torque;

        void AddForce(const Vec3& f);
        void AddForceAtPoint(const Vec3& f, const Vec3& point, const Vec3& c);
        void AddTorque(const Vec3& t);
        void Clear();
    };

    RigidBodyState MakeState(const RigidBodyProperties& props,
                             const Vec3& x0,
                             const Mat3& R0,
                             const Vec3& linearVelocity,
                             const Vec3& angularVelocity);

    Kinematics ComputeKinematics(const RigidBodyProperties& props, const RigidBodyState& state);
    RigidBodyDerivative Derivative(const Kinematics& kin, const ForceAccumulator& forces);

    RigidBodyState Advance(const RigidBodyState& state, const RigidBodyDerivative& d, float dt);
    void Renormalize(RigidBodyState& state);

    Vec3 WorldPoint(const Kinematics& kin, const RigidBodyState& state, const Vec3& qLocal);
    Vec3 PointVelocity(const Kinematics& kin, const RigidBodyState& state, const Vec3& worldPoint);

    float LinearKineticEnergy(const RigidBodyProperties& props, const RigidBodyState& state);
    float AngularKineticEnergy(const Kinematics& kin, const RigidBodyState& state);

    struct RigidBody
    {
        RigidBodyProperties props;
        RigidBodyState state;
        std::vector<Vec3> localPoints;

        Kinematics Evaluate() const { return ComputeKinematics(props, state); }
        void WorldPoints(std::vector<Vec3>& out) const;
    };
}
