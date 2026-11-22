#include "physics/RigidBody.h"

namespace Phys
{
    MassProperties ComputeMassProperties(const std::vector<MassPoint>& points)
    {
        MassProperties out;
        Vec3 weighted;
        for (const MassPoint& p : points)
        {
            out.mass += p.mass;
            weighted = weighted + p.position * p.mass;
        }
        if (out.mass <= 0.0f)
            return out;

        out.centerOfMass = weighted * (1.0f / out.mass);

        Mat3 inertia = Mat3{} * 0.0f;
        for (const MassPoint& p : points)
        {
            Vec3 r = p.position - out.centerOfMass;
            inertia = inertia + (Mat3{} * Anim::Dot(r, r) - Anim::Outer(r, r)) * p.mass;
        }
        out.inertia = inertia;
        return out;
    }

    std::vector<MassPoint> BoxMassPoints(const Vec3& center, const Vec3& halfExtents, float mass, int pointsPerAxis)
    {
        int n = pointsPerAxis < 1 ? 1 : pointsPerAxis;
        std::vector<MassPoint> points;
        points.reserve(static_cast<size_t>(n) * n * n);
        float pointMass = mass / static_cast<float>(n * n * n);
        for (int k = 0; k < n; ++k)
        {
            for (int j = 0; j < n; ++j)
            {
                for (int i = 0; i < n; ++i)
                {
                    Vec3 t{(i + 0.5f) / n * 2.0f - 1.0f, (j + 0.5f) / n * 2.0f - 1.0f, (k + 0.5f) / n * 2.0f - 1.0f};
                    Vec3 p{center.x + t.x * halfExtents.x, center.y + t.y * halfExtents.y, center.z + t.z * halfExtents.z};
                    points.push_back({p, pointMass});
                }
            }
        }
        return points;
    }

    RigidBodyProperties MakeProperties(const MassProperties& massProperties)
    {
        RigidBodyProperties props;
        props.mass = massProperties.mass;
        props.invMass = massProperties.mass > 0.0f ? 1.0f / massProperties.mass : 0.0f;
        props.cObj = massProperties.centerOfMass;
        props.iObj = massProperties.inertia;
        props.iObjInv = Anim::Inverse(massProperties.inertia);
        return props;
    }

    void ForceAccumulator::AddForce(const Vec3& f)
    {
        force = force + f;
    }

    void ForceAccumulator::AddForceAtPoint(const Vec3& f, const Vec3& point, const Vec3& c)
    {
        force = force + f;
        torque = torque + Anim::Cross(point - c, f);
    }

    void ForceAccumulator::AddTorque(const Vec3& t)
    {
        torque = torque + t;
    }

    void ForceAccumulator::Clear()
    {
        force = Vec3{};
        torque = Vec3{};
    }

    RigidBodyState MakeState(const RigidBodyProperties& props,
                             const Vec3& x0,
                             const Mat3& R0,
                             const Vec3& linearVelocity,
                             const Vec3& angularVelocity)
    {
        RigidBodyState state;
        state.R = Anim::Orthonormalize(R0);
        state.c = state.R * props.cObj + x0;
        state.P = linearVelocity * props.mass;
        Mat3 iWorld = state.R * props.iObj * Anim::Transpose(state.R);
        state.L = iWorld * angularVelocity;
        return state;
    }

    Kinematics ComputeKinematics(const RigidBodyProperties& props, const RigidBodyState& state)
    {
        Kinematics kin;
        kin.cDot = state.P * props.invMass;
        kin.iInv = state.R * props.iObjInv * Anim::Transpose(state.R);
        kin.omega = kin.iInv * state.L;
        kin.RDot = Anim::Skew(kin.omega) * state.R;
        kin.x = state.c - state.R * props.cObj;
        kin.xDot = kin.cDot - kin.RDot * props.cObj;
        return kin;
    }

    RigidBodyDerivative Derivative(const Kinematics& kin, const ForceAccumulator& forces)
    {
        RigidBodyDerivative d;
        d.cDot = kin.cDot;
        d.RDot = kin.RDot;
        d.PDot = forces.force;
        d.LDot = forces.torque;
        return d;
    }

    RigidBodyState Advance(const RigidBodyState& state, const RigidBodyDerivative& d, float dt)
    {
        RigidBodyState next;
        next.c = state.c + d.cDot * dt;
        next.R = state.R + d.RDot * dt;
        next.P = state.P + d.PDot * dt;
        next.L = state.L + d.LDot * dt;
        return next;
    }

    void Renormalize(RigidBodyState& state)
    {
        state.R = Anim::Orthonormalize(state.R);
    }

    Vec3 WorldPoint(const Kinematics& kin, const RigidBodyState& state, const Vec3& qLocal)
    {
        return state.R * qLocal + kin.x;
    }

    Vec3 PointVelocity(const Kinematics& kin, const RigidBodyState& state, const Vec3& worldPoint)
    {
        return kin.cDot + Anim::Cross(kin.omega, worldPoint - state.c);
    }

    float LinearKineticEnergy(const RigidBodyProperties& props, const RigidBodyState& state)
    {
        return 0.5f * Anim::Dot(state.P, state.P) * props.invMass;
    }

    float AngularKineticEnergy(const Kinematics& kin, const RigidBodyState& state)
    {
        return 0.5f * Anim::Dot(kin.omega, state.L);
    }

    void RigidBody::WorldPoints(std::vector<Vec3>& out) const
    {
        Kinematics kin = Evaluate();
        out.resize(localPoints.size());
        for (size_t i = 0; i < localPoints.size(); ++i)
            out[i] = WorldPoint(kin, state, localPoints[i]);
    }
}
