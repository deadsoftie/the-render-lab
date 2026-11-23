#include "physics/PhysicsWorld.h"

#include <cmath>

namespace Phys
{
    namespace
    {
        constexpr int kMassPointsPerAxis = 10;
        constexpr float kMinSpringLength = 1e-6f;
        constexpr int kBuoyancySamples = 4;
    }

    void PhysicsWorld::Clear()
    {
        m_bodies.clear();
        m_props.clear();
        m_anchors.clear();
        m_springs.clear();
        m_state.clear();
        m_initialState.clear();
        m_externalForce.clear();
        m_externalTorque.clear();
        m_initialAnchors.clear();
        m_time = 0.0f;
        m_accumulator = 0.0f;
        m_frameStart = 0.0f;
        m_frameDuration = 0.0f;
        m_stepCount = 0;
        mouse = MouseSpring{};
    }

    int PhysicsWorld::AddBody(const BodyDesc& desc)
    {
        RigidBodyProperties props =
            MakeProperties(ComputeMassProperties(BoxMassPoints(Vec3{}, desc.halfExtents, desc.mass, kMassPointsPerAxis)));
        m_bodies.push_back(desc);
        m_props.push_back(props);
        m_state.push_back(MakeState(props, desc.position, desc.rotation, desc.velocity, desc.angularVelocity));
        m_externalForce.push_back(Vec3{});
        m_externalTorque.push_back(Vec3{});
        return static_cast<int>(m_bodies.size()) - 1;
    }

    int PhysicsWorld::AddAnchor(const std::string& name, const Vec3& position)
    {
        Anchor anchor;
        anchor.name = name;
        anchor.start = position;
        anchor.target = position;
        m_anchors.push_back(anchor);
        return static_cast<int>(m_anchors.size()) - 1;
    }

    int PhysicsWorld::AddSpring(const Spring& spring)
    {
        m_springs.push_back(spring);
        return static_cast<int>(m_springs.size()) - 1;
    }

    float PhysicsWorld::CurrentLength(const SpringEnd& a, const SpringEnd& b) const
    {
        return Anim::Length(EndPosition(a, m_state, m_time) - EndPosition(b, m_state, m_time));
    }

    int PhysicsWorld::FindBody(const std::string& name) const
    {
        for (size_t i = 0; i < m_bodies.size(); ++i)
            if (m_bodies[i].name == name)
                return static_cast<int>(i);
        return -1;
    }

    int PhysicsWorld::FindAnchor(const std::string& name) const
    {
        for (size_t i = 0; i < m_anchors.size(); ++i)
            if (m_anchors[i].name == name)
                return static_cast<int>(i);
        return -1;
    }

    void PhysicsWorld::CaptureInitialState()
    {
        m_initialState = m_state;
        m_initialAnchors.clear();
        for (const Anchor& a : m_anchors)
            m_initialAnchors.push_back(a.target);
    }

    void PhysicsWorld::Reset()
    {
        m_state = m_initialState;
        for (size_t i = 0; i < m_anchors.size() && i < m_initialAnchors.size(); ++i)
        {
            m_anchors[i].start = m_initialAnchors[i];
            m_anchors[i].target = m_initialAnchors[i];
            m_anchors[i].velocity = Vec3{};
        }
        m_time = 0.0f;
        m_accumulator = 0.0f;
        m_frameStart = 0.0f;
        m_frameDuration = 0.0f;
        m_stepCount = 0;
    }

    void PhysicsWorld::SetAnchorTarget(int index, const Vec3& position)
    {
        if (index >= 0 && index < static_cast<int>(m_anchors.size()))
            m_anchors[index].target = position;
    }

    void PhysicsWorld::BeginFrame(float duration)
    {
        m_frameStart = m_time;
        m_frameDuration = duration;
        for (Anchor& a : m_anchors)
            a.velocity = duration > 0.0f ? (a.target - a.start) * (1.0f / duration) : Vec3{};
    }

    void PhysicsWorld::DoStep()
    {
        m_integrator.Step(*this, m_state, m_time, settings.dt, settings.integrator);
        m_time += settings.dt;
        ++m_stepCount;
    }

    int PhysicsWorld::Advance(float frameSeconds)
    {
        if (m_state.empty() || settings.dt <= 0.0f)
            return 0;

        // Tolerance keeps round-off from dropping a step.
        m_accumulator += frameSeconds;
        int steps = static_cast<int>(std::floor(m_accumulator / settings.dt + 1e-3f));
        if (steps > settings.maxStepsPerFrame)
        {
            steps = settings.maxStepsPerFrame;
            m_accumulator = 0.0f;
        }
        else
        {
            m_accumulator -= steps * settings.dt;
        }
        if (steps <= 0)
            return 0;

        // Anchors slide to their targets over this frame.
        BeginFrame(steps * settings.dt);
        for (int i = 0; i < steps; ++i)
            DoStep();
        for (Anchor& a : m_anchors)
            a.start = a.target;
        return steps;
    }

    void PhysicsWorld::StepOnce()
    {
        if (m_state.empty())
            return;
        BeginFrame(settings.dt);
        DoStep();
        for (Anchor& a : m_anchors)
            a.start = a.target;
    }

    void PhysicsWorld::ApplyImpulse(int body, const Vec3& impulse, const Vec3& worldPoint)
    {
        if (body < 0 || body >= static_cast<int>(m_state.size()))
            return;
        RigidBodyState& s = m_state[body];
        s.P = s.P + impulse;
        s.L = s.L + Anim::Cross(worldPoint - s.c, impulse);
    }

    void PhysicsWorld::SetExternalForce(int body, const Vec3& force, const Vec3& torque)
    {
        if (body < 0 || body >= static_cast<int>(m_externalForce.size()))
            return;
        m_externalForce[body] = force;
        m_externalTorque[body] = torque;
    }

    void PhysicsWorld::ClearExternalForces()
    {
        for (size_t i = 0; i < m_externalForce.size(); ++i)
        {
            m_externalForce[i] = Vec3{};
            m_externalTorque[i] = Vec3{};
        }
    }

    void PhysicsWorld::AddBuoyancy(size_t body,
                                   const Kinematics& kin,
                                   const RigidBodyState& state,
                                   ForceAccumulator& forces) const
    {
        // Archimedes per cell; the submerged fraction ramps over the cell height so tilt is smooth.
        const Vec3& h = m_bodies[body].halfExtents;
        const int n = kBuoyancySamples;
        Vec3 cell{2.0f * h.x / n, 2.0f * h.y / n, 2.0f * h.z / n};
        float cellVolume = cell.x * cell.y * cell.z;
        float cellHeight = std::fabs(Anim::Column(state.R, 0).y) * cell.x +
                           std::fabs(Anim::Column(state.R, 1).y) * cell.y +
                           std::fabs(Anim::Column(state.R, 2).y) * cell.z;
        cellHeight = cellHeight > 1e-6f ? cellHeight : 1e-6f;
        WaterSurface surface;
        surface.level = buoyancy.level;
        if (body < buoyancy.local.size() && buoyancy.local[body].valid)
            surface = buoyancy.local[body];
        for (int k = 0; k < n; ++k)
            for (int j = 0; j < n; ++j)
                for (int i = 0; i < n; ++i)
                {
                    Vec3 q{-h.x + (i + 0.5f) * cell.x, -h.y + (j + 0.5f) * cell.y, -h.z + (k + 0.5f) * cell.z};
                    Vec3 p = WorldPoint(kin, state, q);
                    float level = surface.level + surface.slopeX * (p.x - surface.reference.x) +
                                  surface.slopeZ * (p.z - surface.reference.z);
                    float fraction = (level - p.y) / cellHeight + 0.5f;
                    fraction = fraction < 0.0f ? 0.0f : (fraction > 1.0f ? 1.0f : fraction);
                    if (fraction <= 0.0f)
                        continue;
                    float displaced = buoyancy.density * fraction * cellVolume;
                    Vec3 relative = PointVelocity(kin, state, p) - surface.velocity;
                    // Buoyancy plus drag against the water.
                    Vec3 f = settings.gravity * -displaced - relative * (displaced * buoyancy.drag);
                    forces.AddForceAtPoint(f, p, state.c);
                }
    }

    ForceAccumulator PhysicsWorld::FluidForce(int body) const
    {
        ForceAccumulator f;
        f.Clear();
        if (body < 0 || body >= static_cast<int>(m_state.size()))
            return f;
        f.AddForce(m_externalForce[body]);
        f.AddTorque(m_externalTorque[body]);
        if (buoyancy.enabled)
            AddBuoyancy(static_cast<size_t>(body), ComputeKinematics(m_props[body], m_state[body]), m_state[body], f);
        return f;
    }

    Vec3 PhysicsWorld::SpringForceOnA(int index) const
    {
        const Spring& s = m_springs[index];
        Vec3 d = EndPosition(s.a, m_state, m_time) - EndPosition(s.b, m_state, m_time);
        float len = Anim::Length(d);
        if (len < kMinSpringLength)
            return Vec3{};
        Vec3 dir = d * (1.0f / len);
        Vec3 vRel = EndVelocity(s.a, m_state, m_time) - EndVelocity(s.b, m_state, m_time);
        // -(k (|d| - L0) + c (v_rel . d_hat)) d_hat.
        float magnitude = s.stiffness * (len - s.restLength) + s.damping * Anim::Dot(vRel, dir);
        return dir * -magnitude;
    }

    Vec3 PhysicsWorld::AnchorPosition(int index, float t) const
    {
        const Anchor& a = m_anchors[index];
        if (m_frameDuration <= 0.0f)
            return a.target;
        float s = (t - m_frameStart) / m_frameDuration;
        s = s < 0.0f ? 0.0f : (s > 1.0f ? 1.0f : s);
        return a.start + (a.target - a.start) * s;
    }

    Vec3 PhysicsWorld::EndPosition(const SpringEnd& end, const SystemState& y, float t) const
    {
        if (end.body >= 0)
            return WorldPoint(ComputeKinematics(m_props[end.body], y[end.body]), y[end.body], end.localPoint);
        return AnchorPosition(end.anchor, t);
    }

    Vec3 PhysicsWorld::EndVelocity(const SpringEnd& end, const SystemState& y, float t) const
    {
        if (end.body >= 0)
        {
            Kinematics kin = ComputeKinematics(m_props[end.body], y[end.body]);
            return PointVelocity(kin, y[end.body], WorldPoint(kin, y[end.body], end.localPoint));
        }
        (void)t;
        return m_anchors[end.anchor].velocity;
    }

    void PhysicsWorld::Evaluate(const SystemState& y, float t, SystemDerivative& out)
    {
        size_t n = y.size();
        out.resize(n);
        m_kin.resize(n);
        m_forces.resize(n);

        // Kinematics, gravity, drag, fluid.
        for (size_t i = 0; i < n; ++i)
        {
            m_kin[i] = ComputeKinematics(m_props[i], y[i]);
            m_forces[i].Clear();
            m_forces[i].AddForce(settings.gravity * m_props[i].mass);
            m_forces[i].AddForce(y[i].P * -settings.linearDrag);
            m_forces[i].AddTorque(y[i].L * -settings.angularDrag);
            m_forces[i].AddForce(m_externalForce[i]);
            m_forces[i].AddTorque(m_externalTorque[i]);
            if (buoyancy.enabled)
                AddBuoyancy(i, m_kin[i], y[i], m_forces[i]);
        }

        auto endPoint = [&](const SpringEnd& e, Vec3& p, Vec3& v)
        {
            if (e.body >= 0)
            {
                p = WorldPoint(m_kin[e.body], y[e.body], e.localPoint);
                v = PointVelocity(m_kin[e.body], y[e.body], p);
            }
            else
            {
                p = AnchorPosition(e.anchor, t);
                v = m_anchors[e.anchor].velocity;
            }
        };

        // Springs, equal and opposite.
        for (const Spring& s : m_springs)
        {
            Vec3 pa, va, pb, vb;
            endPoint(s.a, pa, va);
            endPoint(s.b, pb, vb);
            Vec3 d = pa - pb;
            float len = Anim::Length(d);
            if (len < kMinSpringLength)
                continue;
            Vec3 dir = d * (1.0f / len);
            float magnitude = s.stiffness * (len - s.restLength) + s.damping * Anim::Dot(va - vb, dir);
            Vec3 f = dir * -magnitude;
            if (s.a.body >= 0)
                m_forces[s.a.body].AddForceAtPoint(f, pa, y[s.a.body].c);
            if (s.b.body >= 0)
                m_forces[s.b.body].AddForceAtPoint(-f, pb, y[s.b.body].c);
        }

        if (mouse.active && mouse.body >= 0 && mouse.body < static_cast<int>(n))
        {
            int b = mouse.body;
            Vec3 p = WorldPoint(m_kin[b], y[b], mouse.localPoint);
            Vec3 v = PointVelocity(m_kin[b], y[b], p);
            Vec3 f = (mouse.target - p) * mouse.stiffness - v * mouse.damping;
            m_forces[b].AddForceAtPoint(f, p, y[b].c);
        }

        for (size_t i = 0; i < n; ++i)
            out[i] = Derivative(m_kin[i], m_forces[i]);
    }

    EnergyReport PhysicsWorld::Energy() const
    {
        EnergyReport e;
        for (size_t i = 0; i < m_state.size(); ++i)
        {
            Kinematics kin = ComputeKinematics(m_props[i], m_state[i]);
            e.kinetic += LinearKineticEnergy(m_props[i], m_state[i]) + AngularKineticEnergy(kin, m_state[i]);
            e.gravitational -= m_props[i].mass * Anim::Dot(settings.gravity, m_state[i].c);
        }
        for (const Spring& s : m_springs)
        {
            float stretch = CurrentLength(s.a, s.b) - s.restLength;
            e.elastic += 0.5f * s.stiffness * stretch * stretch;
        }
        return e;
    }

    Vec3 PhysicsWorld::TotalLinearMomentum() const
    {
        Vec3 p;
        for (const RigidBodyState& s : m_state)
            p = p + s.P;
        return p;
    }

    Vec3 PhysicsWorld::TotalAngularMomentum() const
    {
        Vec3 l;
        for (const RigidBodyState& s : m_state)
            l = l + Anim::Cross(s.c, s.P) + s.L;
        return l;
    }

    Kinematics PhysicsWorld::BodyKinematics(int index) const
    {
        return ComputeKinematics(m_props[index], m_state[index]);
    }

    bool BuildChain(const ChainDesc& desc, PhysicsWorld& world, std::string& error)
    {
        if (desc.plankCount < 1)
        {
            error = "chain needs at least one plank";
            return false;
        }
        Vec3 h = desc.plankHalfExtents;
        if (h.x <= 0.0f || h.y <= 0.0f || h.z <= 0.0f || desc.plankMass <= 0.0f)
        {
            error = "plank half extents and mass must be positive";
            return false;
        }
        if (desc.stiffness <= 0.0f || desc.damping < 0.0f || desc.restLength < 0.0f)
        {
            error = "stiffness must be positive, damping and rest length nonnegative";
            return false;
        }

        Vec3 span = desc.anchorB - desc.anchorA;
        float spanLength = Anim::Length(span);
        float plankTotal = 2.0f * h.x * desc.plankCount;
        if (spanLength <= plankTotal)
        {
            error = "anchors must be further apart than the total plank length";
            return false;
        }

        // x along the span, y up.
        Vec3 u = span * (1.0f / spanLength);
        Vec3 up{0.0f, 1.0f, 0.0f};
        Vec3 yAxis = up - u * Anim::Dot(up, u);
        if (Anim::Length(yAxis) < 1e-3f)
        {
            error = "chain must not be vertical";
            return false;
        }
        yAxis = Anim::Normalize(yAxis);
        Vec3 zAxis = Anim::Cross(u, yAxis);
        Mat3 R = Anim::FromColumns(u, yAxis, zAxis);
        float gap = (spanLength - plankTotal) / static_cast<float>(desc.plankCount + 1);

        int anchorA = world.AddAnchor("Anchor A", desc.anchorA);
        int anchorB = world.AddAnchor("Anchor B", desc.anchorB);

        int first = world.BodyCount();
        for (int i = 0; i < desc.plankCount; ++i)
        {
            BodyDesc body;
            body.name = "Plank " + std::to_string(i);
            body.halfExtents = h;
            body.mass = desc.plankMass;
            body.rotation = R;
            body.position = desc.anchorA + u * (gap * (i + 1) + h.x * (2 * i + 1));
            world.AddBody(body);
        }

        // End-edge attachment points.
        Vec3 leftNear{-h.x, 0.0f, h.z};
        Vec3 leftFar{-h.x, 0.0f, -h.z};
        Vec3 rightNear{h.x, 0.0f, h.z};
        Vec3 rightFar{h.x, 0.0f, -h.z};
        float L0 = desc.restLength;
        // Rest lengths of the straight layout.
        float anchorRest = std::sqrt(L0 * L0 + h.z * h.z);
        float diagonalRest = std::sqrt(L0 * L0 + 4.0f * h.z * h.z);

        auto add = [&](SpringEnd a, SpringEnd b, float rest)
        {
            Spring s;
            s.a = a;
            s.b = b;
            s.stiffness = desc.stiffness;
            s.damping = desc.damping;
            s.restLength = rest;
            world.AddSpring(s);
        };
        auto bodyEnd = [](int body, const Vec3& p)
        {
            SpringEnd e;
            e.body = body;
            e.localPoint = p;
            return e;
        };
        SpringEnd endA;
        endA.anchor = anchorA;
        SpringEnd endB;
        endB.anchor = anchorB;

        int last = first + desc.plankCount - 1;
        add(endA, bodyEnd(first, leftNear), anchorRest);
        add(endA, bodyEnd(first, leftFar), anchorRest);
        for (int i = first; i < last; ++i)
        {
            add(bodyEnd(i, rightNear), bodyEnd(i + 1, leftNear), L0);
            add(bodyEnd(i, rightFar), bodyEnd(i + 1, leftFar), L0);
            if (desc.diagonals)
            {
                add(bodyEnd(i, rightNear), bodyEnd(i + 1, leftFar), diagonalRest);
                add(bodyEnd(i, rightFar), bodyEnd(i + 1, leftNear), diagonalRest);
            }
        }
        add(bodyEnd(last, rightNear), endB, anchorRest);
        add(bodyEnd(last, rightFar), endB, anchorRest);
        return true;
    }
}
