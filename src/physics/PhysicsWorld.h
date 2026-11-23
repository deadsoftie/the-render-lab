#pragma once
#include <string>
#include <vector>

#include "physics/Integrator.h"
#include "physics/RigidBody.h"

// Planks, anchors, springs and forces. Every integrator stage re-evaluates all forces.
namespace Phys
{
    // User-moved spring end; slides to its target over a frame.
    struct Anchor
    {
        std::string name;
        Vec3 start;
        Vec3 target;
        Vec3 velocity;
    };

    // Body-local point if body >= 0, else anchor.
    struct SpringEnd
    {
        int body = -1;
        int anchor = -1;
        Vec3 localPoint;
    };

    struct Spring
    {
        SpringEnd a;
        SpringEnd b;
        float stiffness = 100.0f;
        float damping = 1.0f;
        float restLength = 0.0f;
    };

    // position = model origin.
    struct BodyDesc
    {
        std::string name;
        Vec3 halfExtents{0.25f, 0.025f, 0.1f};
        float mass = 1.0f;
        Vec3 position;
        Mat3 rotation;
        Vec3 velocity;
        Vec3 angularVelocity;
    };

    struct PhysicsSettings
    {
        IntegratorType integrator = IntegratorType::RK2;
        float dt = 1.0f / 240.0f;
        int maxStepsPerFrame = 64;
        Vec3 gravity{0.0f, -9.81f, 0.0f};
        float linearDrag = 0.1f;   // F = -b P
        float angularDrag = 0.1f;  // tau = -b L
    };

    struct EnergyReport
    {
        float kinetic = 0.0f;
        float gravitational = 0.0f;
        float elastic = 0.0f;
        float Total() const { return kinetic + gravitational + elastic; }
    };

    // Local water plane around one plank.
    struct WaterSurface
    {
        bool valid = false;
        float level = 0.0f;
        float slopeX = 0.0f;
        float slopeZ = 0.0f;
        Vec3 reference;
        Vec3 velocity;
    };

    // Uses the local plane when valid, else the rest level.
    struct FluidBuoyancy
    {
        bool enabled = false;
        float level = 0.0f;
        float density = 1000.0f;
        float drag = 2.0f;
        std::vector<WaterSurface> local;  // per body
    };

    // Ctrl + drag spring.
    struct MouseSpring
    {
        bool active = false;
        int body = -1;
        Vec3 localPoint;
        Vec3 target;
        float stiffness = 150.0f;
        float damping = 8.0f;
    };

    class PhysicsWorld : public DynamicSystem
    {
       public:
        void Clear();
        int AddBody(const BodyDesc& desc);
        int AddAnchor(const std::string& name, const Vec3& position);
        int AddSpring(const Spring& spring);
        float CurrentLength(const SpringEnd& a, const SpringEnd& b) const;
        int FindBody(const std::string& name) const;
        int FindAnchor(const std::string& name) const;
        // State that Reset returns to.
        void CaptureInitialState();
        void Reset();

        void SetAnchorTarget(int index, const Vec3& position);
        // Fixed steps for the frame time; returns the count.
        int Advance(float frameSeconds);
        void StepOnce();
        // P += J, L += (p - c) x J.
        void ApplyImpulse(int body, const Vec3& impulse, const Vec3& worldPoint);
        // Held over the next steps (impulse coupling).
        void SetExternalForce(int body, const Vec3& force, const Vec3& torque);
        void ClearExternalForces();
        // For overlays and readouts.
        ForceAccumulator FluidForce(int body) const;

        // Flowchart force pass for all bodies.
        void Evaluate(const SystemState& y, float t, SystemDerivative& out) override;

        // Interpolated within the frame.
        Vec3 AnchorPosition(int index, float t) const;
        Vec3 EndPosition(const SpringEnd& end, const SystemState& y, float t) const;
        Vec3 EndVelocity(const SpringEnd& end, const SystemState& y, float t) const;
        Vec3 EndPositionNow(const SpringEnd& end) const { return EndPosition(end, m_state, m_time); }
        Vec3 SpringForceOnA(int index) const;

        // Kinetic, gravitational, elastic.
        EnergyReport Energy() const;
        Vec3 TotalLinearMomentum() const;
        // sum c x P + L.
        Vec3 TotalAngularMomentum() const;
        Kinematics BodyKinematics(int index) const;

        int BodyCount() const { return static_cast<int>(m_bodies.size()); }
        int AnchorCount() const { return static_cast<int>(m_anchors.size()); }
        int SpringCount() const { return static_cast<int>(m_springs.size()); }
        const BodyDesc& Body(int index) const { return m_bodies[index]; }
        const RigidBodyProperties& Properties(int index) const { return m_props[index]; }
        const RigidBodyState& State(int index) const { return m_state[index]; }
        const Anchor& GetAnchor(int index) const { return m_anchors[index]; }
        const Vec3& InitialAnchor(int index) const { return m_initialAnchors[index]; }
        const Spring& GetSpring(int index) const { return m_springs[index]; }
        std::vector<Spring>& Springs() { return m_springs; }
        float Time() const { return m_time; }
        long long StepCount() const { return m_stepCount; }

        PhysicsSettings settings;
        MouseSpring mouse;
        FluidBuoyancy buoyancy;

       private:
        void BeginFrame(float duration);
        void DoStep();
        void AddBuoyancy(size_t body, const Kinematics& kin, const RigidBodyState& state, ForceAccumulator& forces) const;

        std::vector<BodyDesc> m_bodies;
        std::vector<RigidBodyProperties> m_props;
        std::vector<Anchor> m_anchors;
        std::vector<Spring> m_springs;
        SystemState m_state;
        SystemState m_initialState;
        std::vector<Vec3> m_initialAnchors;
        std::vector<Kinematics> m_kin;
        std::vector<ForceAccumulator> m_forces;
        std::vector<Vec3> m_externalForce;
        std::vector<Vec3> m_externalTorque;
        Integrator m_integrator;
        float m_time = 0.0f;
        float m_accumulator = 0.0f;
        float m_frameStart = 0.0f;
        float m_frameDuration = 0.0f;
        long long m_stepCount = 0;
    };

    // Two edge springs and two diagonals per joint, two springs per anchor.
    struct ChainDesc
    {
        int plankCount = 6;
        Vec3 anchorA{-1.5f, 2.0f, 0.0f};
        Vec3 anchorB{1.5f, 2.0f, 0.0f};
        Vec3 plankHalfExtents{0.2f, 0.025f, 0.12f};
        float plankMass = 1.0f;
        float restLength = 0.06f;
        float stiffness = 400.0f;
        float damping = 4.0f;
        bool diagonals = true;
    };

    // False with a message on invalid input.
    bool BuildChain(const ChainDesc& desc, PhysicsWorld& world, std::string& error);
}
