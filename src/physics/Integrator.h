#pragma once
#include <vector>

#include "physics/RigidBody.h"

namespace Phys
{
    enum class IntegratorType
    {
        Euler,
        Midpoint,
        Verlet,
        RK2,
        RK4
    };

    constexpr int kIntegratorCount = 5;
    const char* IntegratorName(IntegratorType type);

    // All bodies form one state, since springs couple them.
    using SystemState = std::vector<RigidBodyState>;
    using SystemDerivative = std::vector<RigidBodyDerivative>;

    // dy/dt for the whole system.
    class DynamicSystem
    {
       public:
        virtual ~DynamicSystem() = default;
        virtual void Evaluate(const SystemState& y, float t, SystemDerivative& out) = 0;
    };

    // out = y + h d; out may alias y.
    void AdvanceSystem(const SystemState& y, const SystemDerivative& d, float h, SystemState& out);
    void RenormalizeSystem(SystemState& y);

    // Re-orthonormalizes R after each step.
    class Integrator
    {
       public:
        void Step(DynamicSystem& system, SystemState& y, float t, float dt, IntegratorType type);

       private:
        void StepEuler(DynamicSystem& system, SystemState& y, float t, float dt);
        void StepMidpoint(DynamicSystem& system, SystemState& y, float t, float dt);
        void StepVerlet(DynamicSystem& system, SystemState& y, float t, float dt);
        void StepRK2(DynamicSystem& system, SystemState& y, float t, float dt);
        void StepRK4(DynamicSystem& system, SystemState& y, float t, float dt);

        SystemDerivative m_k1;
        SystemDerivative m_k2;
        SystemDerivative m_k3;
        SystemDerivative m_k4;
        SystemState m_stage;
    };
}
