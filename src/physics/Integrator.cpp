#include "physics/Integrator.h"

namespace Phys
{
    namespace
    {
        // omega from R_dot R^T.
        Vec3 AngularVelocity(const RigidBodyDerivative& d, const RigidBodyState& s)
        {
            return Anim::Unskew(d.RDot * Anim::Transpose(s.R));
        }
    }

    const char* IntegratorName(IntegratorType type)
    {
        switch (type)
        {
            case IntegratorType::Euler:
                return "Euler";
            case IntegratorType::Midpoint:
                return "Midpoint Euler";
            case IntegratorType::Verlet:
                return "Velocity Verlet";
            case IntegratorType::RK2:
                return "Runge-Kutta 2 (Heun)";
            case IntegratorType::RK4:
                return "Runge-Kutta 4";
        }
        return "Unknown";
    }

    void AdvanceSystem(const SystemState& y, const SystemDerivative& d, float h, SystemState& out)
    {
        out.resize(y.size());
        for (size_t i = 0; i < y.size(); ++i)
            out[i] = Advance(y[i], d[i], h);
    }

    void RenormalizeSystem(SystemState& y)
    {
        for (RigidBodyState& s : y)
            Renormalize(s);
    }

    void Integrator::Step(DynamicSystem& system, SystemState& y, float t, float dt, IntegratorType type)
    {
        if (y.empty())
            return;

        switch (type)
        {
            case IntegratorType::Euler:
                StepEuler(system, y, t, dt);
                break;
            case IntegratorType::Midpoint:
                StepMidpoint(system, y, t, dt);
                break;
            case IntegratorType::Verlet:
                StepVerlet(system, y, t, dt);
                break;
            case IntegratorType::RK2:
                StepRK2(system, y, t, dt);
                break;
            case IntegratorType::RK4:
                StepRK4(system, y, t, dt);
                break;
        }
        RenormalizeSystem(y);
    }

    // y + dt f(y).
    void Integrator::StepEuler(DynamicSystem& system, SystemState& y, float t, float dt)
    {
        system.Evaluate(y, t, m_k1);
        AdvanceSystem(y, m_k1, dt, y);
    }

    // y + dt f(y + dt/2 f(y)).
    void Integrator::StepMidpoint(DynamicSystem& system, SystemState& y, float t, float dt)
    {
        system.Evaluate(y, t, m_k1);
        AdvanceSystem(y, m_k1, 0.5f * dt, m_stage);
        system.Evaluate(m_stage, t + 0.5f * dt, m_k2);
        AdvanceSystem(y, m_k2, dt, y);
    }

    // Kick-drift-kick. R drifts with exp(dt omega~) to stay second order.
    void Integrator::StepVerlet(DynamicSystem& system, SystemState& y, float t, float dt)
    {
        float half = 0.5f * dt;

        // Half kick.
        system.Evaluate(y, t, m_k1);
        for (size_t i = 0; i < y.size(); ++i)
        {
            y[i].P = y[i].P + m_k1[i].PDot * half;
            y[i].L = y[i].L + m_k1[i].LDot * half;
        }

        // R at the half step.
        system.Evaluate(y, t, m_k2);
        m_stage = y;
        for (size_t i = 0; i < y.size(); ++i)
        {
            Vec3 omega = AngularVelocity(m_k2[i], y[i]);
            m_stage[i].R = Anim::RotationFromVector(omega * half) * y[i].R;
        }

        // Drift.
        system.Evaluate(m_stage, t + half, m_k3);
        for (size_t i = 0; i < y.size(); ++i)
        {
            Vec3 omegaMid = AngularVelocity(m_k3[i], m_stage[i]);
            y[i].c = y[i].c + m_k2[i].cDot * dt;
            y[i].R = Anim::RotationFromVector(omegaMid * dt) * y[i].R;
        }

        // Half kick.
        system.Evaluate(y, t + dt, m_k4);
        for (size_t i = 0; i < y.size(); ++i)
        {
            y[i].P = y[i].P + m_k4[i].PDot * half;
            y[i].L = y[i].L + m_k4[i].LDot * half;
        }
    }

    // Heun: y + dt/2 (f(y) + f(y + dt f(y))).
    void Integrator::StepRK2(DynamicSystem& system, SystemState& y, float t, float dt)
    {
        system.Evaluate(y, t, m_k1);
        AdvanceSystem(y, m_k1, dt, m_stage);
        system.Evaluate(m_stage, t + dt, m_k2);
        AdvanceSystem(y, m_k1, 0.5f * dt, y);
        AdvanceSystem(y, m_k2, 0.5f * dt, y);
    }

    // y + dt/6 (k1 + 2 k2 + 2 k3 + k4).
    void Integrator::StepRK4(DynamicSystem& system, SystemState& y, float t, float dt)
    {
        float half = 0.5f * dt;

        system.Evaluate(y, t, m_k1);
        AdvanceSystem(y, m_k1, half, m_stage);
        system.Evaluate(m_stage, t + half, m_k2);
        AdvanceSystem(y, m_k2, half, m_stage);
        system.Evaluate(m_stage, t + half, m_k3);
        AdvanceSystem(y, m_k3, dt, m_stage);
        system.Evaluate(m_stage, t + dt, m_k4);

        AdvanceSystem(y, m_k1, dt / 6.0f, y);
        AdvanceSystem(y, m_k2, dt / 3.0f, y);
        AdvanceSystem(y, m_k3, dt / 3.0f, y);
        AdvanceSystem(y, m_k4, dt / 6.0f, y);
    }
}
