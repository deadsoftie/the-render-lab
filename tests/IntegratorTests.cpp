#include <cmath>
#include <cstdio>

#include "PhysicsTests.h"
#include "TestFramework.h"
#include "physics/Integrator.h"

using namespace Phys;
using Anim::Cross;
using Anim::Dot;
using Anim::Length;

namespace
{
    constexpr float kTwoPi = 6.28318530717958647692f;

    const IntegratorType kAllTypes[kIntegratorCount] = {IntegratorType::Euler,
                                                        IntegratorType::Midpoint,
                                                        IntegratorType::Verlet,
                                                        IntegratorType::RK2,
                                                        IntegratorType::RK4};

    RigidBodyProperties PlankProperties()
    {
        return MakeProperties(ComputeMassProperties(BoxMassPoints(Vec3{}, Vec3{0.3f, 0.025f, 0.125f}, 2.0f, 12)));
    }

    Mat3 RotationFromAxisAngle(const Vec3& axis, float angle)
    {
        Anim::Quat q = Anim::FromAxisAngle(Anim::Normalize(axis), angle);
        return Anim::FromColumns(Anim::Rotate(q, Vec3{1, 0, 0}), Anim::Rotate(q, Vec3{0, 1, 0}), Anim::Rotate(q, Vec3{0, 0, 1}));
    }

    class OscillatorSystem : public DynamicSystem
    {
       public:
        RigidBodyProperties props;
        float k = 0.0f;

        void Evaluate(const SystemState& y, float, SystemDerivative& out) override
        {
            out.resize(y.size());
            Kinematics kin = ComputeKinematics(props, y[0]);
            ForceAccumulator acc;
            acc.AddForce(y[0].c * -k);
            out[0] = Derivative(kin, acc);
        }
    };

    class SpringPairSystem : public DynamicSystem
    {
       public:
        RigidBodyProperties props;
        Vec3 attachA{0.3f, 0.0f, 0.05f};
        Vec3 attachB{-0.3f, 0.02f, 0.0f};
        float k = 50.0f;
        float restLength = 0.2f;

        void Evaluate(const SystemState& y, float, SystemDerivative& out) override
        {
            out.resize(y.size());
            Kinematics ka = ComputeKinematics(props, y[0]);
            Kinematics kb = ComputeKinematics(props, y[1]);
            Vec3 pa = WorldPoint(ka, y[0], attachA);
            Vec3 pb = WorldPoint(kb, y[1], attachB);
            Vec3 d = pa - pb;
            float len = Length(d);
            Vec3 f = d * (-k * (len - restLength) / len);

            ForceAccumulator accA;
            ForceAccumulator accB;
            accA.AddForceAtPoint(f, pa, y[0].c);
            accB.AddForceAtPoint(-f, pb, y[1].c);
            out[0] = Derivative(ka, accA);
            out[1] = Derivative(kb, accB);
        }

        float Energy(const SystemState& y) const
        {
            float e = 0.0f;
            Vec3 points[2];
            const Vec3 attach[2] = {attachA, attachB};
            for (int i = 0; i < 2; ++i)
            {
                Kinematics kin = ComputeKinematics(props, y[i]);
                e += LinearKineticEnergy(props, y[i]) + AngularKineticEnergy(kin, y[i]);
                points[i] = WorldPoint(kin, y[i], attach[i]);
            }
            float stretch = Length(points[0] - points[1]) - restLength;
            return e + 0.5f * k * stretch * stretch;
        }

        SystemState Initial() const
        {
            return {MakeState(props, Vec3{-0.5f, 0.0f, 0.0f}, RotationFromAxisAngle(Vec3{0, 0, 1}, 0.3f), Vec3{0.2f, 0.5f, 0.0f}, Vec3{0.0f, 1.0f, 2.0f}),
                    MakeState(props, Vec3{0.5f, 0.1f, 0.0f}, RotationFromAxisAngle(Vec3{1, 1, 0}, -0.4f), Vec3{-0.1f, 0.0f, 0.3f}, Vec3{1.5f, 0.0f, -0.5f})};
        }
    };

    Vec3 TotalLinearMomentum(const SystemState& y)
    {
        Vec3 p;
        for (const RigidBodyState& s : y)
            p = p + s.P;
        return p;
    }

    Vec3 TotalAngularMomentum(const SystemState& y)
    {
        Vec3 l;
        for (const RigidBodyState& s : y)
            l = l + Cross(s.c, s.P) + s.L;
        return l;
    }

    float OscillatorError(IntegratorType type, int steps)
    {
        OscillatorSystem system;
        system.props = PlankProperties();
        float omega = kTwoPi;
        system.k = system.props.mass * omega * omega;

        float T = 1.3f;
        float dt = T / static_cast<float>(steps);
        SystemState y = {MakeState(system.props, Vec3{1, 0, 0}, Mat3{}, Vec3{}, Vec3{})};
        Integrator integrator;
        for (int i = 0; i < steps; ++i)
            integrator.Step(system, y, i * dt, dt, type);

        double phase = static_cast<double>(omega) * T;
        float xExact = static_cast<float>(std::cos(phase));
        float vExact = static_cast<float>(-static_cast<double>(omega) * std::sin(phase));
        float v = y[0].P.x * system.props.invMass;
        return std::abs(y[0].c.x - xExact) + std::abs(v - vExact) / omega + std::abs(y[0].c.y) + std::abs(y[0].c.z);
    }

    void TestConvergenceOrder()
    {
        struct Case
        {
            IntegratorType type;
            int steps;
            float lo;
            float hi;
        };
        const Case cases[] = {{IntegratorType::Euler, 1300, 1.7f, 2.4f},
                              {IntegratorType::Midpoint, 130, 3.3f, 4.8f},
                              {IntegratorType::Verlet, 130, 3.3f, 4.8f},
                              {IntegratorType::RK2, 130, 3.3f, 4.8f},
                              {IntegratorType::RK4, 20, 12.0f, 20.0f}};

        std::printf("[Integrators] convergence (error ratio when dt halves)\n");
        for (const Case& c : cases)
        {
            float coarse = OscillatorError(c.type, c.steps);
            float fine = OscillatorError(c.type, c.steps * 2);
            float ratio = coarse / fine;
            std::printf("  %-22s N=%5d err=%.3e  N=%5d err=%.3e  ratio=%.2f\n",
                        IntegratorName(c.type), c.steps, coarse, c.steps * 2, fine, ratio);
            CHECK(ratio >= c.lo);
            CHECK(ratio <= c.hi);
        }
    }

    void TestSpringPairConservation()
    {
        SpringPairSystem system;
        system.props = PlankProperties();
        SystemState y0 = system.Initial();
        float e0 = system.Energy(y0);
        Vec3 p0 = TotalLinearMomentum(y0);
        Vec3 l0 = TotalAngularMomentum(y0);
        float dt = 1.0f / 240.0f;
        int steps = 2400;

        std::printf("[Integrators] spring pair, %d steps at dt=1/240\n", steps);
        for (IntegratorType type : kAllTypes)
        {
            SystemState y = y0;
            Integrator integrator;
            float maxDrift = 0.0f;
            for (int i = 0; i < steps; ++i)
            {
                integrator.Step(system, y, i * dt, dt, type);
                maxDrift = std::fmax(maxDrift, std::abs(system.Energy(y) - e0) / e0);
            }
            float endDrift = (system.Energy(y) - e0) / e0;
            float pDrift = Length(TotalLinearMomentum(y) - p0);
            float lDrift = Length(TotalAngularMomentum(y) - l0) / Length(l0);
            std::printf("  %-22s energy end=%+.3e max=%.3e  |dP|=%.2e  |dL|/|L|=%.2e\n",
                        IntegratorName(type), endDrift, maxDrift, pDrift, lDrift);

            CHECK(pDrift < 1e-3f);
            if (type == IntegratorType::RK4)
            {
                CHECK(maxDrift < 1e-3f);
                CHECK(lDrift < 1e-3f);
            }
            if (type == IntegratorType::Verlet)
            {
                CHECK(maxDrift < 1e-2f);
                CHECK(lDrift < 1e-3f);
            }
            if (type == IntegratorType::Euler)
                CHECK(endDrift > 0.05f);
        }
    }

    void TestTorqueFreeEnergyRK4()
    {
        RigidBodyProperties props = PlankProperties();
        SystemState y = {MakeState(props, Vec3{}, Mat3{}, Vec3{}, Vec3{0.4f, 3.0f, 0.2f})};

        class FreeSystem : public DynamicSystem
        {
           public:
            RigidBodyProperties props;
            void Evaluate(const SystemState& s, float, SystemDerivative& out) override
            {
                out.resize(s.size());
                out[0] = Derivative(ComputeKinematics(props, s[0]), ForceAccumulator{});
            }
        } system;
        system.props = props;

        float e0 = AngularKineticEnergy(ComputeKinematics(props, y[0]), y[0]);
        Integrator integrator;
        float dt = 1.0f / 240.0f;
        for (int i = 0; i < 10000; ++i)
            integrator.Step(system, y, i * dt, dt, IntegratorType::RK4);
        float e1 = AngularKineticEnergy(ComputeKinematics(props, y[0]), y[0]);
        CHECK(std::abs(e1 - e0) / e0 < 1e-3f);
        CHECK_NEAR(Anim::Determinant(y[0].R), 1.0f, 1e-5f);
    }

    void TestMidpointDiffersFromHeun()
    {
        SpringPairSystem system;
        system.props = PlankProperties();
        SystemState a = system.Initial();
        SystemState b = a;
        Integrator ia;
        Integrator ib;
        ia.Step(system, a, 0.0f, 0.05f, IntegratorType::Midpoint);
        ib.Step(system, b, 0.0f, 0.05f, IntegratorType::RK2);
        CHECK(Length(a[0].P - b[0].P) > 1e-6f);
    }
}

void RunIntegratorTests()
{
    TestConvergenceOrder();
    TestSpringPairConservation();
    TestTorqueFreeEnergyRK4();
    TestMidpointDiffersFromHeun();
}
