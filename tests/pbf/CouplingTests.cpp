#include "pch.h"
#include <algorithm>
#include <cmath>
#include <random>
#include <glad/glad.h>
#include <glm/gtc/matrix_transform.hpp>

#include "PbfGpuTests.h"
#include "TestFramework.h"
#include "pbf/PbfSim.h"

namespace
{
    bool Start(Pbf::PbfSim& sim, const PbfBinding& b)
    {
        sim.SetShaderDir(TRL_SHADER_DIR);
        bool ok = sim.Load(b);
        CHECK(ok);
        return ok && sim.IsLoaded();
    }

    PbfBinding EmptyDomain(float radius, glm::vec3 domainMax)
    {
        PbfBinding b;
        b.enabled = true;
        b.particleRadius = radius;
        b.domainMin = glm::vec3(0.0f);
        b.domainMax = domainMax;
        b.maxParticles = 8192;
        b.vorticity = 0.0f;
        b.xsph = 0.0f;
        return b;
    }

    float BoxDistance(const Pbf::CouplingBody& body, glm::vec3 p)
    {
        glm::vec3 local = glm::transpose(body.rotation) * (p - body.com) + body.cObj;
        glm::vec3 q = glm::abs(local) - body.halfExtents;
        float outside = glm::length(glm::max(q, glm::vec3(0.0f)));
        float inside = std::min(std::max(q.x, std::max(q.y, q.z)), 0.0f);
        return outside + inside;
    }

    void TestPushOutConservesMomentum()
    {
        Pbf::PbfSim sim;
        float r = 0.0125f;
        if (!Start(sim, EmptyDomain(r, glm::vec3(0.6f))))
            return;
        sim.settings.gravity[1] = 0.0f;

        Pbf::CouplingBody body;
        body.com = glm::vec3(0.3f, 0.3f, 0.3f);
        body.halfExtents = glm::vec3(0.1f, 0.04f, 0.08f);
        glm::mat4 rot = glm::rotate(glm::mat4(1.0f), glm::radians(25.0f), glm::vec3(0.0f, 0.0f, 1.0f)) *
                        glm::rotate(glm::mat4(1.0f), glm::radians(10.0f), glm::vec3(1.0f, 0.0f, 0.0f));
        body.rotation = glm::mat3(rot);

        std::mt19937 rng(11);
        std::uniform_real_distribution<float> u(-0.9f, 0.9f);
        std::uniform_real_distribution<float> biased(-0.2f, 0.9f);
        std::vector<Pbf::Particle> ps;
        for (int i = 0; i < 200; ++i)
        {
            glm::vec3 local(biased(rng) * body.halfExtents.x, u(rng) * body.halfExtents.y, u(rng) * body.halfExtents.z);
            glm::vec3 p = body.com + body.rotation * local;
            Pbf::Particle particle;
            particle.position[0] = p.x;
            particle.position[1] = p.y;
            particle.position[2] = p.z;
            ps.push_back(particle);
        }

        sim.UploadParticles(ps);
        sim.coupling.twoWay = true;
        sim.coupling.friction = 0.0f;
        sim.SetBodies({body});
        sim.ClearBodyImpulses();
        sim.Predict();

        std::vector<float> predicted;
        sim.DownloadPredicted(predicted);
        std::vector<Pbf::BodyForce> impulses;
        sim.DownloadBodyImpulses(impulses);
        CHECK(impulses.size() == 1);
        if (impulses.size() != 1)
            return;

        float m = sim.Params().mass;
        float dt = sim.TimeStep();
        glm::dvec3 particleMomentum(0.0);
        glm::dvec3 expectedTorque(0.0);
        double magnitude = 0.0;
        float worstDistance = 1.0f;
        for (size_t i = 0; i < ps.size(); ++i)
        {
            glm::vec3 before(ps[i].position[0], ps[i].position[1], ps[i].position[2]);
            glm::vec3 after(predicted[i * 4], predicted[i * 4 + 1], predicted[i * 4 + 2]);
            glm::vec3 j = m * (after - before) / dt;
            particleMomentum += glm::dvec3(j);
            magnitude += glm::length(j);
            glm::vec3 local = glm::transpose(body.rotation) * (after - body.com);
            glm::vec3 contact = body.com + body.rotation * glm::clamp(local, -body.halfExtents, body.halfExtents);
            expectedTorque += glm::dvec3(glm::cross(contact - body.com, -j));
            worstDistance = std::min(worstDistance, BoxDistance(body, after));
        }
        glm::dvec3 bodyImpulse(impulses[0].force);
        glm::dvec3 bodyTorque(impulses[0].torque);
        double momentumError = glm::length(particleMomentum + bodyImpulse);
        double torqueError = glm::length(bodyTorque - expectedTorque);
        std::printf("[PBF] push-out: min surface distance %.5f (r %.4f), |J particles| %.4f, |J body| %.4f, momentum error %.2e, torque error %.2e of %.4f\n",
                    worstDistance,
                    r,
                    glm::length(particleMomentum),
                    glm::length(bodyImpulse),
                    momentumError,
                    torqueError,
                    glm::length(expectedTorque));
        CHECK(worstDistance > r - 1e-3f);
        CHECK(magnitude > 0.0);
        CHECK(momentumError < 1e-3 * magnitude + 1e-4);
        CHECK(torqueError < 0.02 * glm::length(expectedTorque) + 1e-4);
        sim.Shutdown();
    }

    void TestFrictionDragsWater()
    {
        Pbf::PbfSim sim;
        float r = 0.0125f;
        if (!Start(sim, EmptyDomain(r, glm::vec3(0.6f))))
            return;
        sim.settings.gravity[1] = 0.0f;
        sim.settings.xsphC = 0.0f;
        sim.settings.vorticityEps = 0.0f;

        Pbf::CouplingBody body;
        body.com = glm::vec3(0.3f);
        body.halfExtents = glm::vec3(0.1f, 0.04f, 0.1f);
        body.velocity = glm::vec3(1.0f, 0.0f, 0.0f);

        Pbf::Particle particle;
        particle.position[0] = 0.3f;
        particle.position[1] = 0.3f + body.halfExtents.y + 1.2f * r;
        particle.position[2] = 0.3f;
        sim.UploadParticles({particle});
        sim.coupling.twoWay = true;
        sim.coupling.friction = 0.5f;
        sim.SetBodies({body});
        sim.ClearBodyImpulses();
        sim.Substep();

        std::vector<float> v;
        sim.DownloadVelocities(v);
        std::vector<Pbf::BodyForce> impulses;
        sim.DownloadBodyImpulses(impulses);
        float m = sim.Params().mass;
        std::printf("[PBF] friction: particle v (%.4f, %.4f, %.4f), body impulse x %.6f (expected %.6f)\n",
                    v[0],
                    v[1],
                    v[2],
                    impulses.empty() ? 0.0f : impulses[0].force.x,
                    -0.5f * m);
        CHECK_NEAR(v[0], 0.5f, 1e-3f);
        CHECK_NEAR(v[1], 0.0f, 1e-3f);
        CHECK(!impulses.empty());
        if (!impulses.empty())
            CHECK_NEAR(impulses[0].force.x, -0.5f * m, 0.01f * m);
        sim.Shutdown();
    }

    void TestFixedBoxBuoyancyAndReadback()
    {
        Pbf::PbfSim sim;
        float r = 0.0125f;
        PbfBinding b = EmptyDomain(r, glm::vec3(0.4f, 0.6f, 0.4f));
        PbfShapeBinding pool;
        pool.boxMin = glm::vec3(0.0f);
        pool.boxMax = glm::vec3(0.4f, 0.2f, 0.4f);
        b.seeds.push_back(pool);
        if (!Start(sim, b))
            return;
        sim.collectNeighborStats = false;
        int count = sim.ParticleCount();

        for (int frame = 0; frame < 60; ++frame)
            sim.StepFrame();

        Pbf::CouplingBody body;
        body.com = glm::vec3(0.2f, 0.2f, 0.2f);
        body.halfExtents = glm::vec3(0.1f, 0.05f, 0.1f);
        sim.coupling.twoWay = true;
        sim.coupling.friction = 0.1f;
        sim.SetBodies({body});
        for (int frame = 0; frame < 90; ++frame)
            sim.StepFrame();

        sim.ClearBodyImpulses();
        const int measured = 60;
        for (int frame = 0; frame < measured; ++frame)
            sim.StepFrame();
        std::vector<Pbf::BodyForce> impulses;
        sim.DownloadBodyImpulses(impulses);
        CHECK(impulses.size() == 1);
        if (impulses.size() != 1)
            return;
        float seconds = measured / 60.0f;
        glm::vec3 force = impulses[0].force / seconds;
        glm::vec3 torque = impulses[0].torque / seconds;

        float poolArea = 0.4f * 0.4f;
        float boxArea = 4.0f * body.halfExtents.x * body.halfExtents.z;
        float spacing = 2.0f * r;
        float waterVolume = static_cast<float>(count) * spacing * spacing * spacing;
        float bottom = body.com.y - body.halfExtents.y;
        float level = (waterVolume - boxArea * bottom) / (poolArea - boxArea);
        float expected = 1000.0f * 9.81f * boxArea * (level - bottom);
        std::printf("[PBF] fixed box in %d particles: F (%.2f, %.2f, %.2f) N, expected lift %.2f N (level %.3f m), |T| %.3f N m\n",
                    count,
                    force.x,
                    force.y,
                    force.z,
                    expected,
                    level,
                    glm::length(torque));
        CHECK(force.y > 0.6f * expected);
        CHECK(force.y < 1.5f * expected);
        CHECK(std::fabs(force.x) < 0.15f * force.y);
        CHECK(std::fabs(force.z) < 0.15f * force.y);
        CHECK(glm::length(torque) < 0.2f * force.y * body.halfExtents.x);

        sim.SetBodyTime(0.0f);
        bool polled = false;
        for (int update = 0; update < 6 && !polled; ++update)
        {
            sim.Update(1.0f / 60.0f);
            polled = sim.PollBodyForces();
        }
        CHECK(polled);
        if (polled)
        {
            std::vector<Pbf::BodyForce> last;
            sim.DownloadBodyImpulses(last);
            const Pbf::BodyForce& f = sim.BodyForces()[0];
            std::printf("[PBF] readback: polled F y %.2f N, frame impulse / dt %.2f N\n",
                        f.force.y,
                        last[0].force.y * 60.0f);
            CHECK_NEAR(f.force.y, last[0].force.y * 60.0f, 1e-2f);
            CHECK(std::isfinite(f.torque.x) && std::isfinite(f.torque.y) && std::isfinite(f.torque.z));
        }

        sim.SetBodies({});
        CHECK(sim.BodyCount() == 0);
        CHECK(glGetError() == GL_NO_ERROR);
        sim.Shutdown();
    }
}

void RunCouplingTests()
{
    TestPushOutConservesMomentum();
    TestFrictionDragsWater();
    TestFixedBoxBuoyancyAndReadback();
}
