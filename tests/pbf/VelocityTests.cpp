#include "pch.h"
#include <algorithm>
#include <cmath>
#include <random>
#include <glad/glad.h>

#include "PbfGpuTests.h"
#include "TestFramework.h"
#include "pbf/PbfSim.h"

namespace
{
    PbfBinding Domain(float radius, int maxParticles)
    {
        PbfBinding b;
        b.enabled = true;
        b.particleRadius = radius;
        b.domainMin = glm::vec3(0.0f);
        b.domainMax = glm::vec3(0.6f);
        b.maxParticles = maxParticles;
        return b;
    }

    std::vector<Pbf::Particle> Block(glm::vec3 origin, int n, float spacing)
    {
        std::vector<Pbf::Particle> ps;
        for (int k = 0; k < n; ++k)
            for (int j = 0; j < n; ++j)
                for (int i = 0; i < n; ++i)
                {
                    Pbf::Particle p;
                    p.position[0] = origin.x + i * spacing;
                    p.position[1] = origin.y + j * spacing;
                    p.position[2] = origin.z + k * spacing;
                    ps.push_back(p);
                }
        return ps;
    }

    bool Start(Pbf::PbfSim& sim, const PbfBinding& b)
    {
        sim.SetShaderDir(TRL_SHADER_DIR);
        bool ok = sim.Load(b);
        CHECK(ok);
        return ok && sim.IsLoaded();
    }

    void TestXsphConservesMomentum()
    {
        Pbf::PbfSim sim;
        float r = 0.0125f;
        if (!Start(sim, Domain(r, 4096)))
            return;
        std::vector<Pbf::Particle> ps = Block(glm::vec3(0.2f), 8, 2.0f * r);
        std::mt19937 rng(3);
        std::uniform_real_distribution<float> u(-1.0f, 1.0f);
        glm::dvec3 before(0.0);
        double varianceBefore = 0.0;
        for (Pbf::Particle& p : ps)
        {
            p.velocity[0] = 0.5f + u(rng);
            p.velocity[1] = u(rng);
            p.velocity[2] = -0.25f + u(rng);
            before += glm::dvec3(p.velocity[0], p.velocity[1], p.velocity[2]);
        }
        glm::dvec3 mean = before / static_cast<double>(ps.size());
        for (const Pbf::Particle& p : ps)
        {
            glm::dvec3 d = glm::dvec3(p.velocity[0], p.velocity[1], p.velocity[2]) - mean;
            varianceBefore += glm::dot(d, d);
        }

        sim.settings.vorticityEps = 0.0f;
        sim.settings.xsphC = 0.1f;
        sim.UploadParticles(ps);
        sim.BuildGrid();
        sim.ApplyVelocityPasses();

        std::vector<float> v;
        sim.DownloadVelocities(v);
        glm::dvec3 after(0.0);
        for (size_t i = 0; i < v.size() / 4; ++i)
            after += glm::dvec3(v[i * 4], v[i * 4 + 1], v[i * 4 + 2]);
        double varianceAfter = 0.0;
        glm::dvec3 meanAfter = after / static_cast<double>(ps.size());
        for (size_t i = 0; i < v.size() / 4; ++i)
        {
            glm::dvec3 d = glm::dvec3(v[i * 4], v[i * 4 + 1], v[i * 4 + 2]) - meanAfter;
            varianceAfter += glm::dot(d, d);
        }

        std::printf("[PBF] XSPH: |sum v| %.4f -> %.4f, variance %.2f -> %.2f\n",
                    glm::length(before), glm::length(after), varianceBefore, varianceAfter);
        CHECK(glm::length(after - before) < 1e-3 * static_cast<double>(ps.size()));
        CHECK(varianceAfter < varianceBefore);
        sim.Shutdown();
    }

    void TestVorticityOfRigidRotation()
    {
        Pbf::PbfSim sim;
        float r = 0.0125f;
        if (!Start(sim, Domain(r, 8192)))
            return;
        int n = 16;
        float spacing = 2.0f * r;
        glm::vec3 origin(0.1f);
        glm::vec3 center = origin + glm::vec3(0.5f * spacing * (n - 1));
        glm::vec3 omega(0.0f, 0.0f, 2.0f);
        std::vector<Pbf::Particle> ps = Block(origin, n, spacing);
        for (Pbf::Particle& p : ps)
        {
            glm::vec3 v = glm::cross(omega, glm::vec3(p.position[0], p.position[1], p.position[2]) - center);
            p.velocity[0] = v.x;
            p.velocity[1] = v.y;
            p.velocity[2] = v.z;
        }
        sim.settings.vorticityEps = 1e-6f;
        sim.settings.xsphC = 0.0f;
        sim.UploadParticles(ps);
        sim.BuildGrid();
        sim.ApplyVelocityPasses();

        std::vector<float> w;
        std::vector<float> pos;
        sim.DownloadVorticity(w);
        sim.DownloadPositions(pos);
        float h = sim.SmoothingRadius();
        double sum = 0.0;
        int count = 0;
        float worstOffAxis = 0.0f;
        for (size_t i = 0; i < pos.size() / 4; ++i)
        {
            glm::vec3 p(pos[i * 4], pos[i * 4 + 1], pos[i * 4 + 2]);
            glm::vec3 d = glm::abs(p - center);
            if (std::max(d.x, std::max(d.y, d.z)) > 0.5f * spacing * (n - 1) - h - 1e-3f)
                continue;
            sum += w[i * 4 + 2];
            worstOffAxis = std::max(worstOffAxis, std::max(std::fabs(w[i * 4]), std::fabs(w[i * 4 + 1])));
            ++count;
        }
        double average = count > 0 ? sum / count : 0.0;
        std::printf("[PBF] vorticity of rigid rotation: %d interior particles, w_z avg %.3f (expected %.3f), max off-axis %.2e\n",
                    count, average, 2.0 * omega.z, worstOffAxis);
        CHECK(count > 100);
        CHECK(average > 0.7 * 2.0 * omega.z);
        CHECK(average < 1.3 * 2.0 * omega.z);
        CHECK(worstOffAxis < 0.05f * 2.0f * omega.z);
        sim.Shutdown();
    }

    void TestEmitter()
    {
        Pbf::PbfSim sim;
        float r = 0.0125f;
        PbfBinding b = Domain(r, 500);
        PbfShapeBinding e;
        e.boxMin = glm::vec3(0.05f, 0.4f, 0.2f);
        e.boxMax = glm::vec3(0.07f, 0.5f, 0.35f);
        e.velocity = glm::vec3(1.5f, 0.0f, 0.0f);
        b.emitters.push_back(e);
        if (!Start(sim, b))
            return;
        CHECK(sim.EmitterCount() == 1);
        CHECK(sim.ParticleCount() == 0);
        int added = sim.EmitFrame(0.1f);
        CHECK(added > 0);
        CHECK(sim.ParticleCount() == added);
        for (int frame = 0; frame < 30; ++frame)
            sim.StepFrame();
        sim.EmitFrame(100.0f);
        CHECK(sim.ParticleCount() == 500);

        std::vector<float> pos;
        sim.DownloadPositions(pos);
        bool finite = true;
        for (float f : pos)
            finite = finite && std::isfinite(f);
        CHECK(finite);
        sim.Shutdown();
    }

    void TestMouseForce()
    {
        Pbf::PbfSim sim;
        float r = 0.0125f;
        if (!Start(sim, Domain(r, 4096)))
            return;
        sim.UploadParticles(Block(glm::vec3(0.1f), 10, 2.0f * r));
        sim.mouse.active = true;
        sim.mouse.origin = glm::vec3(-1.0f, 0.2125f, 0.2125f);
        sim.mouse.direction = glm::vec3(1.0f, 0.0f, 0.0f);
        sim.mouse.radius = 0.05f;
        sim.mouse.impulse = glm::vec3(0.0f, 1.0f, 0.0f);
        sim.ApplyMouseForce();

        std::vector<float> pos;
        std::vector<float> v;
        sim.DownloadPositions(pos);
        sim.DownloadVelocities(v);
        int pushed = 0;
        int wrong = 0;
        for (size_t i = 0; i < pos.size() / 4; ++i)
        {
            float dy = pos[i * 4 + 1] - 0.2125f;
            float dz = pos[i * 4 + 2] - 0.2125f;
            float d = std::sqrt(dy * dy + dz * dz);
            float vy = v[i * 4 + 1];
            if (d < 0.045f)
            {
                pushed += vy > 0.0f ? 1 : 0;
                wrong += vy <= 0.0f ? 1 : 0;
            }
            else if (d > 0.055f)
            {
                wrong += vy != 0.0f ? 1 : 0;
            }
        }
        CHECK(pushed > 0);
        CHECK(wrong == 0);
        sim.Shutdown();
    }
}

void RunVelocityTests()
{
    TestXsphConservesMomentum();
    TestVorticityOfRigidRotation();
    TestEmitter();
    TestMouseForce();
}
