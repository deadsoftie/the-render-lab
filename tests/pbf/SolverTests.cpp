#include "pch.h"
#include <algorithm>
#include <cmath>
#include <glad/glad.h>

#include "PbfGpuTests.h"
#include "TestFramework.h"
#include "pbf/PbfSim.h"

namespace
{
    PbfBinding BoxDomain(glm::vec3 lo, glm::vec3 hi, float radius)
    {
        PbfBinding b;
        b.enabled = true;
        b.particleRadius = radius;
        b.domainMin = lo;
        b.domainMax = hi;
        b.maxParticles = 20000;
        return b;
    }

    std::vector<Pbf::Particle> Lattice(glm::vec3 origin, int n, float spacing)
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

    void Constraints(const Pbf::PbfSim& sim, float& mean, float& worst)
    {
        std::vector<float> predicted;
        sim.DownloadPredicted(predicted);
        size_t n = predicted.size() / 4;
        mean = 0.0f;
        worst = 0.0f;
        for (size_t i = 0; i < n; ++i)
        {
            float C = std::max(predicted[i * 4 + 3] / 1000.0f - 1.0f, 0.0f);
            mean += C;
            worst = std::max(worst, C);
        }
        mean /= static_cast<float>(std::max<size_t>(n, 1));
    }

    void TestRestDensity()
    {
        Pbf::PbfSim sim;
        sim.SetShaderDir(TRL_SHADER_DIR);
        float r = 0.0125f;
        CHECK(sim.Load(BoxDomain(glm::vec3(0.0f), glm::vec3(0.5f), r)));
        if (!sim.IsLoaded())
            return;
        float spacing = 2.0f * r;
        sim.UploadParticles(Lattice(glm::vec3(r), 20, spacing));
        sim.BuildGrid();
        sim.ComputeLambda();

        std::vector<float> predicted;
        sim.DownloadPredicted(predicted);
        float h = sim.SmoothingRadius();
        float lo = r + h + 1e-3f;
        float hi = r + 19 * spacing - h - 1e-3f;
        float worst = 0.0f;
        int interior = 0;
        for (size_t i = 0; i < predicted.size() / 4; ++i)
        {
            const float* p = &predicted[i * 4];
            bool inside = p[0] > lo && p[0] < hi && p[1] > lo && p[1] < hi && p[2] > lo && p[2] < hi;
            if (!inside)
                continue;
            ++interior;
            worst = std::max(worst, std::fabs(p[3] / 1000.0f - 1.0f));
        }
        std::printf("[PBF] rest lattice: %d interior particles, max |rho/rho0 - 1| = %.2e\n", interior, worst);
        CHECK(interior > 1000);
        CHECK(worst < 1e-4f);
        sim.Shutdown();
    }

    void TestCompressedBlockRelaxes()
    {
        Pbf::PbfSim sim;
        sim.SetShaderDir(TRL_SHADER_DIR);
        float r = 0.0125f;
        CHECK(sim.Load(BoxDomain(glm::vec3(0.0f), glm::vec3(0.6f), r)));
        if (!sim.IsLoaded())
            return;
        sim.settings.gravity[1] = 0.0f;
        sim.UploadParticles(Lattice(glm::vec3(0.2f), 10, 0.9f * 2.0f * r));

        float meanBefore, maxBefore;
        sim.BuildGrid();
        sim.ComputeLambda();
        Constraints(sim, meanBefore, maxBefore);

        float meanOne, maxOne;
        sim.ApplyDelta();
        sim.BuildGrid();
        sim.ComputeLambda();
        Constraints(sim, meanOne, maxOne);

        for (int it = 1; it < 30; ++it)
        {
            sim.ApplyDelta();
            sim.BuildGrid();
            sim.ComputeLambda();
        }
        float meanMany, maxMany;
        Constraints(sim, meanMany, maxMany);

        std::printf("[PBF] compressed block: mean C %.4f -> %.4f (1 it), max C %.4f -> %.4f (30 its)\n",
                    meanBefore, meanOne, maxBefore, maxMany);
        CHECK(maxBefore > 0.2f);
        CHECK(meanOne < meanBefore);
        CHECK(maxMany < 0.5f * maxBefore);
        sim.Shutdown();
    }

    void TestHydrostaticColumn()
    {
        Pbf::PbfSim sim;
        sim.SetShaderDir(TRL_SHADER_DIR);
        float r = 0.0125f;
        PbfBinding b = BoxDomain(glm::vec3(0.0f), glm::vec3(0.3f, 1.0f, 0.3f), r);
        PbfShapeBinding seed;
        seed.boxMin = glm::vec3(0.0f);
        seed.boxMax = glm::vec3(0.3f, 0.5f, 0.3f);
        b.seeds.push_back(seed);
        CHECK(sim.Load(b));
        if (!sim.IsLoaded())
            return;

        std::vector<float> start;
        sim.DownloadPositions(start);
        float top0 = 0.0f;
        for (size_t i = 0; i < start.size() / 4; ++i)
            top0 = std::max(top0, start[i * 4 + 1]);

        for (int frame = 0; frame < 300; ++frame)
            sim.StepFrame();
        sim.ComputeNeighborStats();

        std::vector<float> end;
        sim.DownloadPositions(end);
        bool finite = true;
        bool inside = true;
        float top1 = 0.0f;
        for (size_t i = 0; i < end.size() / 4; ++i)
        {
            const float* p = &end[i * 4];
            finite = finite && std::isfinite(p[0]) && std::isfinite(p[1]) && std::isfinite(p[2]);
            inside = inside && p[0] >= r - 1e-4f && p[0] <= 0.3f - r + 1e-4f && p[1] >= r - 1e-4f &&
                     p[1] <= 1.0f - r + 1e-4f && p[2] >= r - 1e-4f && p[2] <= 0.3f - r + 1e-4f;
            top1 = std::max(top1, p[1]);
        }
        const Pbf::NeighborStats& stats = sim.Neighbors();
        std::printf("[PBF] hydrostatic column (%d particles, 5 s): top %.4f -> %.4f m, density/rho0 avg %.3f max %.3f\n",
                    sim.ParticleCount(),
                    top0,
                    top1,
                    stats.densityAverage,
                    stats.densityMax);
        CHECK(finite);
        CHECK(inside);
        CHECK(std::fabs(top1 - top0) < sim.SmoothingRadius());
        CHECK(stats.densityMax < 1.05f);
        sim.Shutdown();
    }

    void TestObstacleRespected()
    {
        Pbf::PbfSim sim;
        sim.SetShaderDir(TRL_SHADER_DIR);
        float r = 0.0125f;
        PbfBinding b = BoxDomain(glm::vec3(-0.5f, 0.0f, -0.25f), glm::vec3(0.5f, 0.6f, 0.25f), r);
        PbfShapeBinding seed;
        seed.boxMin = glm::vec3(-0.5f, 0.0f, -0.25f);
        seed.boxMax = glm::vec3(-0.2f, 0.4f, 0.25f);
        b.seeds.push_back(seed);
        PbfShapeBinding box;
        box.boxMin = glm::vec3(0.1f, 0.0f, -0.1f);
        box.boxMax = glm::vec3(0.2f, 0.15f, 0.1f);
        b.obstacles.push_back(box);
        PbfShapeBinding ball;
        ball.type = PbfShapeType::Sphere;
        ball.center = glm::vec3(-0.05f, 0.05f, 0.0f);
        ball.radius = 0.06f;
        b.obstacles.push_back(ball);
        CHECK(sim.Load(b));
        if (!sim.IsLoaded())
            return;

        for (int frame = 0; frame < 120; ++frame)
            sim.StepFrame();

        std::vector<float> end;
        sim.DownloadPositions(end);
        int insideObstacle = 0;
        for (size_t i = 0; i < end.size() / 4; ++i)
        {
            glm::vec3 p(end[i * 4], end[i * 4 + 1], end[i * 4 + 2]);
            glm::vec3 q = glm::max(box.boxMin - p, p - box.boxMax);
            bool inBox = std::max(q.x, std::max(q.y, q.z)) < -1e-3f;
            bool inBall = glm::length(p - ball.center) < ball.radius - 1e-3f;
            insideObstacle += (inBox || inBall) ? 1 : 0;
        }
        CHECK(insideObstacle == 0);
        sim.Shutdown();
    }
}

void RunSolverTests()
{
    TestRestDensity();
    TestCompressedBlockRelaxes();
    TestHydrostaticColumn();
    TestObstacleRespected();
}
