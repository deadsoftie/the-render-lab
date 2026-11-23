#include "pch.h"
#include <algorithm>
#include <array>
#include <cmath>
#include <glad/glad.h>

#include "PbfGpuTests.h"
#include "TestFramework.h"
#include "pbf/PbfSim.h"

namespace
{
    constexpr float kRadius = 0.0125f;

    PbfBinding Domain()
    {
        PbfBinding b;
        b.enabled = true;
        b.particleRadius = kRadius;
        b.domainMin = glm::vec3(0.0f);
        b.domainMax = glm::vec3(0.6f);
        b.maxParticles = 20000;
        return b;
    }

    std::vector<Pbf::Particle> Block()
    {
        std::vector<Pbf::Particle> ps;
        float spacing = 0.9f * 2.0f * kRadius;
        for (int k = 0; k < 14; ++k)
            for (int j = 0; j < 14; ++j)
                for (int i = 0; i < 14; ++i)
                {
                    Pbf::Particle p;
                    p.position[0] = 0.1f + i * spacing;
                    p.position[1] = 0.05f + j * spacing;
                    p.position[2] = 0.1f + k * spacing;
                    ps.push_back(p);
                }
        return ps;
    }

    bool Start(Pbf::PbfSim& sim, int groupSize, bool listed)
    {
        sim.SetShaderDir(TRL_SHADER_DIR);
        CHECK(sim.SetWorkgroupSize(groupSize));
        CHECK(sim.Load(Domain()));
        if (!sim.IsLoaded())
            return false;
        CHECK(sim.WorkgroupSize() == groupSize);
        sim.settings.gravity[1] = 0.0f;
        sim.settings.neighborList = listed;
        sim.UploadParticles(Block());
        return true;
    }

    std::vector<std::array<float, 4>> SortedByPosition(const std::vector<float>& data)
    {
        std::vector<std::array<float, 4>> rows(data.size() / 4);
        for (size_t i = 0; i < rows.size(); ++i)
            rows[i] = {data[i * 4], data[i * 4 + 1], data[i * 4 + 2], data[i * 4 + 3]};
        std::sort(rows.begin(), rows.end());
        return rows;
    }

    float NearestMatchError(const std::vector<float>& a, const std::vector<float>& b)
    {
        float worst = 0.0f;
        for (size_t i = 0; i + 3 < a.size(); i += 4)
        {
            float best = 1e30f;
            for (size_t j = 0; j + 3 < b.size(); j += 4)
            {
                float dx = a[i] - b[j];
                float dy = a[i + 1] - b[j + 1];
                float dz = a[i + 2] - b[j + 2];
                best = std::min(best, dx * dx + dy * dy + dz * dz);
            }
            worst = std::max(worst, std::sqrt(best));
        }
        return worst;
    }

    void TestNeighborListMatchesCellWalk()
    {
        Pbf::PbfSim walk;
        Pbf::PbfSim listed;
        if (!Start(walk, 256, false) || !Start(listed, 256, false))
            return;

        listed.BuildGrid();
        listed.ComputeLambda();
        std::vector<float> a, b, la, lb;
        listed.DownloadPredicted(a);
        listed.DownloadVelocities(la);
        listed.settings.neighborList = true;
        listed.BuildNeighborList();
        listed.ComputeLambda();
        listed.DownloadPredicted(b);
        listed.DownloadVelocities(lb);
        CHECK(a.size() == b.size());
        float densityError = 0.0f;
        float lambdaError = 0.0f;
        float lambdaScale = 0.0f;
        for (size_t i = 0; i + 3 < a.size() && i + 3 < b.size(); i += 4)
        {
            densityError = std::max(densityError, std::fabs(a[i + 3] - b[i + 3]) / 1000.0f);
            lambdaError = std::max(lambdaError, std::fabs(la[i + 3] - lb[i + 3]));
            lambdaScale = std::max(lambdaScale, std::fabs(la[i + 3]));
        }

        walk.BuildGrid();
        walk.ComputeLambda();
        walk.ApplyDelta();
        listed.ApplyDelta();
        walk.DownloadPredicted(a);
        listed.DownloadPredicted(b);
        CHECK(a.size() == b.size());
        float positionError = NearestMatchError(b, a);

        listed.ComputeNeighborStats();
        std::printf("[PBF] neighbor list vs cell walk: density %.2e, lambda %.2e (max |lambda| %.2e), "
                    "position after delta %.2e m, dropped %u, avg neighbors %.1f\n",
                    densityError,
                    lambdaError,
                    lambdaScale,
                    positionError,
                    listed.Neighbors().listDropped,
                    listed.Neighbors().average);
        CHECK(lambdaScale > 0.0f);
        CHECK(densityError < 1e-4f);
        CHECK(lambdaError < 1e-4f * std::max(lambdaScale, 1e-6f) + 1e-9f);
        CHECK(positionError < 1e-6f);
        CHECK(listed.Neighbors().listDropped == 0);
        walk.Shutdown();
        listed.Shutdown();
    }

    void TestWorkgroupSizesAgree()
    {
        std::vector<std::array<float, 4>> reference;
        for (int size : {256, 128, 64})
        {
            Pbf::PbfSim sim;
            if (!Start(sim, size, false))
                return;
            sim.BuildGrid();
            sim.ComputeLambda();
            std::vector<float> predicted;
            sim.DownloadPredicted(predicted);
            std::vector<std::array<float, 4>> rows = SortedByPosition(predicted);
            if (reference.empty())
            {
                reference = rows;
            }
            else
            {
                CHECK(rows.size() == reference.size());
                float worst = 0.0f;
                for (size_t i = 0; i < rows.size() && i < reference.size(); ++i)
                    worst = std::max(worst, std::fabs(rows[i][3] - reference[i][3]) / 1000.0f);
                std::printf("[PBF] workgroup %d vs 256: max density difference %.2e\n", size, worst);
                CHECK(worst < 1e-4f);
            }
            sim.Shutdown();
        }
    }
}

void RunPerformanceTests()
{
    TestNeighborListMatchesCellWalk();
    TestWorkgroupSizesAgree();
}
