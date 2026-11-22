#include "pch.h"
#include <algorithm>
#include <array>
#include <cmath>
#include <random>
#include <glad/glad.h>

#include "PbfGpuTests.h"
#include "TestFramework.h"
#include "pbf/GpuScan.h"
#include "pbf/PbfSim.h"

namespace
{
    std::vector<unsigned int> CpuExclusiveScan(const std::vector<unsigned int>& in)
    {
        std::vector<unsigned int> out(in.size());
        unsigned int sum = 0;
        for (size_t i = 0; i < in.size(); ++i)
        {
            out[i] = sum;
            sum += in[i];
        }
        return out;
    }

    void TestScanSize(Pbf::GpuScan& scan, unsigned int count, std::mt19937& rng)
    {
        std::uniform_int_distribution<unsigned int> dist(0, 9);
        std::vector<unsigned int> data(count);
        for (unsigned int& v : data)
            v = dist(rng);

        GpuBuffer in;
        GpuBuffer out;
        in.Resize(count * sizeof(unsigned int));
        out.Resize(count * sizeof(unsigned int));
        in.Upload(data.data(), count * sizeof(unsigned int));
        scan.Run(in, out, count);

        std::vector<unsigned int> gpu(count);
        out.Download(gpu.data(), count * sizeof(unsigned int));
        bool match = gpu == CpuExclusiveScan(data);
        if (!match)
            std::cerr << "  scan mismatch at count " << count << "\n";
        CHECK(match);
    }

    void TestScan()
    {
        Pbf::GpuScan scan;
        CHECK(scan.Init(TRL_SHADER_DIR));
        if (!scan.IsReady())
            return;
        std::mt19937 rng(7);
        for (unsigned int count : {1u, 7u, 511u, 512u, 513u, 4099u, 100000u, 300001u})
            TestScanSize(scan, count, rng);
        scan.Destroy();
    }

    PbfBinding GridBinding()
    {
        PbfBinding b;
        b.enabled = true;
        b.particleRadius = 0.02f;
        b.domainMin = glm::vec3(0.0f, 0.0f, 0.0f);
        b.domainMax = glm::vec3(1.0f, 0.6f, 0.5f);
        b.maxParticles = 4096;
        return b;
    }

    std::vector<Pbf::Particle> TestParticles(const PbfBinding& b, float h)
    {
        std::mt19937 rng(11);
        std::uniform_real_distribution<float> ux(b.domainMin.x, b.domainMax.x);
        std::uniform_real_distribution<float> uy(b.domainMin.y, b.domainMax.y);
        std::uniform_real_distribution<float> uz(b.domainMin.z, b.domainMax.z);
        std::vector<Pbf::Particle> ps;
        auto add = [&](float x, float y, float z)
        {
            Pbf::Particle p;
            p.position[0] = x;
            p.position[1] = y;
            p.position[2] = z;
            ps.push_back(p);
        };
        for (int i = 0; i < 2500; ++i)
            add(ux(rng), uy(rng), uz(rng));
        for (int i = 0; i <= 6; ++i)
            for (int k = 0; k <= 3; ++k)
                add(h * i, 0.3f, h * k + 0.01f);
        add(b.domainMin.x, b.domainMin.y, b.domainMin.z);
        add(b.domainMax.x, b.domainMax.y, b.domainMax.z);
        add(b.domainMax.x, b.domainMin.y, b.domainMax.z);
        add(b.domainMin.x, b.domainMax.y, b.domainMin.z);
        return ps;
    }

    void TestGrid()
    {
        Pbf::PbfSim sim;
        sim.SetShaderDir(TRL_SHADER_DIR);
        PbfBinding binding = GridBinding();
        binding.seeds.push_back(PbfShapeBinding{});
        binding.seeds.back().boxMin = glm::vec3(0.0f);
        binding.seeds.back().boxMax = glm::vec3(0.1f);
        CHECK(sim.Load(binding));
        if (!sim.IsLoaded())
            return;

        float h = sim.SmoothingRadius();
        std::vector<Pbf::Particle> input = TestParticles(binding, h);
        sim.UploadParticles(input);
        CHECK(sim.ParticleCount() == static_cast<int>(input.size()));

        sim.BuildGrid();

        std::vector<unsigned int> counts;
        std::vector<unsigned int> starts;
        sim.DownloadCellCounts(counts);
        sim.DownloadCellStarts(starts);
        unsigned int total = 0;
        for (unsigned int c : counts)
            total += c;
        CHECK(total == input.size());
        CHECK(starts == CpuExclusiveScan(counts));

        std::vector<float> sorted;
        sim.DownloadPredicted(sorted);
        std::vector<std::array<float, 3>> a;
        std::vector<std::array<float, 3>> b;
        for (const Pbf::Particle& p : input)
            a.push_back({p.position[0], p.position[1], p.position[2]});
        for (size_t i = 0; i < input.size(); ++i)
            b.push_back({sorted[i * 4], sorted[i * 4 + 1], sorted[i * 4 + 2]});
        std::vector<std::array<float, 3>> sortedA = a;
        std::vector<std::array<float, 3>> sortedB = b;
        std::sort(sortedA.begin(), sortedA.end());
        std::sort(sortedB.begin(), sortedB.end());
        CHECK(sortedA == sortedB);

        const Pbf::ParamsGpu& params = sim.Params();
        int dims[3] = {params.gridDims[0], params.gridDims[1], params.gridDims[2]};
        float lo[3] = {binding.domainMin.x, binding.domainMin.y, binding.domainMin.z};
        int misplaced = 0;
        for (size_t cell = 0; cell < counts.size(); ++cell)
        {
            for (unsigned int s = starts[cell]; s < starts[cell] + counts[cell]; ++s)
            {
                int coord[3];
                bool nearEdge = false;
                for (int axis = 0; axis < 3; ++axis)
                {
                    float t = (b[s][axis] - lo[axis]) / h;
                    nearEdge = nearEdge || std::fabs(t - std::round(t)) < 1e-4f;
                    coord[axis] = std::clamp(static_cast<int>(std::floor(t)), 0, dims[axis] - 1);
                }
                size_t expected = static_cast<size_t>((coord[2] * dims[1] + coord[1]) * dims[0] + coord[0]);
                if (expected != cell && !nearEdge)
                    ++misplaced;
            }
        }
        CHECK(misplaced == 0);

        sim.ComputeNeighborStats();
        std::vector<unsigned int> gpuCounts;
        sim.DownloadNeighborCounts(gpuCounts);
        float h2 = h * h;
        int wrong = 0;
        for (size_t i = 0; i < b.size(); ++i)
        {
            unsigned int strict = 0;
            unsigned int loose = 0;
            for (size_t j = 0; j < b.size(); ++j)
            {
                if (i == j)
                    continue;
                float dx = b[i][0] - b[j][0];
                float dy = b[i][1] - b[j][1];
                float dz = b[i][2] - b[j][2];
                float d2 = dx * dx + dy * dy + dz * dz;
                strict += d2 < h2 * (1.0f - 1e-5f) ? 1u : 0u;
                loose += d2 < h2 * (1.0f + 1e-5f) ? 1u : 0u;
            }
            if (gpuCounts[i] < strict || gpuCounts[i] > loose)
                ++wrong;
        }
        if (wrong > 0)
            std::cerr << "  " << wrong << " particles with wrong neighbor counts\n";
        CHECK(wrong == 0);
        CHECK(sim.Neighbors().maximum > 0);

        sim.BuildGrid();
        std::vector<float> again;
        sim.DownloadPredicted(again);
        CHECK(again.size() == sorted.size());

        sim.Shutdown();
        CHECK(glGetError() == GL_NO_ERROR);
    }
}

void RunGridTests()
{
    TestScan();
    TestGrid();
}
