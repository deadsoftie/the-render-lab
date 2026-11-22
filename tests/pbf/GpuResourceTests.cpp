#include "pch.h"
#include <glad/glad.h>

#include "PbfGpuTests.h"
#include "TestFramework.h"
#include "graphics/resources/GpuBuffer.h"
#include "graphics/resources/GpuQueryTimer.h"
#include "graphics/resources/Shader.h"

void RunGpuBufferTests()
{
    GpuBuffer buffer;
    CHECK(buffer.Resize(1024 * sizeof(float)));
    CHECK(buffer.ID() != 0);
    CHECK(buffer.Capacity() == 1024 * sizeof(float));

    std::vector<float> data(1024);
    for (size_t i = 0; i < data.size(); ++i)
        data[i] = static_cast<float>(i) * 0.5f;
    buffer.Upload(data.data(), data.size() * sizeof(float));

    std::vector<float> back(1024, -1.0f);
    buffer.Download(back.data(), back.size() * sizeof(float));
    CHECK(back == data);

    std::vector<float> part(16, -1.0f);
    buffer.Download(part.data(), part.size() * sizeof(float), 100 * sizeof(float));
    CHECK(part[0] == 50.0f);
    CHECK(part[15] == 57.5f);

    unsigned int oldId = buffer.ID();
    CHECK(buffer.Resize(512 * sizeof(float)));
    CHECK(buffer.ID() == oldId);

    CHECK(buffer.Resize(4096 * sizeof(float), true));
    CHECK(buffer.Capacity() == 4096 * sizeof(float));
    std::vector<float> preserved(1024, -1.0f);
    buffer.Download(preserved.data(), preserved.size() * sizeof(float));
    CHECK(preserved == data);

    buffer.Clear();
    std::vector<float> cleared(4096, -1.0f);
    buffer.Download(cleared.data(), cleared.size() * sizeof(float));
    bool allZero = true;
    for (float v : cleared)
        allZero = allZero && v == 0.0f;
    CHECK(allZero);

    buffer.Destroy();
    CHECK(buffer.ID() == 0);
    CHECK(glGetError() == GL_NO_ERROR);
}

void RunGpuQueryTimerTests()
{
    GpuBuffer buffer;
    buffer.Resize(1 << 22);
    GpuQueryTimer timer;
    for (int frame = 0; frame < 4; ++frame)
    {
        timer.BeginFrame();
        timer.Begin("Clear");
        buffer.Clear();
        timer.End();
        timer.Begin("Clear again");
        buffer.Clear();
        timer.End();
        glFinish();
    }
    timer.BeginFrame();
    CHECK(timer.Stages().size() == 2);
    if (timer.Stages().size() == 2)
    {
        CHECK(timer.Stages()[0].name == "Clear");
        CHECK(timer.Stages()[0].ms >= 0.0f);
        CHECK(timer.TotalMs() >= timer.Stages()[1].ms);
    }
    timer.Destroy();
    CHECK(glGetError() == GL_NO_ERROR);
}

void RunPbfShaderCompileTests()
{
    Shader particles;
    CHECK(particles.LoadFromFiles(ShaderPath("pbf_particles.vert"), ShaderPath("pbf_particles.frag")));

    for (const char* name : {"pbf_cell_count.comp", "pbf_scan.comp", "pbf_scan_add.comp", "pbf_reorder.comp",
                             "pbf_neighbor_stats.comp"})
    {
        Shader compute;
        bool ok = compute.LoadComputeFromFile(ShaderPath(name));
        if (!ok)
            std::cerr << "  failed to compile " << name << "
";
        CHECK(ok);
    }
}
