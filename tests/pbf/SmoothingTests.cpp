#include "pch.h"
#include <cmath>
#include <random>
#include <vector>
#include <glad/glad.h>
#include <glm/gtc/matrix_transform.hpp>

#include "PbfGpuTests.h"
#include "TestFramework.h"
#include "graphics/resources/Shader.h"

namespace
{
    constexpr int kSize = 64;

    GLuint MakeTexture(GLenum internalFormat, const std::vector<float>* data, int w = kSize, int h = kSize)
    {
        GLuint tex = 0;
        glGenTextures(1, &tex);
        glBindTexture(GL_TEXTURE_2D, tex);
        GLenum format = internalFormat == GL_RGBA16F ? GL_RGBA : GL_RED;
        glTexImage2D(GL_TEXTURE_2D, 0, internalFormat, w, h, 0, format, GL_FLOAT, data ? data->data() : nullptr);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
        glBindTexture(GL_TEXTURE_2D, 0);
        return tex;
    }

    std::vector<float> Read(GLuint tex, int channels, int w = kSize, int h = kSize)
    {
        std::vector<float> out(static_cast<size_t>(w) * h * channels);
        glBindTexture(GL_TEXTURE_2D, tex);
        glGetTexImage(GL_TEXTURE_2D, 0, channels == 4 ? GL_RGBA : GL_RED, GL_FLOAT, out.data());
        glBindTexture(GL_TEXTURE_2D, 0);
        return out;
    }

    void Bind(GLuint in, GLuint out, GLenum format)
    {
        glBindImageTexture(0, in, 0, GL_FALSE, 0, GL_READ_ONLY, format);
        glBindImageTexture(1, out, 0, GL_FALSE, 0, GL_WRITE_ONLY, format);
    }

    glm::mat4 Projection()
    {
        return glm::perspective(glm::radians(60.0f), 1.0f, 0.1f, 10.0f);
    }

    std::vector<float> NoisyPlane(float z, float noise, unsigned seed)
    {
        std::mt19937 rng(seed);
        std::uniform_real_distribution<float> u(-noise, noise);
        std::vector<float> depth(kSize * kSize);
        for (float& d : depth)
            d = z + u(rng);
        return depth;
    }

    double InteriorVariance(const std::vector<float>& depth, float z, int border)
    {
        double sum = 0.0;
        int n = 0;
        for (int y = border; y < kSize - border; ++y)
            for (int x = border; x < kSize - border; ++x)
            {
                double d = depth[y * kSize + x] - z;
                sum += d * d;
                ++n;
            }
        return sum / n;
    }

    void TestCurvatureFlow()
    {
        Shader shader;
        bool ok = shader.LoadComputeFromFile(ShaderPath("pbf_curvature_flow.comp"));
        CHECK(ok);
        if (!ok)
            return;

        const float z = 2.0f;
        std::vector<float> depth = NoisyPlane(z, 0.004f, 5);
        for (int y = 0; y < kSize; ++y)
            for (int x = 0; x < 8; ++x)
                depth[y * kSize + x] = 0.0f;
        depth[5 * kSize + 30] = z + 0.5f;

        GLuint tex[2] = {MakeTexture(GL_R32F, &depth), MakeTexture(GL_R32F, nullptr)};
        glm::mat4 proj = Projection();
        shader.Bind();
        shader.SetFloat("uCx", 2.0f / (kSize * proj[0][0]));
        shader.SetFloat("uCy", 2.0f / (kSize * proj[1][1]));
        shader.SetFloat("uStep", 0.3f);
        shader.SetFloat("uEdge", 0.1f);
        int src = 0;
        for (int i = 0; i < 50; ++i)
        {
            Bind(tex[src], tex[1 - src], GL_R32F);
            shader.Dispatch(kSize / 16, kSize / 16);
            src = 1 - src;
        }
        std::vector<float> out = Read(tex[src], 1);

        double before = InteriorVariance(depth, z, 12);
        double after = InteriorVariance(out, z, 12);
        bool backgroundKept = true;
        bool edgeFrozen = true;
        bool finite = true;
        for (int y = 0; y < kSize; ++y)
        {
            backgroundKept = backgroundKept && out[y * kSize + 3] == 0.0f;
            edgeFrozen = edgeFrozen && out[y * kSize + 8] == depth[y * kSize + 8];
        }
        for (float v : out)
            finite = finite && std::isfinite(v);
        float spike = out[5 * kSize + 30];
        std::printf("[PBF] curvature flow: noise variance %.3e -> %.3e, residual rms %.2e m, spike %.3f (kept)\n",
                    before,
                    after,
                    std::sqrt(after),
                    spike);
        CHECK(finite);
        CHECK(after < 0.2 * before);
        CHECK(backgroundKept);
        CHECK(edgeFrozen);
        CHECK_NEAR(spike, z + 0.5f, 1e-6f);

        std::vector<float> flat(kSize * kSize, z);
        glBindTexture(GL_TEXTURE_2D, tex[0]);
        glTexSubImage2D(GL_TEXTURE_2D, 0, 0, 0, kSize, kSize, GL_RED, GL_FLOAT, flat.data());
        glBindTexture(GL_TEXTURE_2D, 0);
        shader.Bind();
        Bind(tex[0], tex[1], GL_R32F);
        shader.Dispatch(kSize / 16, kSize / 16);
        std::vector<float> flatOut = Read(tex[1], 1);
        float worst = 0.0f;
        for (float v : flatOut)
            worst = std::max(worst, std::fabs(v - z));
        CHECK(worst < 1e-6f);

        glDeleteTextures(2, tex);
    }

    void TestBilateral()
    {
        Shader shader;
        bool ok = shader.LoadComputeFromFile(ShaderPath("pbf_bilateral.comp"));
        CHECK(ok);
        if (!ok)
            return;

        std::vector<float> depth = NoisyPlane(2.0f, 0.004f, 9);
        for (int y = 0; y < kSize; ++y)
            for (int x = kSize / 2; x < kSize; ++x)
                depth[y * kSize + x] += 1.0f;

        GLuint tex[2] = {MakeTexture(GL_R32F, &depth), MakeTexture(GL_R32F, nullptr)};
        glm::mat4 proj = Projection();
        shader.Bind();
        shader.SetFloat("uWorldRadius", 0.05f);
        shader.SetFloat("uPixelsPerMeter", 0.5f * proj[1][1] * kSize);
        shader.SetFloat("uDepthFalloff", 0.02f);
        for (int pass = 0; pass < 2; ++pass)
        {
            Bind(tex[pass], tex[1 - pass], GL_R32F);
            shader.SetVec2("uDirection", pass == 0 ? glm::vec2(1.0f, 0.0f) : glm::vec2(0.0f, 1.0f));
            shader.Dispatch(kSize / 16, kSize / 16);
        }
        std::vector<float> out = Read(tex[0], 1);

        double before = 0.0;
        double after = 0.0;
        float edgeWorst = 0.0f;
        for (int y = 8; y < kSize - 8; ++y)
            for (int x = 4; x < kSize / 2 - 4; ++x)
            {
                before += (depth[y * kSize + x] - 2.0) * (depth[y * kSize + x] - 2.0);
                after += (out[y * kSize + x] - 2.0) * (out[y * kSize + x] - 2.0);
            }
        for (int y = 0; y < kSize; ++y)
            edgeWorst = std::max(edgeWorst, std::fabs(out[y * kSize + kSize / 2 - 1] - 2.0f));
        std::printf("[PBF] bilateral: noise %.3e -> %.3e, worst deviation next to the 1 m step %.4f\n",
                    before,
                    after,
                    edgeWorst);
        CHECK(after < 0.5 * before);
        CHECK(edgeWorst < 0.02f);
        glDeleteTextures(2, tex);
    }

    void TestNormals()
    {
        Shader shader;
        bool ok = shader.LoadComputeFromFile(ShaderPath("pbf_normals.comp"));
        CHECK(ok);
        if (!ok)
            return;

        glm::mat4 proj = Projection();
        glm::vec3 n = glm::normalize(glm::vec3(0.3f, 0.5f, 1.0f));
        glm::vec3 point(0.0f, 0.0f, -2.0f);
        float d = glm::dot(n, point);
        std::vector<float> depth(kSize * kSize);
        for (int y = 0; y < kSize; ++y)
            for (int x = 0; x < kSize; ++x)
            {
                glm::vec2 ndc = (glm::vec2(x, y) + 0.5f) / static_cast<float>(kSize) * 2.0f - 1.0f;
                glm::vec3 ray(ndc.x / proj[0][0], ndc.y / proj[1][1], -1.0f);
                depth[y * kSize + x] = d / glm::dot(n, ray);
            }
        for (int x = 40; x < kSize; ++x)
            depth[30 * kSize + x] = 0.0f;

        GLuint depthTex = MakeTexture(GL_R32F, &depth);
        GLuint normalTex = MakeTexture(GL_RGBA16F, nullptr);
        glm::mat3 invViewRot = glm::mat3(glm::rotate(glm::mat4(1.0f), glm::radians(90.0f), glm::vec3(0.0f, 1.0f, 0.0f)));
        shader.Bind();
        shader.SetVec2("uInvProjScale", glm::vec2(1.0f / proj[0][0], 1.0f / proj[1][1]));
        shader.SetMat3("uInvViewRot", invViewRot);
        glBindImageTexture(0, depthTex, 0, GL_FALSE, 0, GL_READ_ONLY, GL_R32F);
        glBindImageTexture(1, normalTex, 0, GL_FALSE, 0, GL_WRITE_ONLY, GL_RGBA16F);
        shader.Dispatch(kSize / 16, kSize / 16);
        std::vector<float> out = Read(normalTex, 4);

        glm::vec3 expected = invViewRot * n;
        float worst = 1.0f;
        for (int y = 0; y < kSize; ++y)
            for (int x = 0; x < kSize; ++x)
            {
                if (depth[y * kSize + x] <= 0.0f)
                {
                    CHECK(out[(y * kSize + x) * 4 + 3] == 0.0f);
                    continue;
                }
                const float* v = &out[(y * kSize + x) * 4];
                worst = std::min(worst, glm::dot(glm::vec3(v[0], v[1], v[2]), expected));
            }
        std::printf("[PBF] normals: worst dot with the plane normal %.5f (holes next to background included)\n", worst);
        CHECK(worst > 0.999f);
        glDeleteTextures(1, &depthTex);
        glDeleteTextures(1, &normalTex);
    }

    void TestThicknessBlur()
    {
        Shader shader;
        bool ok = shader.LoadComputeFromFile(ShaderPath("pbf_thickness_blur.comp"));
        CHECK(ok);
        if (!ok)
            return;

        std::vector<float> thick(kSize * kSize, 0.0f);
        thick[32 * kSize + 32] = 1.0f;
        GLuint tex[2] = {MakeTexture(GL_R16F, &thick), MakeTexture(GL_R16F, nullptr)};
        shader.Bind();
        Bind(tex[0], tex[1], GL_R16F);
        shader.SetVec2("uDirection", glm::vec2(1.0f, 0.0f));
        shader.Dispatch(kSize / 16, kSize / 16);
        Bind(tex[1], tex[0], GL_R16F);
        shader.SetVec2("uDirection", glm::vec2(0.0f, 1.0f));
        shader.Dispatch(kSize / 16, kSize / 16);
        std::vector<float> out = Read(tex[0], 1);
        double sum = 0.0;
        for (float v : out)
            sum += v;
        float peak = out[32 * kSize + 32];
        std::printf("[PBF] thickness blur: sum %.4f (expected 1), peak %.4f\n", sum, peak);
        CHECK_NEAR(sum, 1.0, 0.01);
        CHECK(peak < 0.06f);
        CHECK(std::fabs(out[32 * kSize + 33] - out[32 * kSize + 31]) < 1e-3f);
        glDeleteTextures(2, tex);
    }
}

void RunSmoothingTests()
{
    TestCurvatureFlow();
    TestBilateral();
    TestNormals();
    TestThicknessBlur();
    glBindImageTexture(0, 0, 0, GL_FALSE, 0, GL_READ_ONLY, GL_R32F);
    glBindImageTexture(1, 0, 0, GL_FALSE, 0, GL_WRITE_ONLY, GL_R32F);
    CHECK(glGetError() == GL_NO_ERROR);
}
