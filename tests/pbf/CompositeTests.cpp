#include "pch.h"
#include <cmath>
#include <vector>
#include <glad/glad.h>
#include <glm/gtc/matrix_transform.hpp>

#include "PbfGpuTests.h"
#include "TestFramework.h"
#include "graphics/resources/Shader.h"

namespace
{
    constexpr int kSize = 64;
    constexpr float kEyeDepth = 2.0f;

    GLuint Texture(GLenum internalFormat, GLenum format, GLenum type, const void* data, GLenum filter = GL_NEAREST)
    {
        GLuint tex = 0;
        glGenTextures(1, &tex);
        glBindTexture(GL_TEXTURE_2D, tex);
        glTexImage2D(GL_TEXTURE_2D, 0, internalFormat, kSize, kSize, 0, format, type, data);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, filter);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, filter);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
        glBindTexture(GL_TEXTURE_2D, 0);
        return tex;
    }

    GLuint Quad(GLuint& vbo)
    {
        const float verts[] = {-1, -1, 0, 0, 1, -1, 1, 0, 1, 1, 1, 1, -1, -1, 0, 0, 1, 1, 1, 1, -1, 1, 0, 1};
        GLuint vao = 0;
        glGenVertexArrays(1, &vao);
        glGenBuffers(1, &vbo);
        glBindVertexArray(vao);
        glBindBuffer(GL_ARRAY_BUFFER, vbo);
        glBufferData(GL_ARRAY_BUFFER, sizeof(verts), verts, GL_STATIC_DRAW);
        glEnableVertexAttribArray(0);
        glVertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE, 4 * sizeof(float), nullptr);
        glEnableVertexAttribArray(1);
        glVertexAttribPointer(1, 2, GL_FLOAT, GL_FALSE, 4 * sizeof(float), reinterpret_cast<void*>(2 * sizeof(float)));
        glBindVertexArray(0);
        glBindBuffer(GL_ARRAY_BUFFER, 0);
        return vao;
    }

    bool InFluid(int x, int y)
    {
        return x >= 16 && x < 48 && y >= 16 && y < 48;
    }

    float WindowDepth(const glm::mat4& proj, float eyeDepth)
    {
        glm::vec4 clip = proj * glm::vec4(0.0f, 0.0f, -eyeDepth, 1.0f);
        return clip.z / clip.w * 0.5f + 0.5f;
    }

    glm::vec3 ExpectedComposite(const glm::mat4& proj,
                                int px,
                                int py,
                                float thickness,
                                float sceneByte,
                                glm::vec3 color,
                                float absorption,
                                float scatter,
                                float exposure)
    {
        glm::vec2 uv((px + 0.5f) / kSize, (py + 0.5f) / kSize);
        glm::vec2 ndc = uv * 2.0f - 1.0f;
        glm::vec3 viewPos(ndc.x / proj[0][0] * kEyeDepth, ndc.y / proj[1][1] * kEyeDepth, -kEyeDepth);
        glm::vec3 V = glm::normalize(-viewPos);
        glm::vec3 N(0.0f, 0.0f, 1.0f);
        glm::vec3 scene(std::pow(sceneByte / 255.0f, 2.2f));
        glm::vec3 T = glm::exp(-absorption * thickness * (glm::vec3(1.0f) - color));
        glm::vec3 refracted = scene * T + color * scatter * (glm::vec3(1.0f) - T);
        float NdotV = std::max(glm::dot(N, V), 1e-4f);
        float fresnel = 0.02f + 0.98f * std::pow(1.0f - NdotV, 5.0f);
        glm::vec3 R = glm::reflect(-V, N);
        glm::vec3 env = glm::mix(glm::vec3(0.35f, 0.38f, 0.42f),
                                 glm::vec3(0.55f, 0.7f, 0.95f),
                                 glm::clamp(R.y * 0.5f + 0.5f, 0.0f, 1.0f));
        glm::vec3 e = exposure * env;
        glm::vec3 reflected = e / (e + glm::vec3(1.0f));
        glm::vec3 c = glm::mix(refracted, reflected, fresnel);
        return glm::pow(glm::clamp(c, 0.0f, 1.0f), glm::vec3(1.0f / 2.2f));
    }
}

void RunCompositeTests()
{
    Shader composite;
    Shader merge;
    bool ok = composite.LoadFromFiles(ShaderPath("deferred_light.vert"), ShaderPath("pbf_composite.frag")) &&
              merge.LoadFromFiles(ShaderPath("deferred_light.vert"), ShaderPath("pbf_depth_merge.frag"));
    CHECK(ok);
    if (!ok)
        return;

    const float thickness = 0.1f;
    const unsigned char sceneByte = 128;
    const glm::vec3 color(0.15f, 0.4f, 0.75f);
    const float absorption = 10.0f;
    const float scatter = 0.35f;
    const float exposure = 2.5f;
    glm::mat4 proj = glm::perspective(glm::radians(60.0f), 1.0f, 0.1f, 10.0f);
    glm::mat4 view(1.0f);

    std::vector<unsigned char> sceneColor(kSize * kSize * 4, sceneByte);
    std::vector<float> eye(kSize * kSize, 0.0f);
    std::vector<float> normals(kSize * kSize * 4, 0.0f);
    std::vector<float> thick(kSize * kSize, 0.0f);
    std::vector<float> sceneDepth(kSize * kSize, 1.0f);
    for (int y = 0; y < kSize; ++y)
        for (int x = 0; x < kSize; ++x)
        {
            if (!InFluid(x, y))
                continue;
            int i = y * kSize + x;
            eye[i] = kEyeDepth;
            normals[i * 4 + 2] = 1.0f;
            normals[i * 4 + 3] = 1.0f;
            thick[i] = thickness;
        }
    sceneDepth[0] = 0.25f;

    GLuint texScene = Texture(GL_RGBA8, GL_RGBA, GL_UNSIGNED_BYTE, sceneColor.data(), GL_LINEAR);
    GLuint texEye = Texture(GL_R32F, GL_RED, GL_FLOAT, eye.data());
    GLuint texNormal = Texture(GL_RGBA16F, GL_RGBA, GL_FLOAT, normals.data());
    GLuint texThick = Texture(GL_R16F, GL_RED, GL_FLOAT, thick.data());
    GLuint texDepth = Texture(GL_DEPTH_COMPONENT32F, GL_DEPTH_COMPONENT, GL_FLOAT, sceneDepth.data());
    GLuint texOut = Texture(GL_RGBA32F, GL_RGBA, GL_FLOAT, nullptr);

    GLuint fbo = 0;
    glGenFramebuffers(1, &fbo);
    glBindFramebuffer(GL_FRAMEBUFFER, fbo);
    glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, texOut, 0);
    CHECK(glCheckFramebufferStatus(GL_FRAMEBUFFER) == GL_FRAMEBUFFER_COMPLETE);
    glViewport(0, 0, kSize, kSize);
    const float clear[4] = {0.25f, 0.25f, 0.25f, 1.0f};
    glClearBufferfv(GL_COLOR, 0, clear);

    GLuint vbo = 0;
    GLuint vao = Quad(vbo);
    glDisable(GL_DEPTH_TEST);
    glDisable(GL_BLEND);
    glDepthMask(GL_FALSE);
    composite.Bind();
    const GLuint inputs[] = {texScene, texEye, texNormal, texThick, texDepth, 0};
    const char* names[] = {"uSceneColor", "uEyeDepth", "uNormals", "uThickness", "uSceneDepth", "uHDRITex"};
    for (int i = 0; i < 6; ++i)
    {
        glActiveTexture(GL_TEXTURE0 + i);
        glBindTexture(GL_TEXTURE_2D, inputs[i]);
        composite.SetInt(names[i], i);
    }
    composite.SetVec2("uInvProjScale", glm::vec2(1.0f / proj[0][0], 1.0f / proj[1][1]));
    composite.SetMat4("uView", view);
    composite.SetMat4("uInvView", glm::inverse(view));
    composite.SetMat4("uProj", proj);
    composite.SetVec3("uCamPos", glm::vec3(0.0f));
    composite.SetInt("uHasEnv", 0);
    composite.SetFloat("uHDRIRotation", 0.0f);
    composite.SetFloat("uExposure", exposure);
    composite.SetVec3("uColor", color);
    composite.SetFloat("uAbsorption", absorption);
    composite.SetFloat("uScatter", scatter);
    composite.SetFloat("uRefraction", 0.3f);
    composite.SetFloat("uRoughness", 0.06f);
    composite.SetVec3("uLightDir", glm::vec3(0.0f, 1.0f, 0.0f));
    composite.SetVec3("uLightColor", glm::vec3(0.0f));
    glBindVertexArray(vao);
    glDrawArrays(GL_TRIANGLES, 0, 6);

    for (int i = 0; i < 6; ++i)
    {
        glActiveTexture(GL_TEXTURE0 + i);
        glBindTexture(GL_TEXTURE_2D, 0);
    }
    glActiveTexture(GL_TEXTURE0);

    std::vector<float> out(kSize * kSize * 4);
    glBindTexture(GL_TEXTURE_2D, texOut);
    glGetTexImage(GL_TEXTURE_2D, 0, GL_RGBA, GL_FLOAT, out.data());
    glBindTexture(GL_TEXTURE_2D, 0);

    glm::vec3 expected = ExpectedComposite(proj, 32, 32, 0.0999755859f, sceneByte, color, absorption, scatter, exposure);
    glm::vec3 got(out[(32 * kSize + 32) * 4], out[(32 * kSize + 32) * 4 + 1], out[(32 * kSize + 32) * 4 + 2]);
    float sceneGamma = sceneByte / 255.0f;
    std::printf("[PBF] composite: center (%.4f, %.4f, %.4f), expected (%.4f, %.4f, %.4f), scene %.4f, outside %.4f\n",
                got.x,
                got.y,
                got.z,
                expected.x,
                expected.y,
                expected.z,
                sceneGamma,
                out[0]);
    CHECK(glm::length(got - expected) < 3e-3f);
    CHECK(got.z > got.x);
    CHECK(out[0] == 0.25f);
    CHECK(out[(5 * kSize + 60) * 4 + 1] == 0.25f);

    glFramebufferTexture2D(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, GL_TEXTURE_2D, texDepth, 0);
    glColorMask(GL_FALSE, GL_FALSE, GL_FALSE, GL_FALSE);
    glEnable(GL_DEPTH_TEST);
    glDepthFunc(GL_LESS);
    glDepthMask(GL_TRUE);
    merge.Bind();
    glBindTexture(GL_TEXTURE_2D, texEye);
    merge.SetInt("uEyeDepth", 0);
    merge.SetMat4("uProj", proj);
    glDrawArrays(GL_TRIANGLES, 0, 6);
    glColorMask(GL_TRUE, GL_TRUE, GL_TRUE, GL_TRUE);
    glFramebufferTexture2D(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, GL_TEXTURE_2D, 0, 0);
    glBindFramebuffer(GL_FRAMEBUFFER, 0);
    glBindTexture(GL_TEXTURE_2D, 0);

    std::vector<float> merged(kSize * kSize);
    glBindTexture(GL_TEXTURE_2D, texDepth);
    glGetTexImage(GL_TEXTURE_2D, 0, GL_DEPTH_COMPONENT, GL_FLOAT, merged.data());
    glBindTexture(GL_TEXTURE_2D, 0);
    float fluidDepth = WindowDepth(proj, kEyeDepth);
    std::printf("[PBF] depth merge: center %.6f (expected %.6f), empty %.3f, nearer geometry %.3f\n",
                merged[32 * kSize + 32],
                fluidDepth,
                merged[5 * kSize + 60],
                merged[0]);
    CHECK_NEAR(merged[32 * kSize + 32], fluidDepth, 1e-5f);
    CHECK(merged[5 * kSize + 60] == 1.0f);
    CHECK(merged[0] == 0.25f);

    glDisable(GL_DEPTH_TEST);
    glBindVertexArray(0);
    glDeleteBuffers(1, &vbo);
    glDeleteVertexArrays(1, &vao);
    glDeleteFramebuffers(1, &fbo);
    GLuint textures[] = {texScene, texEye, texNormal, texThick, texDepth, texOut};
    glDeleteTextures(6, textures);
    CHECK(glGetError() == GL_NO_ERROR);
}
