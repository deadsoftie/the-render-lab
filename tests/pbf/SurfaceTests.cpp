#include "pch.h"
#include <vector>
#include <glad/glad.h>
#include <glm/gtc/matrix_transform.hpp>

#include "PbfGpuTests.h"
#include "TestFramework.h"
#include "graphics/render_targets/FluidSurfaceBuffer.h"
#include "graphics/resources/Shader.h"

namespace
{
    constexpr int kSize = 64;
    constexpr float kRadius = 0.1f;
    constexpr float kEyeDistance = 2.0f;

    struct SurfaceRig
    {
        Shader depth;
        Shader thickness;
        FluidSurfaceBuffer buffer;
        GLuint sceneDepth = 0;
        GLuint vao = 0;
        GLuint vbo = 0;
        glm::mat4 view{1.0f};
        glm::mat4 proj{1.0f};
        int points = 0;

        bool Init()
        {
            bool ok = depth.LoadFromFiles(ShaderPath("pbf_particles.vert"), ShaderPath("pbf_depth.frag")) &&
                      thickness.LoadFromFiles(ShaderPath("pbf_particles.vert"), ShaderPath("pbf_thickness.frag")) &&
                      buffer.Create(kSize, kSize);
            view = glm::lookAt(glm::vec3(0.0f, 0.0f, kEyeDistance), glm::vec3(0.0f), glm::vec3(0.0f, 1.0f, 0.0f));
            proj = glm::perspective(glm::radians(60.0f), 1.0f, 0.1f, 10.0f);
            glGenTextures(1, &sceneDepth);
            glBindTexture(GL_TEXTURE_2D, sceneDepth);
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
            glBindTexture(GL_TEXTURE_2D, 0);
            glGenVertexArrays(1, &vao);
            glGenBuffers(1, &vbo);
            glBindVertexArray(vao);
            glBindBuffer(GL_ARRAY_BUFFER, vbo);
            glEnableVertexAttribArray(0);
            glVertexAttribPointer(0, 4, GL_FLOAT, GL_FALSE, 4 * sizeof(float), nullptr);
            glBindVertexArray(0);
            glBindBuffer(GL_ARRAY_BUFFER, 0);
            return ok;
        }

        void SetPoints(int count)
        {
            std::vector<float> data(static_cast<size_t>(count) * 4, 0.0f);
            for (int i = 0; i < count; ++i)
                data[i * 4 + 3] = 1.0f;
            glBindBuffer(GL_ARRAY_BUFFER, vbo);
            glBufferData(GL_ARRAY_BUFFER, static_cast<GLsizeiptr>(data.size() * sizeof(float)), data.data(), GL_STATIC_DRAW);
            glBindBuffer(GL_ARRAY_BUFFER, 0);
            points = count;
        }

        void SetSceneDepth(float eyeDistance)
        {
            float value = 1.0f;
            if (eyeDistance > 0.0f)
            {
                glm::vec4 clip = proj * glm::vec4(0.0f, 0.0f, -eyeDistance, 1.0f);
                value = clip.z / clip.w * 0.5f + 0.5f;
            }
            std::vector<float> data(kSize * kSize, value);
            glBindTexture(GL_TEXTURE_2D, sceneDepth);
            glTexImage2D(GL_TEXTURE_2D, 0, GL_DEPTH_COMPONENT32F, kSize, kSize, 0, GL_DEPTH_COMPONENT, GL_FLOAT, data.data());
            glBindTexture(GL_TEXTURE_2D, 0);
        }

        void Setup(Shader& shader, int viewportW, int viewportH)
        {
            shader.Bind();
            shader.SetMat4("uView", view);
            shader.SetMat4("uProj", proj);
            shader.SetFloat("uRadius", kRadius);
            shader.SetFloat("uThicknessScale", 1.0f);
            shader.SetFloat("uViewportHeight", static_cast<float>(viewportH));
            shader.SetVec2("uViewport", glm::vec2(viewportW, viewportH));
            shader.SetInt("uSceneDepth", 0);
        }

        void Render()
        {
            const float zero[4] = {0.0f, 0.0f, 0.0f, 0.0f};
            const float one = 1.0f;
            glActiveTexture(GL_TEXTURE0);
            glBindTexture(GL_TEXTURE_2D, sceneDepth);
            glEnable(GL_PROGRAM_POINT_SIZE);
            glBindVertexArray(vao);

            buffer.BindDepthForWriting();
            glClearBufferfv(GL_COLOR, 0, zero);
            glClearBufferfv(GL_DEPTH, 0, &one);
            glEnable(GL_DEPTH_TEST);
            glDepthMask(GL_TRUE);
            glDisable(GL_BLEND);
            Setup(depth, buffer.Width(), buffer.Height());
            glDrawArrays(GL_POINTS, 0, points);

            buffer.BindThicknessForWriting();
            glClearBufferfv(GL_COLOR, 0, zero);
            glDisable(GL_DEPTH_TEST);
            glDepthMask(GL_FALSE);
            glEnable(GL_BLEND);
            glBlendFunc(GL_ONE, GL_ONE);
            Setup(thickness, buffer.ThicknessWidth(), buffer.ThicknessHeight());
            glDrawArrays(GL_POINTS, 0, points);

            glDisable(GL_BLEND);
            glDepthMask(GL_TRUE);
            glBindVertexArray(0);
            FluidSurfaceBuffer::UnbindWriting();
            glFinish();
        }

        std::vector<float> Read(GLuint tex, int w, int h) const
        {
            std::vector<float> out(static_cast<size_t>(w) * h);
            glBindTexture(GL_TEXTURE_2D, tex);
            glGetTexImage(GL_TEXTURE_2D, 0, GL_RED, GL_FLOAT, out.data());
            glBindTexture(GL_TEXTURE_2D, 0);
            return out;
        }

        void Destroy()
        {
            buffer.Destroy();
            glDeleteTextures(1, &sceneDepth);
            glDeleteBuffers(1, &vbo);
            glDeleteVertexArrays(1, &vao);
        }
    };
}

void RunSurfaceTests()
{
    SurfaceRig rig;
    bool ok = rig.Init();
    CHECK(ok);
    if (!ok)
        return;
    CHECK(rig.buffer.ThicknessWidth() == kSize / 2);
    CHECK(rig.buffer.ThicknessHeight() == kSize / 2);

    rig.SetPoints(1);
    rig.SetSceneDepth(0.0f);
    rig.Render();
    std::vector<float> eye = rig.Read(rig.buffer.TexEyeDepth(0), kSize, kSize);
    std::vector<float> thick = rig.Read(rig.buffer.TexThickness(), kSize / 2, kSize / 2);
    float centerEye = eye[(kSize / 2) * kSize + kSize / 2];
    float cornerEye = eye[0];
    float centerThick = thick[(kSize / 4) * (kSize / 2) + kSize / 4];
    float cornerThick = thick[0];
    std::printf("[PBF] surface: center eye depth %.4f (expected %.4f), corner %.4f, center thickness %.4f (expected about %.4f)\n",
                centerEye,
                kEyeDistance - kRadius,
                cornerEye,
                centerThick,
                2.0f * kRadius);
    CHECK_NEAR(centerEye, kEyeDistance - kRadius, 0.01f);
    CHECK(cornerEye == 0.0f);
    CHECK_NEAR(centerThick, 2.0f * kRadius, 0.03f);
    CHECK(cornerThick == 0.0f);

    rig.SetPoints(3);
    rig.Render();
    thick = rig.Read(rig.buffer.TexThickness(), kSize / 2, kSize / 2);
    float stacked = thick[(kSize / 4) * (kSize / 2) + kSize / 4];
    std::printf("[PBF] surface: three stacked sprites thickness %.4f (one sprite %.4f)\n", stacked, centerThick);
    CHECK_NEAR(stacked, 3.0f * centerThick, 0.01f);

    rig.SetPoints(1);
    rig.SetSceneDepth(kEyeDistance - 0.5f);
    rig.Render();
    eye = rig.Read(rig.buffer.TexEyeDepth(0), kSize, kSize);
    thick = rig.Read(rig.buffer.TexThickness(), kSize / 2, kSize / 2);
    float occludedEye = eye[(kSize / 2) * kSize + kSize / 2];
    float occludedThick = thick[(kSize / 4) * (kSize / 2) + kSize / 4];
    std::printf("[PBF] surface: behind geometry eye depth %.4f, thickness %.4f\n", occludedEye, occludedThick);
    CHECK(occludedEye == 0.0f);
    CHECK(occludedThick == 0.0f);

    rig.SetSceneDepth(kEyeDistance + 0.5f);
    rig.Render();
    eye = rig.Read(rig.buffer.TexEyeDepth(0), kSize, kSize);
    CHECK_NEAR(eye[(kSize / 2) * kSize + kSize / 2], kEyeDistance - kRadius, 0.01f);

    CHECK(rig.buffer.Resize(kSize * 2, kSize));
    CHECK(rig.buffer.Width() == kSize * 2);
    CHECK(rig.buffer.ThicknessWidth() == kSize);
    rig.Destroy();
    CHECK(glGetError() == GL_NO_ERROR);
}
