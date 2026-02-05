#include "pch.h"
#include "graphics/Renderer.h"
#include "graphics/Geometry.h"

#include <glad/glad.h>
#include <glm/ext/matrix_transform.hpp>

bool Renderer::Init()
{
    // Load shader
    if (!m_litShader.LoadFromFiles("assets/shaders/basic_lit.vert",
                                   "assets/shaders/basic_lit.frag"))
    {
        return false;
    }

    auto cube = Geometry::MakeCube(0.5f);
    m_cubeMesh.Create(cube.vertices, cube.indices);

    auto sphere = Geometry::MakeSphere(0.5f, 32, 16);
    m_sphereMesh.Create(sphere.vertices, sphere.indices);

    m_cornell = Geometry::MakeCornellBox({1.0f, 1.0f, 1.0f});
    m_cornellMesh.Create(m_cornell.mesh.vertices, m_cornell.mesh.indices);

    auto ground = Geometry::MakeGroundPlane(12.0f, -1.25f);
    m_groundMesh.Create(ground.vertices, ground.indices);

    // basic GL defaults for now
    glEnable(GL_DEPTH_TEST);

    m_ready = true;

    m_lightCount = 2;

    m_lights[0].position = {0.0f, 0.85f, -0.15f};
    m_lights[0].color = {1.0f, 1.0f, 1.0f};

    m_lights[1].position = {-0.55f, 0.45f, 0.35f};
    m_lights[1].color = {0.5f, 0.6f, 0.8f};

    m_mat.ambient = 0.02f;

    return true;
}

void Renderer::Shutdown()
{
    // Typically no-op: Mesh/Shader destructors clean up GL objects.
    m_ready = false;
}

void Renderer::SetViewport(int w, int h)
{
    m_viewportW = (w > 0) ? w : 1;
    m_viewportH = (h > 0) ? h : 1;
}

void Renderer::RenderFrame(const Camera& camera)
{
    if (!m_ready)
        return;

    // Viewport + clear
    glViewport(0, 0, m_viewportW, m_viewportH);
    glClearColor(0.1f, 0.12f, 0.15f, 1.0f);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    // Bind shader + set uniforms
    m_litShader.Bind();

    m_litShader.SetMat4("uModel", glm::mat4(1.0f));
    m_litShader.SetMat4("uView", camera.GetView());
    m_litShader.SetMat4("uProj", camera.GetProj());

    m_litShader.SetVec3("uCamPos", camera.GetPosition());

    m_litShader.SetVec3("uAlbedo", m_mat.albedo);
    m_litShader.SetFloat("uAmbient", m_mat.ambient);
    m_litShader.SetFloat("uShininess", m_mat.shininess);

    int count = glm::clamp(m_lightCount, 0, kMaxLights);
    m_litShader.SetInt("uLightCount", count);

    glm::vec3 pos[kMaxLights];
    glm::vec3 col[kMaxLights];

    for (int i = 0; i < count; ++i)
    {
        pos[i] = m_lights[i].position;

        if (m_lightEnabled[i])
            col[i] = m_lights[i].color * m_lightIntensity[i];
        else
            col[i] = glm::vec3(0.0f);
    }

    m_litShader.SetVec3Array("uLightPos", pos, count);
    m_litShader.SetVec3Array("uLightColor", col, count);

    // Cornell walls
    for (const auto& part : m_cornell.parts)
    {
        m_litShader.SetVec3("uAlbedo", part.albedo);
        m_cornellMesh.DrawRange(part.indexStart, part.indexCount);
    }

    // Ground plane under everything (dark gray)
    m_litShader.SetVec3("uAlbedo", glm::vec3(0.20f, 0.20f, 0.20f));
    m_groundMesh.Draw();

    constexpr float cornellFloorY = -1.0f;

    auto DrawCube = [&](glm::mat4 model, glm::vec3 albedo)
    {
        m_litShader.SetMat4("uModel", model);
        m_litShader.SetVec3("uAlbedo", albedo);
        m_cubeMesh.Draw();
    };

    auto DrawSphere = [&](glm::mat4 model, glm::vec3 albedo)
    {
        m_litShader.SetMat4("uModel", model);
        m_litShader.SetVec3("uAlbedo", albedo);
        m_sphereMesh.Draw();
    };

    // Tall cube (left)
    glm::vec3 tallPos(-0.45f, 0.0f, -0.20f);
    glm::vec3 tallScl(0.25f, 0.90f, 0.25f);
    {
        glm::mat4 M(1.0f);
        float centerY = cornellFloorY + 0.5f * tallScl.y;
        M = glm::translate(M, glm::vec3(tallPos.x, centerY, tallPos.z));
        M = glm::scale(M, tallScl);
        DrawCube(M, m_albedoTall);
    }

    // Short cube (right)
    glm::vec3 shortPos(0.0f, 0.0f, 0.20f);
    glm::vec3 shortScl(0.45f, 0.35f, 0.45f);
    {
        glm::mat4 M(1.0f);
        float centerY = cornellFloorY + 0.5f * shortScl.y;
        M = glm::translate(M, glm::vec3(shortPos.x, centerY, shortPos.z));
        M = glm::scale(M, shortScl);
        DrawCube(M, m_albedoShort);
    }

    // Small cube (center)
    glm::vec3 smallScl(0.20f, 0.20f, 0.20f);
    {
        glm::mat4 M(1.0f);

        // top of short cube = floorY + fullHeight(short)
        float topShortY = cornellFloorY + shortScl.y;
        float centerY = topShortY + 0.5f * smallScl.y;

        // place directly above the short cube in x/z
        M = glm::translate(M, glm::vec3(shortPos.x, centerY, shortPos.z));
        M = glm::scale(M, smallScl);

        DrawCube(M, m_albedoSmall);
    }

    // Sphere on the floor
    {
        glm::vec3 sphereScl(0.35f);         // scale of the *mesh*, base radius is 0.5
        float radius = 0.5f * sphereScl.y;  // world-space radius

        glm::mat4 M(1.0f);
        M = glm::translate(M, glm::vec3(0.55f, cornellFloorY + radius, -0.25f));
        M = glm::scale(M, sphereScl);

        DrawSphere(M, m_albedoSphere);
    }

    m_litShader.Unbind();
}

void Renderer::DrawDebugUI()
{
    if (!m_ready)
        return;

    ImGui::Begin("Renderer");
    ImGui::Text("Basic Lit");

    ImGui::Separator();

    ImGui::Text("Lights");
    ImGui::SliderInt("Light Count", &m_lightCount, 1, kMaxLights);
    for (int i = 0; i < m_lightCount; ++i)
    {
        ImGui::PushID(i);

        ImGui::Text("Light %d", i);

        ImGui::Checkbox("Enabled", &m_lightEnabled[i]);
        ImGui::SameLine();
        ImGui::DragFloat("Intensity", &m_lightIntensity[i], 0.05f, 0.0f, 50.0f);

        ImGui::DragFloat3("Pos", &m_lights[i].position.x, 0.05f);
        ImGui::ColorEdit3("Color", &m_lights[i].color.x);

        ImGui::Separator();
        ImGui::PopID();
    }
    if (ImGui::Button("All Lights On"))
    {
        for (bool& i : m_lightEnabled)
            i = true;
    }
    ImGui::SameLine();
    if (ImGui::Button("All Lights Off"))
    {
        for (bool& i : m_lightEnabled)
            i = false;
    }

    ImGui::Separator();

    ImGui::Text("Objects");
    ImGui::ColorEdit3("Tall Cube Albedo", &m_albedoTall.x);
    ImGui::ColorEdit3("Short Cube Albedo", &m_albedoShort.x);
    ImGui::ColorEdit3("Small Cube Albedo", &m_albedoSmall.x);
    ImGui::ColorEdit3("Sphere Albedo", &m_albedoSphere.x);

    ImGui::End();
}
