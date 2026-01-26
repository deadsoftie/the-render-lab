#include "pch.h"
#include "Graphics/Renderer.h"

#include <glad/glad.h>
#include <vector>

bool Renderer::Init()
{
    // Load shader
    if (!m_litShader.LoadFromFiles("assets/shaders/basic_lit.vert",
                                   "assets/shaders/basic_lit.frag"))
    {
        return false;
    }

    // Simple triangle
    std::vector<float> verts = {
        -0.5f,
        -0.5f,
        0.f,
        0.f,
        0.f,
        1.f,
        0.5f,
        -0.5f,
        0.f,
        0.f,
        0.f,
        1.f,
        0.0f,
        0.5f,
        0.f,
        0.f,
        0.f,
        1.f,
    };
    std::vector<unsigned int> idx = {0, 1, 2};

    m_triangle.Create(verts, idx);

    // basic GL defaults for now
    glEnable(GL_DEPTH_TEST);

    m_ready = true;
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
    m_litShader.SetMat4("uModel", m_model);
    m_litShader.SetMat4("uView", camera.GetView());
    m_litShader.SetMat4("uProj", camera.GetProj());

    m_litShader.SetVec3("uCamPos", camera.GetPosition());
    m_litShader.SetVec3("uLightPos", m_light.position);
    m_litShader.SetVec3("uLightColor", m_light.color);

    m_litShader.SetVec3("uAlbedo", m_mat.albedo);
    m_litShader.SetFloat("uAmbient", m_mat.ambient);
    m_litShader.SetFloat("uShininess", m_mat.shininess);

    m_triangle.Draw();
    m_litShader.Unbind();
}

void Renderer::DrawDebugUI()
{
    if (!m_ready)
        return;

    ImGui::Begin("Renderer");
    ImGui::Text("Basic Lit Triangle");

    ImGui::Separator();
    ImGui::Text("Light");
    ImGui::DragFloat3("Light Pos", &m_light.position.x, 0.05f);
    ImGui::ColorEdit3("Light Color", &m_light.color.x);

    ImGui::Separator();
    ImGui::Text("Material");
    ImGui::ColorEdit3("Albedo", &m_mat.albedo.x);
    ImGui::DragFloat("Ambient", &m_mat.ambient, 0.01f, 0.0f, 1.0f);
    ImGui::DragFloat("Shininess", &m_mat.shininess, 1.0f, 1.0f, 256.0f);

    ImGui::End();
}
