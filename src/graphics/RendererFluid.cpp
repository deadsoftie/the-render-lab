#include "pch.h"
#include <map>
#include <glad/glad.h>

#include "graphics/Renderer.h"

void Renderer::LoadPbf()
{
    if (!m_activeScene.pbf.enabled)
    {
        m_pbf.Unload();
        return;
    }
    if (!m_pbf.Load(m_activeScene.pbf))
        std::cerr << "[Renderer] Fluid disabled: " << m_pbf.Status() << "\n";
}

void Renderer::DrawPbfParticles(const Camera& camera)
{
    if (!m_pbf.IsLoaded() || !m_showFluidParticles || m_pbf.ParticleCount() == 0)
        return;

    glEnable(GL_PROGRAM_POINT_SIZE);
    m_pbfParticleShader.Bind();
    m_pbfParticleShader.SetMat4("uView", camera.GetView());
    m_pbfParticleShader.SetMat4("uProj", camera.GetProj());
    m_pbfParticleShader.SetMat4("uInvView", glm::inverse(camera.GetView()));
    m_pbfParticleShader.SetFloat("uRadius", m_pbf.Radius());
    m_pbfParticleShader.SetFloat("uViewportHeight", static_cast<float>(m_viewportH));
    m_pbfParticleShader.SetVec3("uColor", m_pbf.Binding().color);

    m_pbf.BindParticleVao();
    glDrawArrays(GL_POINTS, 0, m_pbf.ParticleCount());
    m_pbf.UnbindParticleVao();

    m_pbfParticleShader.Unbind();
    glDisable(GL_PROGRAM_POINT_SIZE);
}

void Renderer::DrawFluidPanel()
{
    if (!m_ready)
        return;

    ImGui::Begin("Fluid PBF");
    if (!m_pbf.IsLoaded())
    {
        ImGui::TextDisabled("%s", m_pbf.Status().empty() ? "No fluid in this scene" : m_pbf.Status().c_str());
        ImGui::End();
        return;
    }

    const Pbf::ParamsGpu& p = m_pbf.Params();
    ImGui::Text("Particles %d / %d", m_pbf.ParticleCount(), m_pbf.MaxParticles());
    ImGui::Text("Radius %.4f m   h %.4f m   grid %d x %d x %d (%d cells)",
                m_pbf.Radius(),
                m_pbf.SmoothingRadius(),
                p.gridDims[0],
                p.gridDims[1],
                p.gridDims[2],
                p.gridDims[3]);
    ImGui::Text("Particle mass %.5f kg   rest density %.0f kg/m^3", p.mass, p.rho0);

    ImGui::Checkbox("Pause##fluid", &m_pbf.paused);
    ImGui::SameLine();
    if (ImGui::Button("Reset##fluid"))
        m_pbf.Reset();
    ImGui::SameLine();
    ImGui::Checkbox("Particles", &m_showFluidParticles);

    Pbf::Settings& settings = m_pbf.settings;
    ImGui::SliderInt("Iterations", &settings.iterations, 1, 10);
    ImGui::SliderInt("Substeps", &settings.substeps, 1, 6);
    ImGui::SliderFloat("Relaxation", &settings.epsilon, 0.001f, 1.0f, "%.3f", ImGuiSliderFlags_Logarithmic);
    ImGui::SliderFloat("Tensile k", &settings.sCorrK, 0.0f, 0.5f, "%.3f");
    ImGui::SliderFloat("Gravity Y##fluid", &settings.gravity[1], -20.0f, 0.0f, "%.2f");
    ImGui::Text("dt %.5f s   simulated %.2f s", m_pbf.TimeStep(), m_pbf.SimulatedSeconds());

    ImGui::Checkbox("Stats", &m_pbf.collectNeighborStats);
    if (m_pbf.collectNeighborStats)
    {
        const Pbf::NeighborStats& n = m_pbf.Neighbors();
        ImGui::SameLine();
        ImGui::Text("neighbors avg %.1f max %u   density/rho0 avg %.3f max %.3f",
                    n.average,
                    n.maximum,
                    n.densityAverage,
                    n.densityMax);
    }

    ImGui::Separator();
    const GpuQueryTimer& timer = m_pbf.Timer();
    std::map<std::string, float> byStage;
    for (const GpuQueryTimer::Stage& stage : timer.Stages())
        byStage[stage.name] += stage.ms;
    ImGui::Text("GPU %.3f ms", timer.TotalMs());
    for (const auto& [name, ms] : byStage)
        ImGui::Text("  %-14s %.3f ms", name.c_str(), ms);

    ImGui::End();
}
