#include "pch.h"
#include <algorithm>
#include <map>
#include <ImGuizmo.h>
#include <glad/glad.h>

#include "graphics/Renderer.h"
#include "input/Input.h"

namespace
{
    constexpr float kFluidPokeSpeed = 0.6f;
    constexpr float kFluidMaxImpulse = 4.0f;
    constexpr float kFluidMouseGain = 1.0f / 6.0f;

    enum CouplingMode
    {
        kCouplingTwoWay = 0,
        kCouplingBuoyancy = 1,
        kCouplingOff = 2
    };

    glm::vec3 AnimToGlm(const Anim::Vec3& v)
    {
        return glm::vec3(v.x, v.y, v.z);
    }

    Anim::Vec3 GlmToAnim(const glm::vec3& v)
    {
        return Anim::Vec3{v.x, v.y, v.z};
    }

    int CouplingModeFromName(const std::string& name)
    {
        if (name == "buoyancy")
            return kCouplingBuoyancy;
        if (name == "off")
            return kCouplingOff;
        return kCouplingTwoWay;
    }
}

void Renderer::LoadPbf()
{
    if (!m_activeScene.pbf.enabled)
    {
        m_pbf.Unload();
        return;
    }
    if (!m_pbf.Load(m_activeScene.pbf))
        std::cerr << "[Renderer] Fluid disabled: " << m_pbf.Status() << "\n";
    m_couplingMode = CouplingModeFromName(m_activeScene.pbf.coupling);
    m_couplingStrength = m_activeScene.pbf.couplingStrength;
    m_pbf.coupling.friction = m_activeScene.pbf.friction;
}

float Renderer::FluidRestLevel() const
{
    const PbfBinding& b = m_pbf.Binding();
    glm::vec3 extent = b.domainMax - b.domainMin;
    float spacing = 2.0f * b.particleRadius;
    float area = std::max(extent.x * extent.z, 1e-6f);
    return b.domainMin.y + static_cast<float>(m_pbf.ParticleCount()) * spacing * spacing * spacing / area;
}

void Renderer::ApplyFluidForces()
{
    if (!m_physicsActive)
        return;
    m_physics.ClearExternalForces();
    bool fluid = m_pbf.IsLoaded();
    m_physics.buoyancy.enabled = fluid && m_couplingMode == kCouplingBuoyancy;
    if (!fluid)
        return;
    if (m_physics.buoyancy.enabled)
    {
        m_physics.buoyancy.level = FluidRestLevel();
        return;
    }
    if (m_couplingMode != kCouplingTwoWay)
        return;

    m_pbf.PollBodyForces();
    const std::vector<Pbf::BodyForce>& forces = m_pbf.BodyForces();
    int n = std::min(static_cast<int>(forces.size()), m_physics.BodyCount());
    for (int i = 0; i < n; ++i)
        m_physics.SetExternalForce(i,
                                   GlmToAnim(forces[i].force * m_couplingStrength),
                                   GlmToAnim(forces[i].torque * m_couplingStrength));
}

void Renderer::UploadFluidBodies()
{
    if (!m_pbf.IsLoaded())
        return;
    m_pbf.coupling.twoWay = m_couplingMode == kCouplingTwoWay;
    std::vector<Pbf::CouplingBody> bodies;
    if (m_physicsActive && m_couplingMode != kCouplingOff)
    {
        int n = std::min(m_physics.BodyCount(), Pbf::kMaxBodies);
        for (int i = 0; i < n; ++i)
        {
            const Phys::RigidBodyState& s = m_physics.State(i);
            Phys::Kinematics kin = m_physics.BodyKinematics(i);
            Pbf::CouplingBody b;
            b.com = AnimToGlm(s.c);
            b.cObj = AnimToGlm(m_physics.Properties(i).cObj);
            b.rotation = glm::mat3(AnimToGlm(Anim::Column(s.R, 0)),
                                   AnimToGlm(Anim::Column(s.R, 1)),
                                   AnimToGlm(Anim::Column(s.R, 2)));
            b.halfExtents = AnimToGlm(m_physics.Body(i).halfExtents);
            b.velocity = AnimToGlm(kin.cDot);
            b.omega = AnimToGlm(kin.omega);
            bodies.push_back(b);
        }
    }
    m_pbf.SetBodies(bodies);
}

void Renderer::UpdateFluidInput(float deltaSeconds)
{
    Pbf::MouseForce& mouse = m_pbf.mouse;
    mouse.active = false;
    if (!m_pbf.IsLoaded() || m_ikTargetIndex >= 0)
        return;

    ImGuiIO& io = ImGui::GetIO();
    if (io.WantCaptureMouse || !io.KeyShift || !Input::MouseDown(GLFW_MOUSE_BUTTON_LEFT) || ImGuizmo::IsUsing())
        return;

    glm::vec3 origin, direction;
    if (!MouseRay(origin, direction))
        return;

    const PbfBinding& binding = m_pbf.Binding();
    glm::vec3 center = 0.5f * (binding.domainMin + binding.domainMax);
    float depth = std::max(glm::dot(center - origin, direction), 0.1f);
    float worldPerPixel = 2.0f * depth / (m_cachedProj[1][1] * static_cast<float>(std::max(m_viewportH, 1)));

    double dx = 0.0, dy = 0.0;
    Input::GetMouseDelta(dx, dy);
    glm::mat4 cameraToWorld = glm::inverse(m_cachedView);
    glm::vec3 right = glm::vec3(cameraToWorld[0]);
    glm::vec3 up = glm::vec3(cameraToWorld[1]);
    glm::vec3 drag = (right * static_cast<float>(dx) - up * static_cast<float>(dy)) * (worldPerPixel / std::max(deltaSeconds, 1e-3f));

    glm::vec3 impulse = (drag + direction * kFluidPokeSpeed) * m_fluidMouseStrength;
    float len = glm::length(impulse);
    if (len > kFluidMaxImpulse)
        impulse *= kFluidMaxImpulse / len;

    mouse.active = true;
    mouse.origin = origin;
    mouse.direction = direction;
    mouse.radius = m_fluidMouseRadius;
    mouse.impulse = impulse * kFluidMouseGain;
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
    ImGui::SliderFloat("Vorticity", &settings.vorticityEps, 0.0f, 2.0f, "%.3f");
    ImGui::SliderFloat("XSPH viscosity", &settings.xsphC, 0.0f, 0.2f, "%.3f");
    if (m_pbf.EmitterCount() > 0)
    {
        ImGui::Checkbox("Emitters", &m_pbf.emittersEnabled);
        ImGui::SameLine();
        ImGui::TextDisabled("(%d)", m_pbf.EmitterCount());
    }
    ImGui::SliderFloat("Mouse radius", &m_fluidMouseRadius, 0.03f, 0.5f, "%.2f m");
    ImGui::SliderFloat("Mouse strength", &m_fluidMouseStrength, 0.0f, 3.0f, "%.2f");
    ImGui::TextDisabled("Shift + left drag pushes the water");
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

    if (m_physicsActive)
    {
        ImGui::Separator();
        const char* modes[] = {"Two-way (PBF)", "Buoyancy (analytic)", "Off"};
        ImGui::Combo("Coupling", &m_couplingMode, modes, 3);
        if (m_couplingMode == kCouplingTwoWay)
        {
            ImGui::SliderFloat("Coupling strength", &m_couplingStrength, 0.0f, 2.0f, "%.2f");
            ImGui::SliderFloat("Plank friction", &m_pbf.coupling.friction, 0.0f, 1.0f, "%.2f");
            ImGui::Checkbox("Wait for GPU forces", &m_pbf.coupling.waitForForces);
        }
        else if (m_couplingMode == kCouplingBuoyancy)
        {
            ImGui::Text("Water level %.3f m", FluidRestLevel());
            ImGui::SliderFloat("Water drag", &m_physics.buoyancy.drag, 0.0f, 10.0f, "%.2f 1/s");
        }
        ImGui::Checkbox("Fluid force arrows", &m_showFluidForces);
        if (m_physics.BodyCount() > Pbf::kMaxBodies)
            ImGui::TextDisabled("Only the first %d planks are coupled", Pbf::kMaxBodies);

        float lift = 0.0f;
        float weight = 0.0f;
        for (int i = 0; i < m_physics.BodyCount(); ++i)
        {
            lift += m_physics.FluidForce(i).force.y;
            weight -= m_physics.Properties(i).mass * m_physics.settings.gravity.y;
        }
        ImGui::Text("Fluid lift %.1f N   plank weight %.1f N", lift, weight);
        int selected = m_physicsSelectedBody;
        if (selected >= 0 && selected < m_physics.BodyCount())
        {
            Phys::ForceAccumulator f = m_physics.FluidForce(selected);
            ImGui::Text("%s: F (%.1f, %.1f, %.1f) N   T (%.2f, %.2f, %.2f) N m",
                        m_physics.Body(selected).name.c_str(),
                        f.force.x,
                        f.force.y,
                        f.force.z,
                        f.torque.x,
                        f.torque.y,
                        f.torque.z);
        }
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
