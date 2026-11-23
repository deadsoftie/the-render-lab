#include "pch.h"
#include <algorithm>
#include <cmath>
#include <map>
#include <ImGuizmo.h>
#include <glad/glad.h>

#include "graphics/Renderer.h"
#include "input/Input.h"
#include "pbf/PbfWaterLevel.h"

namespace
{
    constexpr float kFluidPokeSpeed = 0.6f;
    constexpr float kFluidMaxImpulse = 4.0f;
    constexpr float kFluidMouseGain = 1.0f / 6.0f;

    enum CouplingMode
    {
        kCouplingLevel = 0,
        kCouplingImpulse = 1,
        kCouplingBuoyancy = 2,
        kCouplingOff = 3
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
        if (name == "impulse")
            return kCouplingImpulse;
        if (name == "buoyancy")
            return kCouplingBuoyancy;
        if (name == "off")
            return kCouplingOff;
        return kCouplingLevel;
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
    Phys::FluidBuoyancy& buoyancy = m_physics.buoyancy;
    bool fluid = m_pbf.IsLoaded();
    buoyancy.enabled = fluid && (m_couplingMode == kCouplingLevel || m_couplingMode == kCouplingBuoyancy);
    if (!fluid)
        return;
    buoyancy.level = FluidRestLevel();

    if (m_couplingMode == kCouplingLevel)
    {
        m_pbf.PollBodyForces();
        UpdateWaterSurfaces();
        return;
    }
    buoyancy.local.clear();
    m_waterPlanes.clear();
    if (m_couplingMode != kCouplingImpulse)
        return;

    if (m_pbf.PollBodyForces())
    {
        const std::vector<Pbf::BodyForce>& forces = m_pbf.BodyForces();
        m_smoothedForces.resize(forces.size());
        float keep = std::clamp(m_couplingSmoothing, 0.0f, 0.95f);
        for (size_t i = 0; i < forces.size(); ++i)
        {
            m_smoothedForces[i].force = glm::mix(forces[i].force, m_smoothedForces[i].force, keep);
            m_smoothedForces[i].torque = glm::mix(forces[i].torque, m_smoothedForces[i].torque, keep);
        }
    }
    int n = std::min(static_cast<int>(m_smoothedForces.size()), m_physics.BodyCount());
    for (int i = 0; i < n; ++i)
        m_physics.SetExternalForce(i,
                                   GlmToAnim(m_smoothedForces[i].force * m_couplingStrength),
                                   GlmToAnim(m_smoothedForces[i].torque * m_couplingStrength));
}

void Renderer::UpdateWaterSurfaces()
{
    Phys::FluidBuoyancy& buoyancy = m_physics.buoyancy;
    if (!m_pbf.HasColumns())
        return;

    const PbfBinding& binding = m_pbf.Binding();
    Pbf::ColumnField field;
    field.nx = m_pbf.ColumnsX();
    field.nz = m_pbf.ColumnsZ();
    field.cell = m_pbf.SmoothingRadius();
    field.originX = binding.domainMin.x;
    field.originZ = binding.domainMin.z;
    field.floorY = binding.domainMin.y;
    field.particleVolume = m_pbf.ParticleVolume();
    field.data = m_pbf.Columns().data();

    int n = m_physics.BodyCount();
    m_waterFootprints.resize(static_cast<size_t>(n));
    for (int i = 0; i < n; ++i)
    {
        const Phys::RigidBodyState& s = m_physics.State(i);
        Phys::Kinematics kin = m_physics.BodyKinematics(i);
        m_waterFootprints[i] = Pbf::BoxFootprint(kin.x,
                                                 Anim::Column(s.R, 0),
                                                 Anim::Column(s.R, 1),
                                                 Anim::Column(s.R, 2),
                                                 m_physics.Body(i).halfExtents);
    }

    float margin = m_couplingMarginCells * field.cell;
    m_waterPlanes.resize(static_cast<size_t>(n));
    buoyancy.local.resize(static_cast<size_t>(n));
    for (int i = 0; i < n; ++i)
    {
        Pbf::WaterPlane plane = Pbf::FitWaterPlane(field, m_waterFootprints, i, margin, buoyancy.level);
        m_waterPlanes[i] = plane;
        Phys::WaterSurface& surface = buoyancy.local[i];
        surface.valid = plane.valid;
        surface.level = plane.level;
        surface.slopeX = plane.slopeX;
        surface.slopeZ = plane.slopeZ;
        surface.reference = Anim::Vec3{plane.refX, plane.level, plane.refZ};
        surface.velocity = plane.velocity;
    }
}

void Renderer::UploadFluidBodies()
{
    if (!m_pbf.IsLoaded())
        return;
    m_pbf.coupling.twoWay = m_couplingMode == kCouplingImpulse;
    m_pbf.columnsEnabled = m_physicsActive && m_couplingMode == kCouplingLevel;
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
    if (!m_pbf.IsLoaded() || !m_showFluidParticles || m_pbf.ParticleCount() == 0 || FluidSurfaceActive())
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

bool Renderer::FluidSurfaceActive() const
{
    return m_useDeferred && m_showFluidParticles && m_fluidView != 1 && m_pbf.IsLoaded() && m_pbf.ParticleCount() > 0;
}

void Renderer::FluidSurfacePass(const Camera& camera)
{
    if (!FluidSurfaceActive() || !m_fluidSurface.Resize(m_viewportW, m_viewportH))
        return;

    m_fluidRenderTimer.BeginFrame();
    m_fluidRenderTimer.Begin("Surface depth");
    m_fluidSurface.BindDepthForWriting();
    const float zero[4] = {0.0f, 0.0f, 0.0f, 0.0f};
    const float one = 1.0f;
    glClearBufferfv(GL_COLOR, 0, zero);
    glClearBufferfv(GL_DEPTH, 0, &one);
    glEnable(GL_DEPTH_TEST);
    glDepthFunc(GL_LESS);
    glDepthMask(GL_TRUE);
    glDisable(GL_BLEND);
    glDisable(GL_CULL_FACE);
    glEnable(GL_PROGRAM_POINT_SIZE);
    glActiveTexture(GL_TEXTURE0);
    glBindTexture(GL_TEXTURE_2D, m_gbuffer.TexDepth());

    m_pbf.BindParticleVao();
    m_pbfDepthShader.Bind();
    m_pbfDepthShader.SetMat4("uView", camera.GetView());
    m_pbfDepthShader.SetMat4("uProj", camera.GetProj());
    m_pbfDepthShader.SetFloat("uRadius", m_pbf.Radius());
    m_pbfDepthShader.SetFloat("uViewportHeight", static_cast<float>(m_fluidSurface.Height()));
    m_pbfDepthShader.SetVec2("uViewport", glm::vec2(m_fluidSurface.Width(), m_fluidSurface.Height()));
    m_pbfDepthShader.SetInt("uSceneDepth", 0);
    glDrawArrays(GL_POINTS, 0, m_pbf.ParticleCount());

    m_fluidRenderTimer.Begin("Surface thickness");
    m_fluidSurface.BindThicknessForWriting();
    glClearBufferfv(GL_COLOR, 0, zero);
    glDisable(GL_DEPTH_TEST);
    glDepthMask(GL_FALSE);
    glEnable(GL_BLEND);
    glBlendEquation(GL_FUNC_ADD);
    glBlendFunc(GL_ONE, GL_ONE);
    m_pbfThicknessShader.Bind();
    m_pbfThicknessShader.SetMat4("uView", camera.GetView());
    m_pbfThicknessShader.SetMat4("uProj", camera.GetProj());
    m_pbfThicknessShader.SetFloat("uRadius", m_pbf.Radius());
    m_pbfThicknessShader.SetFloat("uThicknessScale", m_fluidThicknessScale);
    m_pbfThicknessShader.SetFloat("uViewportHeight", static_cast<float>(m_fluidSurface.ThicknessHeight()));
    m_pbfThicknessShader.SetVec2("uViewport",
                                 glm::vec2(m_fluidSurface.ThicknessWidth(), m_fluidSurface.ThicknessHeight()));
    m_pbfThicknessShader.SetInt("uSceneDepth", 0);
    glDrawArrays(GL_POINTS, 0, m_pbf.ParticleCount());

    m_pbfThicknessShader.Unbind();
    m_pbf.UnbindParticleVao();
    glBindTexture(GL_TEXTURE_2D, 0);
    glDisable(GL_BLEND);
    glDepthMask(GL_TRUE);
    glEnable(GL_DEPTH_TEST);
    glDisable(GL_PROGRAM_POINT_SIZE);
    FluidSurfaceBuffer::UnbindWriting();
    glViewport(0, 0, m_viewportW, m_viewportH);

    SmoothFluidSurface(camera);
    m_fluidRenderTimer.End();
}

void Renderer::SmoothFluidSurface(const Camera& camera)
{
    const FluidSurfaceBuffer& fs = m_fluidSurface;
    int groupsX = (fs.Width() + 15) / 16;
    int groupsY = (fs.Height() + 15) / 16;
    glm::mat4 proj = camera.GetProj();
    float radius = m_pbf.Radius();
    m_fluidDepthResult = 0;

    bool smooth = m_fluidView != 2 && m_fluidView != 4;
    if (smooth && m_fluidSmoothing == 0 && m_fluidSmoothIterations > 0)
    {
        m_fluidRenderTimer.Begin("Surface smoothing");
        m_pbfCurvatureShader.Bind();
        m_pbfCurvatureShader.SetFloat("uCx", 2.0f / (static_cast<float>(fs.Width()) * proj[0][0]));
        m_pbfCurvatureShader.SetFloat("uCy", 2.0f / (static_cast<float>(fs.Height()) * proj[1][1]));
        m_pbfCurvatureShader.SetFloat("uStep", m_fluidSmoothStep);
        m_pbfCurvatureShader.SetFloat("uEdge", m_fluidEdgeRadii * radius);
        int src = 0;
        for (int i = 0; i < m_fluidSmoothIterations; ++i)
        {
            glBindImageTexture(0, fs.TexEyeDepth(src), 0, GL_FALSE, 0, GL_READ_ONLY, GL_R32F);
            glBindImageTexture(1, fs.TexEyeDepth(1 - src), 0, GL_FALSE, 0, GL_WRITE_ONLY, GL_R32F);
            m_pbfCurvatureShader.Dispatch(groupsX, groupsY);
            src = 1 - src;
        }
        m_fluidDepthResult = src;
    }
    else if (smooth && m_fluidSmoothing == 1)
    {
        m_fluidRenderTimer.Begin("Surface smoothing");
        m_pbfBilateralShader.Bind();
        m_pbfBilateralShader.SetFloat("uWorldRadius", m_fluidBilateralRadii * radius);
        m_pbfBilateralShader.SetFloat("uPixelsPerMeter", 0.5f * proj[1][1] * static_cast<float>(fs.Height()));
        m_pbfBilateralShader.SetFloat("uDepthFalloff", m_fluidBilateralFalloffRadii * radius);
        for (int pass = 0; pass < 2; ++pass)
        {
            glBindImageTexture(0, fs.TexEyeDepth(pass), 0, GL_FALSE, 0, GL_READ_ONLY, GL_R32F);
            glBindImageTexture(1, fs.TexEyeDepth(1 - pass), 0, GL_FALSE, 0, GL_WRITE_ONLY, GL_R32F);
            m_pbfBilateralShader.SetVec2("uDirection", pass == 0 ? glm::vec2(1.0f, 0.0f) : glm::vec2(0.0f, 1.0f));
            m_pbfBilateralShader.Dispatch(groupsX, groupsY);
        }
        m_fluidDepthResult = 0;
    }

    if (smooth)
    {
        m_fluidRenderTimer.Begin("Surface normals");
        m_pbfNormalsShader.Bind();
        m_pbfNormalsShader.SetVec2("uInvProjScale", glm::vec2(1.0f / proj[0][0], 1.0f / proj[1][1]));
        m_pbfNormalsShader.SetMat3("uInvViewRot", glm::mat3(glm::inverse(camera.GetView())));
        glBindImageTexture(0, fs.TexEyeDepth(m_fluidDepthResult), 0, GL_FALSE, 0, GL_READ_ONLY, GL_R32F);
        glBindImageTexture(1, fs.TexNormal(), 0, GL_FALSE, 0, GL_WRITE_ONLY, GL_RGBA16F);
        m_pbfNormalsShader.Dispatch(groupsX, groupsY);
    }

    if (m_fluidBlurThickness)
    {
        m_fluidRenderTimer.Begin("Thickness blur");
        int tx = (fs.ThicknessWidth() + 15) / 16;
        int ty = (fs.ThicknessHeight() + 15) / 16;
        m_pbfThicknessBlurShader.Bind();
        glBindImageTexture(0, fs.TexThickness(), 0, GL_FALSE, 0, GL_READ_ONLY, GL_R16F);
        glBindImageTexture(1, fs.TexThicknessTemp(), 0, GL_FALSE, 0, GL_WRITE_ONLY, GL_R16F);
        m_pbfThicknessBlurShader.SetVec2("uDirection", glm::vec2(1.0f, 0.0f));
        m_pbfThicknessBlurShader.Dispatch(tx, ty);
        glBindImageTexture(0, fs.TexThicknessTemp(), 0, GL_FALSE, 0, GL_READ_ONLY, GL_R16F);
        glBindImageTexture(1, fs.TexThickness(), 0, GL_FALSE, 0, GL_WRITE_ONLY, GL_R16F);
        m_pbfThicknessBlurShader.SetVec2("uDirection", glm::vec2(0.0f, 1.0f));
        m_pbfThicknessBlurShader.Dispatch(tx, ty);
    }

    glBindImageTexture(0, 0, 0, GL_FALSE, 0, GL_READ_ONLY, GL_R32F);
    glBindImageTexture(1, 0, 0, GL_FALSE, 0, GL_WRITE_ONLY, GL_R32F);
    glUseProgram(0);
}

void Renderer::CompositeFluidSurface(const Camera& camera)
{
    if (!FluidSurfaceActive() || m_fluidView != 0 || m_fluidSurface.Width() == 0)
        return;

    m_fluidRenderTimer.Begin("Surface composite");
    const FluidSurfaceBuffer& fs = m_fluidSurface;
    fs.CopySceneColor();

    const PbfBinding& binding = m_pbf.Binding();
    glm::mat4 view = camera.GetView();
    glm::mat4 proj = camera.GetProj();
    glm::vec3 lightDir(0.0f, 1.0f, 0.0f);
    glm::vec3 lightColor(0.0f);
    bool hasEnv = m_hdriTex.ID() != 0;
    if (hasEnv && m_hasHDRISun)
    {
        float c = std::cos(-m_hdriRotation);
        float s = std::sin(-m_hdriRotation);
        glm::vec3 d = m_hdriSunDir;
        lightDir = glm::normalize(glm::vec3(d.x * c + d.z * s, d.y, -d.x * s + d.z * c));
        lightColor = glm::vec3(m_fluidSpecular);
    }
    else if (m_lightCount > 0 && m_lightEnabled[0])
    {
        glm::vec3 center = 0.5f * (binding.domainMin + binding.domainMax);
        glm::vec3 toLight = m_lights[0].position - center;
        if (glm::dot(toLight, toLight) > 1e-8f)
            lightDir = glm::normalize(toLight);
        lightColor = m_lights[0].color * m_lightIntensity[0] * m_fluidSpecular;
    }

    glBindFramebuffer(GL_FRAMEBUFFER, 0);
    glViewport(0, 0, m_viewportW, m_viewportH);
    glDisable(GL_DEPTH_TEST);
    glDisable(GL_BLEND);

    Shader& sh = m_pbfCompositeShader;
    sh.Bind();
    const GLuint textures[] = {fs.TexSceneColor(),
                               fs.TexEyeDepth(m_fluidDepthResult),
                               fs.TexNormal(),
                               fs.TexThickness(),
                               m_gbuffer.TexDepth(),
                               m_hdriTex.ID()};
    const char* names[] = {"uSceneColor", "uEyeDepth", "uNormals", "uThickness", "uSceneDepth", "uHDRITex"};
    for (int i = 0; i < 6; ++i)
    {
        glActiveTexture(GL_TEXTURE0 + i);
        glBindTexture(GL_TEXTURE_2D, textures[i]);
        sh.SetInt(names[i], i);
    }
    sh.SetVec2("uInvProjScale", glm::vec2(1.0f / proj[0][0], 1.0f / proj[1][1]));
    sh.SetMat4("uView", view);
    sh.SetMat4("uInvView", glm::inverse(view));
    sh.SetMat4("uProj", proj);
    sh.SetVec3("uCamPos", camera.GetPosition());
    sh.SetInt("uHasEnv", hasEnv ? 1 : 0);
    sh.SetFloat("uHDRIRotation", m_hdriRotation);
    sh.SetFloat("uExposure", m_exposure);
    sh.SetVec3("uColor", binding.color);
    sh.SetFloat("uAbsorption", binding.absorption);
    sh.SetFloat("uScatter", m_fluidScatter);
    sh.SetFloat("uRefraction", m_fluidRefraction);
    sh.SetFloat("uRoughness", m_fluidRoughness);
    sh.SetVec3("uLightDir", lightDir);
    sh.SetVec3("uLightColor", lightColor);
    glBindVertexArray(m_quadVAO);
    glDrawArrays(GL_TRIANGLES, 0, 6);

    if (m_fluidWriteDepth)
    {
        m_gbuffer.BindForWriting();
        glViewport(0, 0, m_viewportW, m_viewportH);
        glColorMask(GL_FALSE, GL_FALSE, GL_FALSE, GL_FALSE);
        glEnable(GL_DEPTH_TEST);
        glDepthFunc(GL_LESS);
        glDepthMask(GL_TRUE);
        m_pbfDepthMergeShader.Bind();
        glActiveTexture(GL_TEXTURE0);
        glBindTexture(GL_TEXTURE_2D, fs.TexEyeDepth(m_fluidDepthResult));
        m_pbfDepthMergeShader.SetInt("uEyeDepth", 0);
        m_pbfDepthMergeShader.SetMat4("uProj", proj);
        glDrawArrays(GL_TRIANGLES, 0, 6);
        glColorMask(GL_TRUE, GL_TRUE, GL_TRUE, GL_TRUE);
        GBuffer::UnbindWriting();
    }

    glBindVertexArray(0);
    for (int i = 5; i >= 0; --i)
    {
        glActiveTexture(GL_TEXTURE0 + i);
        glBindTexture(GL_TEXTURE_2D, 0);
    }
    glUseProgram(0);
    glEnable(GL_DEPTH_TEST);
    m_fluidRenderTimer.End();
}

void Renderer::DrawFluidSurfaceDebug(const Camera& camera)
{
    if (!FluidSurfaceActive() || m_fluidView < 2 || m_fluidSurface.Width() == 0)
        return;

    const PbfBinding& binding = m_pbf.Binding();
    glm::mat4 view = camera.GetView();
    glm::vec3 forward = -glm::vec3(view[0][2], view[1][2], view[2][2]);
    glm::vec3 eye = camera.GetPosition();
    float nearest = 1e30f;
    float farthest = 0.0f;
    for (int corner = 0; corner < 8; ++corner)
    {
        glm::vec3 p((corner & 1) ? binding.domainMax.x : binding.domainMin.x,
                    (corner & 2) ? binding.domainMax.y : binding.domainMin.y,
                    (corner & 4) ? binding.domainMax.z : binding.domainMin.z);
        float d = glm::dot(p - eye, forward);
        nearest = std::min(nearest, d);
        farthest = std::max(farthest, d);
    }
    nearest = std::max(nearest, 0.0f);

    glBindFramebuffer(GL_FRAMEBUFFER, 0);
    glViewport(0, 0, m_viewportW, m_viewportH);
    glDisable(GL_DEPTH_TEST);
    glDisable(GL_BLEND);
    m_pbfSurfaceDebugShader.Bind();
    glActiveTexture(GL_TEXTURE0);
    glBindTexture(GL_TEXTURE_2D, m_fluidSurface.TexEyeDepth(m_fluidView == 2 ? 0 : m_fluidDepthResult));
    glActiveTexture(GL_TEXTURE1);
    glBindTexture(GL_TEXTURE_2D, m_fluidSurface.TexThickness());
    glActiveTexture(GL_TEXTURE2);
    glBindTexture(GL_TEXTURE_2D, m_fluidSurface.TexNormal());
    m_pbfSurfaceDebugShader.SetInt("uEyeDepth", 0);
    m_pbfSurfaceDebugShader.SetInt("uThickness", 1);
    m_pbfSurfaceDebugShader.SetInt("uNormals", 2);
    const int modeForView[] = {0, 0, 1, 1, 2, 3};
    m_pbfSurfaceDebugShader.SetInt("uMode", modeForView[std::clamp(m_fluidView, 0, 5)]);
    m_pbfSurfaceDebugShader.SetVec2("uDepthRange", glm::vec2(nearest, std::max(farthest, nearest + 1e-3f)));
    m_pbfSurfaceDebugShader.SetFloat("uAbsorption", binding.absorption);
    m_pbfSurfaceDebugShader.SetVec3("uColor", binding.color);
    glBindVertexArray(m_quadVAO);
    glDrawArrays(GL_TRIANGLES, 0, 6);
    glBindVertexArray(0);
    glBindTexture(GL_TEXTURE_2D, 0);
    glActiveTexture(GL_TEXTURE1);
    glBindTexture(GL_TEXTURE_2D, 0);
    glActiveTexture(GL_TEXTURE0);
    glBindTexture(GL_TEXTURE_2D, 0);
    m_pbfSurfaceDebugShader.Unbind();
    glEnable(GL_DEPTH_TEST);
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
    ImGui::Checkbox("Show fluid", &m_showFluidParticles);
    const char* views[] = {"Surface", "Particles", "Raw depth", "Smoothed depth", "Thickness", "Normals"};
    ImGui::SetNextItemWidth(180.0f);
    ImGui::Combo("View##fluid", &m_fluidView, views, IM_ARRAYSIZE(views));
    if (m_fluidView == 0)
    {
        ImGui::SliderFloat("Refraction", &m_fluidRefraction, 0.0f, 1.5f, "%.2f");
        ImGui::SliderFloat("Scattering", &m_fluidScatter, 0.0f, 1.0f, "%.2f");
        ImGui::SliderFloat("Absorption", &m_pbf.BindingMutable().absorption, 0.0f, 40.0f, "%.1f 1/m");
        ImGui::ColorEdit3("Water color", &m_pbf.BindingMutable().color.x);
        ImGui::SliderFloat("Sun specular", &m_fluidSpecular, 0.0f, 20.0f, "%.1f");
        ImGui::SliderFloat("Roughness##fluid", &m_fluidRoughness, 0.02f, 0.5f, "%.2f");
        ImGui::Checkbox("Write fluid depth", &m_fluidWriteDepth);
    }
    if (m_fluidView != 1)
    {
        const char* smoothing[] = {"Curvature flow", "Bilateral", "None"};
        ImGui::SetNextItemWidth(180.0f);
        ImGui::Combo("Smoothing", &m_fluidSmoothing, smoothing, IM_ARRAYSIZE(smoothing));
        if (m_fluidSmoothing == 0)
        {
            ImGui::SliderInt("Flow iterations", &m_fluidSmoothIterations, 0, 120);
            ImGui::SliderFloat("Flow step", &m_fluidSmoothStep, 0.05f, 0.5f, "%.2f");
            ImGui::SliderFloat("Edge threshold (r)", &m_fluidEdgeRadii, 1.0f, 20.0f, "%.1f");
        }
        else if (m_fluidSmoothing == 1)
        {
            ImGui::SliderFloat("Filter radius (r)", &m_fluidBilateralRadii, 1.0f, 8.0f, "%.1f");
            ImGui::SliderFloat("Depth falloff (r)", &m_fluidBilateralFalloffRadii, 0.5f, 8.0f, "%.1f");
        }
        ImGui::Checkbox("Blur thickness", &m_fluidBlurThickness);
        ImGui::SameLine();
        ImGui::SetNextItemWidth(140.0f);
        ImGui::SliderFloat("Thickness scale", &m_fluidThicknessScale, 0.5f, 3.0f, "%.2f");
    }
    if (m_fluidView != 1 && !m_useDeferred)
        ImGui::TextDisabled("Surface views need the deferred renderer");

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
        const char* modes[] = {"Water level (two-way)", "Impulse exchange (two-way)", "Buoyancy (rest level)", "Off"};
        ImGui::Combo("Coupling", &m_couplingMode, modes, IM_ARRAYSIZE(modes));
        if (m_couplingMode == kCouplingLevel)
        {
            ImGui::SliderFloat("Water drag", &m_physics.buoyancy.drag, 0.0f, 10.0f, "%.2f 1/s");
            ImGui::SliderFloat("Sample margin (h)", &m_couplingMarginCells, 0.5f, 4.0f, "%.1f");
            ImGui::Checkbox("Wait for GPU data", &m_pbf.coupling.waitForForces);
            int selected = m_physicsSelectedBody;
            if (selected >= 0 && selected < static_cast<int>(m_waterPlanes.size()))
            {
                const Pbf::WaterPlane& w = m_waterPlanes[selected];
                ImGui::Text("Local water: level %.3f m  slope (%.3f, %.3f)  flow (%.2f, %.2f) m/s  %d samples%s",
                            w.level,
                            w.slopeX,
                            w.slopeZ,
                            w.velocity.x,
                            w.velocity.z,
                            w.samples,
                            w.valid ? "" : " (rest level)");
            }
        }
        else if (m_couplingMode == kCouplingImpulse)
        {
            ImGui::TextDisabled("Momentum exchange; unstable when planks are light next to the water they push");
            ImGui::SliderFloat("Coupling strength", &m_couplingStrength, 0.0f, 2.0f, "%.2f");
            ImGui::SliderFloat("Force smoothing", &m_couplingSmoothing, 0.0f, 0.95f, "%.2f");
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
    if (FluidSurfaceActive())
    {
        ImGui::Text("Surface render %.3f ms", m_fluidRenderTimer.TotalMs());
        for (const GpuQueryTimer::Stage& stage : m_fluidRenderTimer.Stages())
            ImGui::Text("  %-18s %.3f ms", stage.name.c_str(), stage.ms);
    }

    ImGui::End();
}
