#include "pch.h"
#include <algorithm>
#include <cfloat>
#include <chrono>
#include <cstring>
#include <glad/glad.h>
#include <glm/ext/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>

#include "graphics/Renderer.h"

namespace
{
    Anim::Vec3 ToAnim(const glm::vec3& v)
    {
        return Anim::Vec3{v.x, v.y, v.z};
    }

    glm::vec3 ToGlm(const Anim::Vec3& v)
    {
        return glm::vec3(v.x, v.y, v.z);
    }

    Anim::Mat3 RotationFromEulerDegrees(const glm::vec3& degrees)
    {
        glm::mat4 M(1.0f);
        M = glm::rotate(M, glm::radians(degrees.x), glm::vec3(1.0f, 0.0f, 0.0f));
        M = glm::rotate(M, glm::radians(degrees.y), glm::vec3(0.0f, 1.0f, 0.0f));
        M = glm::rotate(M, glm::radians(degrees.z), glm::vec3(0.0f, 0.0f, 1.0f));
        return Anim::FromColumns(ToAnim(glm::vec3(M[0])), ToAnim(glm::vec3(M[1])), ToAnim(glm::vec3(M[2])));
    }

    Phys::IntegratorType IntegratorFromName(const std::string& name)
    {
        if (name == "Euler")
            return Phys::IntegratorType::Euler;
        if (name == "Midpoint")
            return Phys::IntegratorType::Midpoint;
        if (name == "Verlet")
            return Phys::IntegratorType::Verlet;
        if (name == "RK4")
            return Phys::IntegratorType::RK4;
        return Phys::IntegratorType::RK2;
    }

    bool ResolveSpringEnd(const Phys::PhysicsWorld& world,
                          const PhysicsSpringEndBinding& binding,
                          Phys::SpringEnd& end,
                          std::string& error)
    {
        if (!binding.anchor.empty())
        {
            end.anchor = world.FindAnchor(binding.anchor);
            if (end.anchor < 0)
                error = "spring references unknown anchor '" + binding.anchor + "'";
            return end.anchor >= 0;
        }
        end.body = world.FindBody(binding.body);
        end.localPoint = ToAnim(binding.point);
        if (end.body < 0)
            error = "spring references unknown body '" + binding.body + "'";
        return end.body >= 0;
    }
}

void Renderer::LoadPhysics()
{
    m_physics.Clear();
    m_physicsBodyObjects.clear();
    m_physicsAnchorObjects.clear();
    m_physicsActive = false;
    m_physicsPaused = false;
    m_physicsStepRequested = false;
    m_physicsStatus.clear();
    m_physicsSelectedBody = 0;
    std::fill(std::begin(m_energyHistory), std::end(m_energyHistory), 0.0f);
    m_energyHistoryOffset = 0;

    const PhysicsBinding& binding = m_activeScene.physics;
    if (!binding.enabled)
        return;

    Phys::PhysicsSettings& settings = m_physics.settings;
    settings.integrator = IntegratorFromName(binding.integrator);
    settings.dt = binding.dt;
    settings.maxStepsPerFrame = binding.maxStepsPerFrame;
    settings.gravity = ToAnim(binding.gravity);
    settings.linearDrag = binding.linearDrag;
    settings.angularDrag = binding.angularDrag;

    std::string error;
    if (binding.chain.enabled)
    {
        const PhysicsChainBinding& c = binding.chain;
        Phys::ChainDesc desc;
        desc.plankCount = c.count;
        desc.anchorA = ToAnim(c.anchorA);
        desc.anchorB = ToAnim(c.anchorB);
        desc.plankHalfExtents = ToAnim(c.plankHalfExtents);
        desc.plankMass = c.plankMass;
        desc.restLength = c.restLength;
        desc.stiffness = c.stiffness;
        desc.damping = c.damping;
        desc.diagonals = c.diagonals;
        if (!Phys::BuildChain(desc, m_physics, error))
        {
            m_physicsStatus = "Invalid chain: " + error;
            m_physics.Clear();
            return;
        }
    }

    for (const PhysicsAnchorBinding& a : binding.anchors)
        m_physics.AddAnchor(a.name, ToAnim(a.position));

    for (const PhysicsBodyBinding& b : binding.bodies)
    {
        Phys::BodyDesc desc;
        desc.name = b.name;
        desc.halfExtents = ToAnim(b.halfExtents);
        desc.mass = b.mass;
        desc.position = ToAnim(b.position);
        desc.rotation = RotationFromEulerDegrees(b.rotationEulerDegrees);
        desc.velocity = ToAnim(b.velocity);
        desc.angularVelocity = ToAnim(b.angularVelocity);
        m_physics.AddBody(desc);
    }

    for (const PhysicsSpringBinding& sb : binding.springs)
    {
        Phys::Spring spring;
        if (!ResolveSpringEnd(m_physics, sb.a, spring.a, error) ||
            !ResolveSpringEnd(m_physics, sb.b, spring.b, error))
        {
            m_physicsStatus = "Invalid spring: " + error;
            m_physics.Clear();
            return;
        }
        spring.stiffness = sb.stiffness;
        spring.damping = sb.damping;
        spring.restLength =
            sb.restLength >= 0.0f ? sb.restLength : m_physics.CurrentLength(spring.a, spring.b);
        m_physics.AddSpring(spring);
    }

    m_physics.CaptureInitialState();

    if (m_physics.SpringCount() > 0)
    {
        m_physicsStiffness = m_physics.GetSpring(0).stiffness;
        m_physicsDamping = m_physics.GetSpring(0).damping;
    }

    for (int i = 0; i < m_physics.BodyCount(); ++i)
    {
        const Phys::BodyDesc& body = m_physics.Body(i);
        SceneObject obj;
        obj.name = body.name;
        obj.meshRef = "cube";
        obj.scale = 2.0f * ToGlm(body.halfExtents);
        obj.material = binding.bodyMaterial;
        obj.hasModelOverride = true;
        m_physicsBodyObjects.push_back(static_cast<int>(m_activeScene.objects.size()));
        m_activeScene.objects.push_back(obj);
    }

    for (int i = 0; i < m_physics.AnchorCount(); ++i)
    {
        SceneObject obj;
        obj.name = m_physics.GetAnchor(i).name;
        obj.meshRef = "sphere";
        obj.position = ToGlm(m_physics.GetAnchor(i).target);
        obj.scale = glm::vec3(2.0f * binding.anchorRadius);
        obj.material = binding.anchorMaterial;
        m_physicsAnchorObjects.push_back(static_cast<int>(m_activeScene.objects.size()));
        m_activeScene.objects.push_back(obj);
    }

    m_physicsActive = true;
    SyncPhysicsObjects();
}

void Renderer::SyncPhysicsObjects()
{
    for (int i = 0; i < static_cast<int>(m_physicsBodyObjects.size()); ++i)
    {
        Phys::Kinematics kin = m_physics.BodyKinematics(i);
        Anim::Mat4 M = Anim::ToMat4(m_physics.State(i).R, kin.x);
        SceneObject& obj = m_activeScene.objects[m_physicsBodyObjects[i]];
        obj.modelOverride = glm::make_mat4(M.m);
        obj.position = ToGlm(kin.x);
    }
}

void Renderer::ResetPhysics()
{
    m_physics.Reset();
    for (int i = 0; i < static_cast<int>(m_physicsAnchorObjects.size()); ++i)
        m_activeScene.objects[m_physicsAnchorObjects[i]].position = ToGlm(m_physics.InitialAnchor(i));
    std::fill(std::begin(m_energyHistory), std::end(m_energyHistory), 0.0f);
    m_energyHistoryOffset = 0;
    SyncPhysicsObjects();
}

void Renderer::UpdatePhysics(float deltaSeconds)
{
    if (!m_physicsActive)
        return;

    for (int i = 0; i < static_cast<int>(m_physicsAnchorObjects.size()); ++i)
        m_physics.SetAnchorTarget(i, ToAnim(m_activeScene.objects[m_physicsAnchorObjects[i]].position));

    auto start = std::chrono::steady_clock::now();
    if (!m_physicsPaused)
    {
        m_physicsStepsLastFrame = m_physics.Advance(deltaSeconds);
    }
    else if (m_physicsStepRequested)
    {
        m_physics.StepOnce();
        m_physicsStepsLastFrame = 1;
    }
    else
    {
        m_physicsStepsLastFrame = 0;
    }
    m_physicsStepRequested = false;
    m_physicsStepMs =
        std::chrono::duration<float, std::milli>(std::chrono::steady_clock::now() - start).count();

    if (m_physicsStepsLastFrame > 0)
    {
        m_energyHistory[m_energyHistoryOffset] = m_physics.Energy().Total();
        m_energyHistoryOffset = (m_energyHistoryOffset + 1) % kEnergyHistoryCount;
    }

    SyncPhysicsObjects();
}

void Renderer::DestroySpringLineBuffer()
{
    if (m_springLineVBO)
    {
        glDeleteBuffers(1, &m_springLineVBO);
        m_springLineVBO = 0;
    }
    if (m_springLineVAO)
    {
        glDeleteVertexArrays(1, &m_springLineVAO);
        m_springLineVAO = 0;
    }
}

void Renderer::DrawSpringLines(const Camera& camera)
{
    if (!m_physicsActive || !m_showSprings || m_physics.SpringCount() == 0)
        return;

    if (m_springLineVAO == 0)
    {
        glGenVertexArrays(1, &m_springLineVAO);
        glGenBuffers(1, &m_springLineVBO);
        glBindVertexArray(m_springLineVAO);
        glBindBuffer(GL_ARRAY_BUFFER, m_springLineVBO);
        glEnableVertexAttribArray(0);
        glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), static_cast<void*>(0));
        glBindBuffer(GL_ARRAY_BUFFER, 0);
        glBindVertexArray(0);
    }

    std::vector<float> vertices;
    vertices.reserve(static_cast<size_t>(m_physics.SpringCount()) * 6);
    for (int i = 0; i < m_physics.SpringCount(); ++i)
    {
        const Phys::Spring& s = m_physics.GetSpring(i);
        Anim::Vec3 a = m_physics.EndPositionNow(s.a);
        Anim::Vec3 b = m_physics.EndPositionNow(s.b);
        vertices.insert(vertices.end(), {a.x, a.y, a.z, b.x, b.y, b.z});
    }

    glBindBuffer(GL_ARRAY_BUFFER, m_springLineVBO);
    glBufferData(GL_ARRAY_BUFFER,
                 static_cast<GLsizeiptr>(vertices.size() * sizeof(float)),
                 vertices.data(),
                 GL_DYNAMIC_DRAW);
    glBindBuffer(GL_ARRAY_BUFFER, 0);

    glDisable(GL_DEPTH_TEST);
    glDisable(GL_CULL_FACE);

    m_boneLineShader.Bind();
    m_boneLineShader.SetMat4("uModel", glm::mat4(1.0f));
    m_boneLineShader.SetMat4("uView", camera.GetView());
    m_boneLineShader.SetMat4("uProj", camera.GetProj());
    m_boneLineShader.SetVec3("uColor", glm::vec3(0.95f, 0.95f, 0.85f));

    glBindVertexArray(m_springLineVAO);
    glDrawArrays(GL_LINES, 0, static_cast<GLsizei>(vertices.size() / 3));
    glBindVertexArray(0);

    m_boneLineShader.Unbind();
    glEnable(GL_DEPTH_TEST);
}

void Renderer::DrawPhysicsPanel()
{
    if (!m_ready)
        return;

    ImGui::Begin("Physics");
    if (!m_physicsActive)
    {
        ImGui::TextDisabled("%s",
                            m_physicsStatus.empty() ? "No physics in this scene" : m_physicsStatus.c_str());
        ImGui::End();
        return;
    }

    Phys::PhysicsSettings& settings = m_physics.settings;

    int integrator = static_cast<int>(settings.integrator);
    const char* names[Phys::kIntegratorCount];
    for (int i = 0; i < Phys::kIntegratorCount; ++i)
        names[i] = Phys::IntegratorName(static_cast<Phys::IntegratorType>(i));
    if (ImGui::Combo("Integrator", &integrator, names, Phys::kIntegratorCount))
        settings.integrator = static_cast<Phys::IntegratorType>(integrator);

    float rate = 1.0f / settings.dt;
    if (ImGui::SliderFloat("Step rate (Hz)", &rate, 30.0f, 2000.0f, "%.0f", ImGuiSliderFlags_Logarithmic))
        settings.dt = 1.0f / std::max(rate, 1.0f);
    ImGui::SliderInt("Max steps / frame", &settings.maxStepsPerFrame, 1, 256);

    ImGui::Checkbox("Pause", &m_physicsPaused);
    ImGui::SameLine();
    ImGui::BeginDisabled(!m_physicsPaused);
    if (ImGui::Button("Step"))
        m_physicsStepRequested = true;
    ImGui::EndDisabled();
    ImGui::SameLine();
    if (ImGui::Button("Reset"))
        ResetPhysics();
    ImGui::SameLine();
    ImGui::Checkbox("Springs", &m_showSprings);

    ImGui::SliderFloat("Gravity Y", &settings.gravity.y, -20.0f, 0.0f, "%.2f");
    ImGui::SliderFloat("Linear drag", &settings.linearDrag, 0.0f, 2.0f, "%.3f");
    ImGui::SliderFloat("Angular drag", &settings.angularDrag, 0.0f, 2.0f, "%.3f");
    bool springsChanged =
        ImGui::SliderFloat("Stiffness", &m_physicsStiffness, 10.0f, 5000.0f, "%.0f", ImGuiSliderFlags_Logarithmic);
    springsChanged |= ImGui::SliderFloat("Damping", &m_physicsDamping, 0.0f, 50.0f, "%.2f");
    if (springsChanged)
    {
        for (Phys::Spring& s : m_physics.Springs())
        {
            s.stiffness = m_physicsStiffness;
            s.damping = m_physicsDamping;
        }
    }

    ImGui::Separator();
    Phys::EnergyReport e = m_physics.Energy();
    Anim::Vec3 P = m_physics.TotalLinearMomentum();
    Anim::Vec3 L = m_physics.TotalAngularMomentum();
    ImGui::Text("t = %.2f s, %lld steps, %d this frame (%.3f ms)",
                m_physics.Time(),
                m_physics.StepCount(),
                m_physicsStepsLastFrame,
                m_physicsStepMs);
    ImGui::Text("Energy: kinetic %.4f  gravity %.4f  elastic %.4f  total %.4f J",
                e.kinetic,
                e.gravitational,
                e.elastic,
                e.Total());
    ImGui::Text("|P| = %.4f kg m/s   |L| = %.4f kg m^2/s", Anim::Length(P), Anim::Length(L));
    ImGui::PlotLines("##energy",
                     m_energyHistory,
                     kEnergyHistoryCount,
                     m_energyHistoryOffset,
                     "total energy",
                     FLT_MAX,
                     FLT_MAX,
                     ImVec2(0, 60));

    ImGui::Separator();
    if (m_physics.BodyCount() > 0)
    {
        m_physicsSelectedBody = std::clamp(m_physicsSelectedBody, 0, m_physics.BodyCount() - 1);
        if (ImGui::BeginCombo("Body", m_physics.Body(m_physicsSelectedBody).name.c_str()))
        {
            for (int i = 0; i < m_physics.BodyCount(); ++i)
                if (ImGui::Selectable(m_physics.Body(i).name.c_str(), i == m_physicsSelectedBody))
                    m_physicsSelectedBody = i;
            ImGui::EndCombo();
        }
        const Phys::RigidBodyState& s = m_physics.State(m_physicsSelectedBody);
        const Phys::RigidBodyProperties& props = m_physics.Properties(m_physicsSelectedBody);
        Phys::Kinematics kin = m_physics.BodyKinematics(m_physicsSelectedBody);
        ImGui::Text("mass %.3f kg   I_obj diag (%.4f, %.4f, %.4f)",
                    props.mass,
                    Anim::At(props.iObj, 0, 0),
                    Anim::At(props.iObj, 1, 1),
                    Anim::At(props.iObj, 2, 2));
        ImGui::Text("c     (%.3f, %.3f, %.3f)", s.c.x, s.c.y, s.c.z);
        ImGui::Text("c_dot (%.3f, %.3f, %.3f)", kin.cDot.x, kin.cDot.y, kin.cDot.z);
        ImGui::Text("w     (%.3f, %.3f, %.3f)", kin.omega.x, kin.omega.y, kin.omega.z);
        ImGui::Text("P     (%.3f, %.3f, %.3f)", s.P.x, s.P.y, s.P.z);
        ImGui::Text("L     (%.4f, %.4f, %.4f)", s.L.x, s.L.y, s.L.z);
    }

    ImGui::End();
}
