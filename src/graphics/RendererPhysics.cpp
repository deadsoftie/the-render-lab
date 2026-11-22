#include "pch.h"
#include <algorithm>
#include <cfloat>
#include <chrono>
#include <cmath>
#include <ImGuizmo.h>
#include <glad/glad.h>
#include <glm/ext/matrix_transform.hpp>
#include <glm/gtc/constants.hpp>
#include <glm/gtc/type_ptr.hpp>

#include "graphics/Renderer.h"
#include "input/Input.h"

namespace
{
    constexpr int kCoilTurns = 10;
    constexpr int kCoilSegmentsPerTurn = 10;
    constexpr float kCoilRadius = 0.012f;
    constexpr float kCoilLeadFraction = 0.12f;
    constexpr float kAnchorNudgeSpeed = 1.0f;
    constexpr float kForceArrowScale = 0.01f;
    constexpr float kFluidForceArrowScale = 0.005f;
    constexpr float kOmegaArrowScale = 0.05f;
    constexpr float kAxisLength = 0.15f;
    constexpr float kMarkerSize = 0.03f;
    constexpr float kKickSpeed = 2.0f;

    Anim::Vec3 ToAnim(const glm::vec3& v)
    {
        return Anim::Vec3{v.x, v.y, v.z};
    }

    glm::vec3 ToGlm(const Anim::Vec3& v)
    {
        return glm::vec3(v.x, v.y, v.z);
    }

    glm::mat3 ToGlm(const Anim::Mat3& m)
    {
        return glm::make_mat3(m.m);
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

    glm::vec3 StrainColor(float length, float restLength)
    {
        float strain = (length - restLength) / std::max(restLength, 1e-3f);
        const glm::vec3 rest(0.95f, 0.95f, 0.9f);
        if (strain < 0.0f)
            return glm::mix(rest, glm::vec3(0.2f, 0.45f, 1.0f), std::min(-strain / 0.25f, 1.0f));
        return glm::mix(rest, glm::vec3(1.0f, 0.2f, 0.12f), std::min(strain / 0.75f, 1.0f));
    }

    void PushLine(std::vector<float>& out, const glm::vec3& a, const glm::vec3& b, const glm::vec3& color)
    {
        out.insert(out.end(), {a.x, a.y, a.z, color.r, color.g, color.b, b.x, b.y, b.z, color.r, color.g, color.b});
    }

    void PushCross(std::vector<float>& out, const glm::vec3& p, float size, const glm::vec3& color)
    {
        PushLine(out, p - glm::vec3(size, 0, 0), p + glm::vec3(size, 0, 0), color);
        PushLine(out, p - glm::vec3(0, size, 0), p + glm::vec3(0, size, 0), color);
        PushLine(out, p - glm::vec3(0, 0, size), p + glm::vec3(0, 0, size), color);
    }

    void PushCoil(std::vector<float>& out, const glm::vec3& a, const glm::vec3& b, const glm::vec3& color)
    {
        glm::vec3 d = b - a;
        float len = glm::length(d);
        if (len < 1e-4f)
            return;
        glm::vec3 u = d / len;
        glm::vec3 helper = std::fabs(u.y) < 0.9f ? glm::vec3(0, 1, 0) : glm::vec3(1, 0, 0);
        glm::vec3 e1 = glm::normalize(glm::cross(u, helper));
        glm::vec3 e2 = glm::cross(u, e1);
        float radius = std::min(kCoilRadius, 0.25f * len);

        const int segments = kCoilTurns * kCoilSegmentsPerTurn;
        glm::vec3 coilStart = a + d * kCoilLeadFraction;
        glm::vec3 coilSpan = d * (1.0f - 2.0f * kCoilLeadFraction);
        glm::vec3 previous = coilStart;
        PushLine(out, a, coilStart, color);
        for (int k = 1; k <= segments; ++k)
        {
            float t = static_cast<float>(k) / segments;
            float angle = glm::two_pi<float>() * kCoilTurns * t;
            float envelope = std::sin(glm::pi<float>() * t);
            glm::vec3 p = coilStart + coilSpan * t + (e1 * std::cos(angle) + e2 * std::sin(angle)) * (radius * std::sqrt(envelope));
            PushLine(out, previous, p, color);
            previous = p;
        }
        PushLine(out, previous, b, color);
    }

    void PushRing(std::vector<float>& out, const glm::vec3& center, float radius, const glm::vec3& color)
    {
        constexpr int kSegments = 32;
        for (int k = 0; k < kSegments; ++k)
        {
            float a0 = glm::two_pi<float>() * k / kSegments;
            float a1 = glm::two_pi<float>() * (k + 1) / kSegments;
            PushLine(out,
                     center + radius * glm::vec3(std::cos(a0), 0.0f, std::sin(a0)),
                     center + radius * glm::vec3(std::cos(a1), 0.0f, std::sin(a1)),
                     color);
        }
    }

    bool RayBox(const glm::vec3& origin, const glm::vec3& dir, const glm::vec3& half, float& tHit)
    {
        float tMin = 0.0f;
        float tMax = FLT_MAX;
        for (int axis = 0; axis < 3; ++axis)
        {
            if (std::fabs(dir[axis]) < 1e-8f)
            {
                if (origin[axis] < -half[axis] || origin[axis] > half[axis])
                    return false;
                continue;
            }
            float inv = 1.0f / dir[axis];
            float t0 = (-half[axis] - origin[axis]) * inv;
            float t1 = (half[axis] - origin[axis]) * inv;
            if (t0 > t1)
                std::swap(t0, t1);
            tMin = std::max(tMin, t0);
            tMax = std::min(tMax, t1);
            if (tMin > tMax)
                return false;
        }
        tHit = tMin;
        return true;
    }
}

void Renderer::LoadPhysics()
{
    m_physics.Clear();
    m_physicsBodyObjects.clear();
    m_physicsAnchorObjects.clear();
    m_physicsFirstObject = -1;
    m_physicsActive = false;
    m_physicsPaused = false;
    m_physicsStepRequested = false;
    m_mouseDragActive = false;
    m_physicsStatus.clear();
    m_physicsSelectedBody = 0;
    std::fill(std::begin(m_energyHistory), std::end(m_energyHistory), 0.0f);
    std::fill(std::begin(m_momentumHistory), std::end(m_momentumHistory), 0.0f);
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

    m_physicsFirstObject = static_cast<int>(m_activeScene.objects.size());
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

void Renderer::RebuildPhysics()
{
    Phys::PhysicsSettings settings = m_physics.settings;
    float stiffness = m_physicsStiffness;
    float damping = m_physicsDamping;

    PhysicsChainBinding& chain = m_activeScene.physics.chain;
    if (chain.enabled && m_physicsAnchorObjects.size() >= 2)
    {
        chain.anchorA = m_activeScene.objects[m_physicsAnchorObjects[0]].position;
        chain.anchorB = m_activeScene.objects[m_physicsAnchorObjects[1]].position;
    }

    if (m_physicsFirstObject >= 0 && m_physicsFirstObject <= static_cast<int>(m_activeScene.objects.size()))
    {
        m_activeScene.objects.erase(m_activeScene.objects.begin() + m_physicsFirstObject, m_activeScene.objects.end());
        if (m_selection.kind == SelectionKind::Object && m_selection.index >= m_physicsFirstObject)
            m_selection = Selection{};
    }

    LoadPhysics();
    if (!m_physicsActive)
        return;

    m_physics.settings = settings;
    m_physicsStiffness = stiffness;
    m_physicsDamping = damping;
    for (Phys::Spring& s : m_physics.Springs())
    {
        s.stiffness = stiffness;
        s.damping = damping;
    }
}

int Renderer::PhysicsBodyForObject(int objectIndex) const
{
    for (size_t i = 0; i < m_physicsBodyObjects.size(); ++i)
        if (m_physicsBodyObjects[i] == objectIndex)
            return static_cast<int>(i);
    return -1;
}

int Renderer::PhysicsAnchorForObject(int objectIndex) const
{
    for (size_t i = 0; i < m_physicsAnchorObjects.size(); ++i)
        if (m_physicsAnchorObjects[i] == objectIndex)
            return static_cast<int>(i);
    return -1;
}

bool Renderer::PickPhysicsBody(const glm::vec3& origin,
                               const glm::vec3& direction,
                               int& body,
                               glm::vec3& hit,
                               Anim::Vec3& localPoint) const
{
    float best = FLT_MAX;
    body = -1;
    for (int i = 0; i < m_physics.BodyCount(); ++i)
    {
        Phys::Kinematics kin = m_physics.BodyKinematics(i);
        glm::mat3 R = ToGlm(m_physics.State(i).R);
        glm::mat3 Rt = glm::transpose(R);
        glm::vec3 x = ToGlm(kin.x);
        glm::vec3 localOrigin = Rt * (origin - x);
        glm::vec3 localDir = Rt * direction;
        float t = 0.0f;
        if (RayBox(localOrigin, localDir, ToGlm(m_physics.Body(i).halfExtents), t) && t < best)
        {
            best = t;
            body = i;
            localPoint = ToAnim(localOrigin + localDir * t);
            hit = origin + direction * t;
        }
    }
    return body >= 0;
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
    m_mouseDragActive = false;
    for (int i = 0; i < static_cast<int>(m_physicsAnchorObjects.size()); ++i)
        m_activeScene.objects[m_physicsAnchorObjects[i]].position = ToGlm(m_physics.InitialAnchor(i));
    std::fill(std::begin(m_energyHistory), std::end(m_energyHistory), 0.0f);
    std::fill(std::begin(m_momentumHistory), std::end(m_momentumHistory), 0.0f);
    m_energyHistoryOffset = 0;
    SyncPhysicsObjects();
}

void Renderer::UpdatePhysicsInput(float deltaSeconds)
{
    ImGuiIO& io = ImGui::GetIO();

    if (m_mouseDragActive)
    {
        glm::vec3 origin, direction;
        if (!Input::MouseDown(GLFW_MOUSE_BUTTON_LEFT))
        {
            m_mouseDragActive = false;
            m_physics.mouse.active = false;
        }
        else if (MouseRay(origin, direction))
        {
            float denom = glm::dot(direction, m_mouseDragPlaneNormal);
            if (std::fabs(denom) > 1e-6f)
            {
                float t = glm::dot(m_mouseDragPlanePoint - origin, m_mouseDragPlaneNormal) / denom;
                if (t > 0.0f)
                    m_physics.mouse.target = ToAnim(origin + direction * t);
            }
        }
    }
    else if (io.KeyCtrl && Input::MousePressed(GLFW_MOUSE_BUTTON_LEFT) && !io.WantCaptureMouse &&
             !ImGuizmo::IsOver() && !ImGuizmo::IsUsing())
    {
        glm::vec3 origin, direction, hit;
        int body = -1;
        Anim::Vec3 local;
        if (MouseRay(origin, direction) && PickPhysicsBody(origin, direction, body, hit, local))
        {
            glm::mat4 cameraToWorld = glm::inverse(m_cachedView);
            m_mouseDragPlaneNormal = glm::normalize(-glm::vec3(cameraToWorld[2]));
            m_mouseDragPlanePoint = hit;
            m_mouseDragActive = true;
            m_physics.mouse.active = true;
            m_physics.mouse.body = body;
            m_physics.mouse.localPoint = local;
            m_physics.mouse.target = ToAnim(hit);
            m_physicsSelectedBody = body;
        }
    }

    if (io.WantCaptureKeyboard || m_selection.kind != SelectionKind::Object)
        return;
    int anchor = PhysicsAnchorForObject(m_selection.index);
    if (anchor < 0)
        return;

    glm::mat4 cameraToWorld = glm::inverse(m_cachedView);
    glm::vec3 forward(-cameraToWorld[2].x, 0.0f, -cameraToWorld[2].z);
    forward = glm::dot(forward, forward) > 1e-8f ? glm::normalize(forward) : glm::vec3(0.0f, 0.0f, -1.0f);
    glm::vec3 right(-forward.z, 0.0f, forward.x);

    glm::vec3 move(0.0f);
    if (Input::KeyDown(GLFW_KEY_RIGHT))
        move += right;
    if (Input::KeyDown(GLFW_KEY_LEFT))
        move -= right;
    if (Input::KeyDown(GLFW_KEY_UP))
        move += forward;
    if (Input::KeyDown(GLFW_KEY_DOWN))
        move -= forward;
    if (Input::KeyDown(GLFW_KEY_PAGE_UP))
        move.y += 1.0f;
    if (Input::KeyDown(GLFW_KEY_PAGE_DOWN))
        move.y -= 1.0f;
    if (glm::dot(move, move) > 0.0f)
        m_activeScene.objects[m_selection.index].position += glm::normalize(move) * (kAnchorNudgeSpeed * deltaSeconds);
}

void Renderer::UpdatePhysics(float deltaSeconds)
{
    if (!m_physicsActive)
        return;

    UpdatePhysicsInput(deltaSeconds);

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
        m_momentumHistory[m_energyHistoryOffset] = Anim::Length(m_physics.TotalLinearMomentum());
        m_energyHistoryOffset = (m_energyHistoryOffset + 1) % kEnergyHistoryCount;
    }

    SyncPhysicsObjects();
}

void Renderer::DestroyPhysicsLineBuffer()
{
    if (m_physLineVBO)
    {
        glDeleteBuffers(1, &m_physLineVBO);
        m_physLineVBO = 0;
    }
    if (m_physLineVAO)
    {
        glDeleteVertexArrays(1, &m_physLineVAO);
        m_physLineVAO = 0;
    }
}

void Renderer::DrawPhysicsOverlay(const Camera& camera, bool deferred)
{
    if (!m_physicsActive)
        return;

    if (m_physLineVAO == 0)
    {
        glGenVertexArrays(1, &m_physLineVAO);
        glGenBuffers(1, &m_physLineVBO);
        glBindVertexArray(m_physLineVAO);
        glBindBuffer(GL_ARRAY_BUFFER, m_physLineVBO);
        glEnableVertexAttribArray(0);
        glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float), static_cast<void*>(0));
        glEnableVertexAttribArray(1);
        glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float), reinterpret_cast<void*>(3 * sizeof(float)));
        glBindBuffer(GL_ARRAY_BUFFER, 0);
        glBindVertexArray(0);
    }

    std::vector<float>& v = m_physLineVertices;
    v.clear();

    if (m_showSprings)
    {
        for (int i = 0; i < m_physics.SpringCount(); ++i)
        {
            const Phys::Spring& s = m_physics.GetSpring(i);
            glm::vec3 a = ToGlm(m_physics.EndPositionNow(s.a));
            glm::vec3 b = ToGlm(m_physics.EndPositionNow(s.b));
            PushCoil(v, a, b, StrainColor(glm::length(b - a), s.restLength));
        }
    }
    for (int i = 0; i < static_cast<int>(m_physicsAnchorObjects.size()); ++i)
    {
        const SceneObject& obj = m_activeScene.objects[m_physicsAnchorObjects[i]];
        PushRing(v, obj.position, 1.6f * obj.scale.x, glm::vec3(1.0f, 0.55f, 0.15f));
    }
    if (m_physics.mouse.active)
    {
        const Phys::MouseSpring& ms = m_physics.mouse;
        Phys::Kinematics kin = m_physics.BodyKinematics(ms.body);
        glm::vec3 p = ToGlm(Phys::WorldPoint(kin, m_physics.State(ms.body), ms.localPoint));
        glm::vec3 target = ToGlm(ms.target);
        PushLine(v, p, target, glm::vec3(1.0f, 0.9f, 0.1f));
        PushCross(v, target, kMarkerSize, glm::vec3(1.0f, 0.9f, 0.1f));
    }
    size_t depthTestedFloats = v.size();

    for (int i = 0; i < m_physics.BodyCount(); ++i)
    {
        const Phys::RigidBodyState& s = m_physics.State(i);
        Phys::Kinematics kin = m_physics.BodyKinematics(i);
        glm::vec3 c = ToGlm(s.c);
        if (m_showCenterOfMass)
            PushCross(v, c, kMarkerSize, glm::vec3(1.0f, 1.0f, 0.2f));
        if (m_showBodyAxes)
        {
            PushLine(v, c, c + ToGlm(Anim::Column(s.R, 0)) * kAxisLength, glm::vec3(1.0f, 0.2f, 0.2f));
            PushLine(v, c, c + ToGlm(Anim::Column(s.R, 1)) * kAxisLength, glm::vec3(0.2f, 1.0f, 0.2f));
            PushLine(v, c, c + ToGlm(Anim::Column(s.R, 2)) * kAxisLength, glm::vec3(0.3f, 0.5f, 1.0f));
        }
        if (m_showAngularVelocity)
            PushLine(v, c, c + ToGlm(kin.omega) * kOmegaArrowScale, glm::vec3(1.0f, 0.3f, 1.0f));
        if (m_showBodyPoints)
        {
            Anim::Vec3 h = m_physics.Body(i).halfExtents;
            for (int corner = 0; corner < 8; ++corner)
            {
                Anim::Vec3 q{(corner & 1) ? h.x : -h.x, (corner & 2) ? h.y : -h.y, (corner & 4) ? h.z : -h.z};
                PushCross(v, ToGlm(Phys::WorldPoint(kin, s, q)), 0.5f * kMarkerSize, glm::vec3(0.2f, 1.0f, 1.0f));
            }
        }
    }
    if (m_showSpringForces)
    {
        for (int i = 0; i < m_physics.SpringCount(); ++i)
        {
            const Phys::Spring& s = m_physics.GetSpring(i);
            glm::vec3 f = ToGlm(m_physics.SpringForceOnA(i)) * kForceArrowScale;
            glm::vec3 color(0.3f, 1.0f, 0.4f);
            if (s.a.body >= 0)
            {
                glm::vec3 p = ToGlm(m_physics.EndPositionNow(s.a));
                PushLine(v, p, p + f, color);
            }
            if (s.b.body >= 0)
            {
                glm::vec3 p = ToGlm(m_physics.EndPositionNow(s.b));
                PushLine(v, p, p - f, color);
            }
        }
    }

    if (m_showFluidForces && m_pbf.IsLoaded())
    {
        for (int i = 0; i < m_physics.BodyCount(); ++i)
        {
            glm::vec3 c = ToGlm(m_physics.State(i).c);
            glm::vec3 f = ToGlm(m_physics.FluidForce(i).force);
            PushLine(v, c, c + f * kFluidForceArrowScale, glm::vec3(0.2f, 0.8f, 1.0f));
        }
    }

    if (v.empty())
        return;

    glBindBuffer(GL_ARRAY_BUFFER, m_physLineVBO);
    glBufferData(GL_ARRAY_BUFFER, static_cast<GLsizeiptr>(v.size() * sizeof(float)), v.data(), GL_DYNAMIC_DRAW);
    glBindBuffer(GL_ARRAY_BUFFER, 0);

    glDisable(GL_CULL_FACE);
    m_physLineShader.Bind();
    m_physLineShader.SetMat4("uView", camera.GetView());
    m_physLineShader.SetMat4("uProj", camera.GetProj());
    m_physLineShader.SetVec2("uViewport", glm::vec2(m_viewportW, m_viewportH));
    m_physLineShader.SetFloat("uDepthBias", 1e-5f);
    m_physLineShader.SetInt("uSceneDepth", 0);
    glActiveTexture(GL_TEXTURE0);
    glBindTexture(GL_TEXTURE_2D, m_gbuffer.TexDepth());
    glBindVertexArray(m_physLineVAO);

    GLsizei testedCount = static_cast<GLsizei>(depthTestedFloats / 6);
    GLsizei totalCount = static_cast<GLsizei>(v.size() / 6);
    if (testedCount > 0)
    {
        bool useSceneDepth = deferred && !m_springsOnTop;
        if (!deferred && !m_springsOnTop)
            glEnable(GL_DEPTH_TEST);
        else
            glDisable(GL_DEPTH_TEST);
        m_physLineShader.SetInt("uDepthTest", useSceneDepth ? 1 : 0);
        glDrawArrays(GL_LINES, 0, testedCount);
    }
    if (totalCount > testedCount)
    {
        glDisable(GL_DEPTH_TEST);
        m_physLineShader.SetInt("uDepthTest", 0);
        glDrawArrays(GL_LINES, testedCount, totalCount - testedCount);
    }

    glBindVertexArray(0);
    glBindTexture(GL_TEXTURE_2D, 0);
    m_physLineShader.Unbind();
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

    PhysicsChainBinding& chain = m_activeScene.physics.chain;
    if (chain.enabled)
    {
        float span = glm::length(chain.anchorB - chain.anchorA);
        int maxCount = std::max(1, static_cast<int>((span - 1e-3f) / (2.0f * chain.plankHalfExtents.x)));
        maxCount = std::min(maxCount, 30);
        int count = std::min(chain.count, maxCount);
        ImGui::SameLine();
        ImGui::SetNextItemWidth(120.0f);
        if (ImGui::SliderInt("Planks", &count, 1, maxCount) && count != chain.count)
        {
            chain.count = count;
            RebuildPhysics();
            ImGui::End();
            return;
        }
    }

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
    ImGui::SliderFloat("Mouse spring", &m_physics.mouse.stiffness, 10.0f, 1000.0f, "%.0f N/m", ImGuiSliderFlags_Logarithmic);

    ImGui::Checkbox("Springs", &m_showSprings);
    ImGui::SameLine();
    ImGui::Checkbox("On top", &m_springsOnTop);
    ImGui::SameLine();
    ImGui::Checkbox("COM", &m_showCenterOfMass);
    ImGui::SameLine();
    ImGui::Checkbox("Axes", &m_showBodyAxes);
    ImGui::SameLine();
    ImGui::Checkbox("Omega", &m_showAngularVelocity);
    ImGui::SameLine();
    ImGui::Checkbox("Forces", &m_showSpringForces);
    ImGui::SameLine();
    ImGui::Checkbox("q_i", &m_showBodyPoints);

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
    float plotWidth = ImGui::GetContentRegionAvail().x * 0.5f - 4.0f;
    ImGui::PlotLines("##energy",
                     m_energyHistory,
                     kEnergyHistoryCount,
                     m_energyHistoryOffset,
                     "total energy",
                     FLT_MAX,
                     FLT_MAX,
                     ImVec2(plotWidth, 60));
    ImGui::SameLine();
    ImGui::PlotLines("##momentum",
                     m_momentumHistory,
                     kEnergyHistoryCount,
                     m_energyHistoryOffset,
                     "|P|",
                     FLT_MAX,
                     FLT_MAX,
                     ImVec2(plotWidth, 60));

    ImGui::Separator();
    if (m_physics.BodyCount() > 0)
    {
        if (m_selection.kind == SelectionKind::Object)
        {
            int selectedBody = PhysicsBodyForObject(m_selection.index);
            if (selectedBody >= 0)
                m_physicsSelectedBody = selectedBody;
        }
        m_physicsSelectedBody = std::clamp(m_physicsSelectedBody, 0, m_physics.BodyCount() - 1);
        if (ImGui::BeginCombo("Body", m_physics.Body(m_physicsSelectedBody).name.c_str()))
        {
            for (int i = 0; i < m_physics.BodyCount(); ++i)
                if (ImGui::Selectable(m_physics.Body(i).name.c_str(), i == m_physicsSelectedBody))
                    m_physicsSelectedBody = i;
            ImGui::EndCombo();
        }
        ImGui::SameLine();
        if (ImGui::Button("Kick"))
        {
            const Phys::RigidBodyState& s = m_physics.State(m_physicsSelectedBody);
            Anim::Vec3 end = s.c + Anim::Column(s.R, 0) * m_physics.Body(m_physicsSelectedBody).halfExtents.x;
            Anim::Vec3 impulse = Anim::Vec3{0.0f, 1.0f, 0.3f} * (kKickSpeed * m_physics.Properties(m_physicsSelectedBody).mass);
            m_physics.ApplyImpulse(m_physicsSelectedBody, impulse, end);
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
