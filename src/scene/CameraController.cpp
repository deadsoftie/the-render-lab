#include "pch.h"
#include "CameraController.h"

#include "Camera.h"
#include "input/Input.h"

#include <glm/gtc/constants.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <algorithm>

// ImGui integration: ignore camera input when UI wants the mouse
#include <imgui.h>

static glm::vec3 SphericalToCartesian(float yaw, float pitch, float radius)
{
    // yaw: around +Y
    // pitch: up/down
    float cy = cosf(yaw);
    float sy = sinf(yaw);
    float cp = cosf(pitch);
    float sp = sinf(pitch);

    // Forward direction from target to camera (unit)
    // Z forward convention doesn't matter as long as consistent.
    glm::vec3 dir;
    dir.x = cy * cp;
    dir.y = sp;
    dir.z = sy * cp;

    return dir * radius;
}

void CameraController::ClampPitch(float& pitch)
{
    // Prevent flipping at poles
    const float limit = glm::half_pi<float>() - 0.001f;
    pitch = std::clamp(pitch, -limit, +limit);
}

void CameraController::InitializeFromCamera(const Camera& camera)
{
    m_target = camera.GetTarget();

    glm::vec3 toCam = camera.GetPosition() - m_target;
    m_distance = glm::length(toCam);
    if (m_distance < 1e-4f)
        m_distance = 1.0f;

    glm::vec3 dir = toCam / m_distance;

    m_pitch = asinf(std::clamp(dir.y, -1.0f, 1.0f));
    m_yaw = atan2f(dir.z, dir.x);

    ClampPitch(m_pitch);
}

void CameraController::Update(Camera& camera, float /*dt*/, int viewportW, int viewportH)
{
    // If ImGui is interacting with the mouse, ignore camera controls
    ImGuiIO& io = ImGui::GetIO();
    if (io.WantCaptureMouse)
        return;

    // Read deltas
    double mdx = 0.0, mdy = 0.0;
    Input::GetMouseDelta(mdx, mdy);

    double sx = 0.0, sy = 0.0;
    Input::GetScrollDelta(sx, sy);

    const bool orbiting = Input::MouseDown(GLFW_MOUSE_BUTTON_LEFT);
    const bool panning = Input::MouseDown(GLFW_MOUSE_BUTTON_RIGHT);
    const bool orbitBegin = orbiting && !m_wasOrbiting;

    glm::vec3 offset = SphericalToCartesian(m_yaw, m_pitch, m_distance);
    glm::vec3 camPos = m_target + offset;

    // Re-anchor orbit pivot when orbit begins: pivot becomes "focus point" in front of camera
    if (orbitBegin)
    {
        glm::vec3 forward = glm::normalize(m_target - camPos);  // towards target
        m_target = camPos + forward * m_distance;               // focus point in front
        camPos = m_target + offset;
    }

    // Zoom
    if (sy != 0.0)
    {
        m_distance -= static_cast<float>(sy) * m_zoomSpeed;
        m_distance = std::clamp(m_distance, m_minDist, m_maxDist);
    }

    // Orbit
    if (orbiting)
    {
        m_yaw += static_cast<float>(mdx) * m_orbitSpeed;
        m_pitch += static_cast<float>(mdy) * m_orbitSpeed;
        ClampPitch(m_pitch);

        offset = SphericalToCartesian(m_yaw, m_pitch, m_distance);
        camPos = m_target + offset;
    }

    // Pan
    if (panning)
    {
        glm::vec3 forward = glm::normalize(m_target - camPos);
        glm::vec3 worldUp(0.0f, 1.0f, 0.0f);
        glm::vec3 right = glm::normalize(glm::cross(forward, worldUp));
        glm::vec3 up = glm::normalize(glm::cross(right, forward));

        float vh = (viewportH > 0) ? static_cast<float>(viewportH) : 1.0f;

        const float worldPerPixel = (2.0f * m_distance * tanf(camera.GetFovY() * 0.5f)) / vh;

        glm::vec3 pan = (-right * static_cast<float>(mdx) + up * static_cast<float>(mdy)) *
                        (worldPerPixel * m_panSpeed);

        m_target += pan;
        camPos += pan;
    }

    // Apply to camera
    camera.SetTarget(m_target);
    camera.SetPosition(camPos);
    camera.SetUp(glm::vec3(0.0f, 1.0f, 0.0f));
    camera.SetViewport(viewportW, viewportH);

    m_wasOrbiting = orbiting;
}
