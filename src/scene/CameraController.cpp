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
    mTarget = camera.GetTarget();

    glm::vec3 toCam = camera.GetPosition() - mTarget;
    mDistance = glm::length(toCam);
    if (mDistance < 1e-4f)
        mDistance = 1.0f;

    glm::vec3 dir = toCam / mDistance;

    mPitch = asinf(std::clamp(dir.y, -1.0f, 1.0f));
    mYaw = atan2f(dir.z, dir.x);

    ClampPitch(mPitch);
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

    // Zoom (scroll): adjust distance
    if (sy != 0.0)
    {
        // Positive scroll usually means "up" -> zoom in
        mDistance -= static_cast<float>(sy) * mZoomSpeed;
        mDistance = std::clamp(mDistance, mMinDist, mMaxDist);
    }

    // Compute current camera basis from yaw/pitch
    // Camera position will be target + dir*dist
    if (orbiting)
    {
        // Drag right => yaw increases, drag up => pitch increases (invert if you prefer)
        mYaw += static_cast<float>(mdx) * mOrbitSpeed;
        mPitch += static_cast<float>(mdy) * mOrbitSpeed;
        ClampPitch(mPitch);
    }

    // Build the camera position from spherical coords
    glm::vec3 offset = SphericalToCartesian(mYaw, mPitch, mDistance);
    glm::vec3 camPos = mTarget + offset;

    // Pan: move target and camera together in view plane
    if (panning)
    {
        // View direction (from camera to target)
        glm::vec3 forward = glm::normalize(mTarget - camPos);

        // Right and Up in world
        glm::vec3 worldUp(0.0f, 1.0f, 0.0f);
        glm::vec3 right = glm::normalize(glm::cross(forward, worldUp));
        glm::vec3 up = glm::normalize(glm::cross(right, forward));

        // Scale pan with distance and viewport size for consistent feel
        float scale = mPanSpeed * mDistance;

        float vw = (viewportW > 0) ? static_cast<float>(viewportW) : 1.0f;
        float vh = (viewportH > 0) ? static_cast<float>(viewportH) : 1.0f;

        // Convert pixels to normalized-ish movement
        float dx = static_cast<float>(mdx) / vw;
        float dy = static_cast<float>(mdy) / vh;

        // Drag right -> pan right, drag up -> pan up
        glm::vec3 pan = (-right * dx + up * dy) * scale * 1000.0f;
        // ^ 1000 is just a feel constant to make default pan not too tiny.
        // You can remove it and instead increase m_panSpeed.

        mTarget += pan;
        camPos += pan;
    }

    // Apply to camera
    camera.SetTarget(mTarget);
    camera.SetPosition(camPos);
    camera.SetUp(glm::vec3(0.0f, 1.0f, 0.0f));
    camera.SetViewport(viewportW, viewportH);
}
