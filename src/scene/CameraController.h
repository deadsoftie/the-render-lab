#pragma once
#include <glm/glm.hpp>

class Camera;

class CameraController
{
   public:
    void InitializeFromCamera(const Camera& camera);

    // Per-frame update
    // dt is optional here; included for future smoothing
    void Update(Camera& camera, float dt, int viewportW, int viewportH);

    // Settings
    void SetOrbitSpeed(float s) { m_orbitSpeed = s; }  // radians per pixel scale
    void SetPanSpeed(float s) { m_panSpeed = s; }      // world units per pixel scale
    void SetZoomSpeed(float s) { m_zoomSpeed = s; }    // distance units per scroll step
    void SetDistanceLimits(float minD, float maxD)
    {
        m_minDist = minD;
        m_maxDist = maxD;
    }

    void SetTarget(const glm::vec3& t) { m_target = t; }
    glm::vec3 GetTarget() const { return m_target; }

   private:
    bool m_wasOrbiting = false;

    // Orbit state
    float m_yaw = 0.0f;
    float m_pitch = 0.0f;
    float m_distance = 3.0f;

    glm::vec3 m_target{0.0f, 0.0f, 0.0f};

    // Tune-able values
    float m_orbitSpeed = 0.0075f;
    float m_panSpeed = 1.0f;
    float m_zoomSpeed = 0.35f;

    float m_minDist = 0.25f;
    float m_maxDist = 50.0f;

    static void ClampPitch(float& pitch);
};