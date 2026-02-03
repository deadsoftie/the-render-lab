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
    void SetOrbitSpeed(float s) { mOrbitSpeed = s; }  // radians per pixel scale
    void SetPanSpeed(float s) { mPanSpeed = s; }      // world units per pixel scale
    void SetZoomSpeed(float s) { mZoomSpeed = s; }    // distance units per scroll step
    void SetDistanceLimits(float minD, float maxD)
    {
        mMinDist = minD;
        mMaxDist = maxD;
    }

    void SetTarget(const glm::vec3& t) { mTarget = t; }
    glm::vec3 GetTarget() const { return mTarget; }

   private:
    // Orbit state
    float mYaw = 0.0f;
    float mPitch = 0.0f;
    float mDistance = 3.0f;

    glm::vec3 mTarget{0.0f, 0.0f, 0.0f};

    // Tune-able values
    float mOrbitSpeed = 0.0075f;
    float mPanSpeed = 0.0020f;
    float mZoomSpeed = 0.35f;

    float mMinDist = 0.25f;
    float mMaxDist = 50.0f;

    static void ClampPitch(float& pitch);
};