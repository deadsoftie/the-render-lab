#pragma once
#include <glm/glm.hpp>

class Camera
{
   public:
    Camera() = default;

    // --- Configuration ---
    void SetPerspective(float fovYRadians, float nearZ, float farZ);
    void SetViewport(int width, int height);

    void SetPosition(const glm::vec3& pos);
    void SetTarget(const glm::vec3& target);
    void SetUp(const glm::vec3& up);

    // --- Queries ---
    const glm::vec3& GetPosition() const { return m_position; }
    const glm::vec3& GetTarget() const { return m_target; }

    float GetAspect() const;
    glm::mat4 GetView() const;
    glm::mat4 GetProj() const;

    float GetFovY() const { return m_fovY; }  // radians

   private:
    glm::vec3 m_position{0.f, 0.f, 2.f};
    glm::vec3 m_target{0.f, 0.f, 0.f};
    glm::vec3 m_up{0.f, 1.f, 0.f};

    float m_fovY = glm::radians(60.0f);
    float m_nearZ = 0.1f;
    float m_farZ = 100.0f;

    int m_viewportW = 1280;
    int m_viewportH = 720;
};
