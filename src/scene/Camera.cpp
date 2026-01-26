#include "pch.h"
#include "Scene/Camera.h"

#include <glm/gtc/matrix_transform.hpp>

void Camera::SetPerspective(float fovYRadians, float nearZ, float farZ)
{
    m_fovY = fovYRadians;
    m_nearZ = nearZ;
    m_farZ = farZ;
}

void Camera::SetViewport(int width, int height)
{
    m_viewportW = (width > 0) ? width : 1;
    m_viewportH = (height > 0) ? height : 1;
}

void Camera::SetPosition(const glm::vec3& pos)
{
    m_position = pos;
}

void Camera::SetTarget(const glm::vec3& target)
{
    m_target = target;
}

void Camera::SetUp(const glm::vec3& up)
{
    m_up = up;
}

float Camera::GetAspect() const
{
    return static_cast<float>(m_viewportW) / static_cast<float>(m_viewportH);
}

glm::mat4 Camera::GetView() const
{
    return glm::lookAt(m_position, m_target, m_up);
}

glm::mat4 Camera::GetProj() const
{
    return glm::perspective(m_fovY, GetAspect(), m_nearZ, m_farZ);
}
