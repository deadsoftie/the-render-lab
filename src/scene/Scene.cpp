#include "pch.h"
#include "Scene.h"

#include <glm/gtc/matrix_transform.hpp>

glm::mat4 ComputeModelMatrix(const SceneObject& obj)
{
    glm::mat4 M = glm::translate(glm::mat4(1.0f), obj.position);
    M = glm::rotate(M, glm::radians(obj.rotationEulerDegrees.x), glm::vec3(1.0f, 0.0f, 0.0f));
    M = glm::rotate(M, glm::radians(obj.rotationEulerDegrees.y), glm::vec3(0.0f, 1.0f, 0.0f));
    M = glm::rotate(M, glm::radians(obj.rotationEulerDegrees.z), glm::vec3(0.0f, 0.0f, 1.0f));
    M = glm::scale(M, obj.scale);
    return M;
}
