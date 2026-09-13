#pragma once
#include <glm/glm.hpp>
#include <string>
#include <unordered_map>
#include <vector>

#include "Scene.h"

struct RaycastMesh
{
    std::vector<glm::vec3> positions;
    std::vector<unsigned int> indices;
    glm::vec3 localMin{0.0f};
    glm::vec3 localMax{0.0f};
};

// Interleaved layout is pos(3)+nrm(3); picking only needs positions.
RaycastMesh BuildRaycastMesh(const std::vector<float>& interleavedVerts,
                             const std::vector<unsigned int>& indices);

bool RaycastScene(const Scene& scene, const std::unordered_map<std::string, RaycastMesh>& meshes,
                  const glm::vec3& rayOrigin, const glm::vec3& rayDir, int& outIndex);
