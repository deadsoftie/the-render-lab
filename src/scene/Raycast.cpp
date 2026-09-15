#include "pch.h"
#include "Raycast.h"

#include <algorithm>
#include <cmath>
#include <limits>

namespace
{
    bool RayIntersectsAABB(const glm::vec3& orig, const glm::vec3& invDir, const glm::vec3& mn,
                           const glm::vec3& mx)
    {
        float tmin = std::numeric_limits<float>::lowest();
        float tmax = std::numeric_limits<float>::max();
        for (int i = 0; i < 3; ++i)
        {
            float a = (mn[i] - orig[i]) * invDir[i];
            float b = (mx[i] - orig[i]) * invDir[i];
            tmin = std::max(tmin, std::min(a, b));
            tmax = std::min(tmax, std::max(a, b));
        }
        return tmax >= std::max(tmin, 0.0f);
    }

    bool RayIntersectsTriangle(const glm::vec3& orig, const glm::vec3& dir, const glm::vec3& v0,
                               const glm::vec3& v1, const glm::vec3& v2, float& outT)
    {
        constexpr float kEpsilon = 1e-6f;
        glm::vec3 edge1 = v1 - v0;
        glm::vec3 edge2 = v2 - v0;
        glm::vec3 h = glm::cross(dir, edge2);
        float a = glm::dot(edge1, h);
        if (std::fabs(a) < kEpsilon)
            return false;

        float f = 1.0f / a;
        glm::vec3 s = orig - v0;
        float u = f * glm::dot(s, h);
        if (u < 0.0f || u > 1.0f)
            return false;

        glm::vec3 q = glm::cross(s, edge1);
        float v = f * glm::dot(dir, q);
        if (v < 0.0f || u + v > 1.0f)
            return false;

        float t = f * glm::dot(edge2, q);
        if (t < kEpsilon)
            return false;

        outT = t;
        return true;
    }
}

RaycastMesh BuildRaycastMesh(const std::vector<float>& interleavedVerts,
                             const std::vector<unsigned int>& indices)
{
    // Stride matches Geometry::MeshData: pos(3) + nrm(3) + uv(2) + tan(3) = 11 floats.
    constexpr size_t kStride = 11;

    RaycastMesh rm;
    rm.indices = indices;
    rm.positions.reserve(interleavedVerts.size() / kStride);

    glm::vec3 mn(std::numeric_limits<float>::max());
    glm::vec3 mx(std::numeric_limits<float>::lowest());
    for (size_t i = 0; i + kStride - 1 < interleavedVerts.size(); i += kStride)
    {
        glm::vec3 p(interleavedVerts[i], interleavedVerts[i + 1], interleavedVerts[i + 2]);
        rm.positions.push_back(p);
        mn = glm::min(mn, p);
        mx = glm::max(mx, p);
    }
    rm.localMin = mn;
    rm.localMax = mx;
    return rm;
}

bool RaycastScene(const Scene& scene, const std::unordered_map<std::string, RaycastMesh>& meshes,
                  const glm::vec3& rayOrigin, const glm::vec3& rayDir, int& outIndex)
{
    bool found = false;
    float bestWorldT = std::numeric_limits<float>::max();

    for (size_t i = 0; i < scene.objects.size(); ++i)
    {
        const SceneObject& obj = scene.objects[i];
        if (!obj.visible)
            continue;

        constexpr float kMinScale = 1e-6f;
        if (std::fabs(obj.scale.x) < kMinScale || std::fabs(obj.scale.y) < kMinScale ||
            std::fabs(obj.scale.z) < kMinScale)
            continue;

        auto it = meshes.find(obj.meshRef);
        if (it == meshes.end())
            continue;
        const RaycastMesh& rm = it->second;

        glm::mat4 M = ComputeModelMatrix(obj);
        glm::mat4 invM = glm::inverse(M);

        glm::vec3 localOrigin = glm::vec3(invM * glm::vec4(rayOrigin, 1.0f));
        glm::vec3 localDir = glm::normalize(glm::vec3(invM * glm::vec4(rayDir, 0.0f)));

        // Avoid true infinities in the slab test below (0*inf is NaN) by keeping
        // near-axis-aligned local ray components a hair off exact zero.
        constexpr float kDirEpsilon = 1e-8f;
        if (std::fabs(localDir.x) < kDirEpsilon) localDir.x = kDirEpsilon;
        if (std::fabs(localDir.y) < kDirEpsilon) localDir.y = kDirEpsilon;
        if (std::fabs(localDir.z) < kDirEpsilon) localDir.z = kDirEpsilon;
        glm::vec3 invDir(1.0f / localDir.x, 1.0f / localDir.y, 1.0f / localDir.z);

        if (!RayIntersectsAABB(localOrigin, invDir, rm.localMin, rm.localMax))
            continue;

        for (size_t f = 0; f + 2 < rm.indices.size(); f += 3)
        {
            const glm::vec3& v0 = rm.positions[rm.indices[f]];
            const glm::vec3& v1 = rm.positions[rm.indices[f + 1]];
            const glm::vec3& v2 = rm.positions[rm.indices[f + 2]];

            float localT;
            if (!RayIntersectsTriangle(localOrigin, localDir, v0, v1, v2, localT))
                continue;

            glm::vec3 worldHit = glm::vec3(M * glm::vec4(localOrigin + localDir * localT, 1.0f));
            float worldT = glm::length(worldHit - rayOrigin);
            if (worldT < bestWorldT)
            {
                bestWorldT = worldT;
                outIndex = static_cast<int>(i);
                found = true;
            }
        }
    }

    return found;
}
