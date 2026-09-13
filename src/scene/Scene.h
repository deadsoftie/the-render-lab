#pragma once
#include <glm/glm.hpp>
#include <string>
#include <vector>

enum class ObjectRole
{
    None,
    Cornell,
    Ground,
    TallCube,
    ShortCube,
    SmallCube,
    Sphere,
    Probe
};

struct SceneObject
{
    std::string meshRef;
    ObjectRole role = ObjectRole::None;
    glm::vec3 position{0.0f};
    glm::vec3 scale{1.0f};
    float alpha = 64.0f;  // Probe role only: per-object Phong shininess
};

struct Scene
{
    std::string name;
    std::vector<SceneObject> objects;
};
