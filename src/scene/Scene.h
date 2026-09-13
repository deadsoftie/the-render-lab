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

struct SceneLight
{
    glm::vec3 position{0.0f};
    glm::vec3 color{1.0f};
    float range = 1.5f;
    float intensity = 1.0f;
    bool enabled = true;
};

struct ScenePipeline
{
    std::string hdriFile;  // empty = PBS, no HDRI
    bool shadowsEnabled = true;
    bool useMSM = true;
    bool aoEnabled = true;
    bool celEnabled = false;
    bool useLightVolumes = false;
    float exposure = 2.5f;
    float hdriRotation = 0.0f;
};

struct Scene
{
    std::string name;
    std::vector<SceneObject> objects;
    std::vector<SceneLight> lights;
    ScenePipeline pipeline;
};
