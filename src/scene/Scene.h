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

struct Material
{
    glm::vec3 kd{0.8f, 0.3f, 0.2f};  // diffuse
    glm::vec3 ks{0.04f};             // specular / F0
    float ambient = 0.08f;
    float alpha = 64.0f;  // Phong shininess exponent (1..256); used as PBS roughness
};

struct SceneObject
{
    std::string meshRef;
    ObjectRole role = ObjectRole::None;
    glm::vec3 position{0.0f};
    glm::vec3 rotationEulerDegrees{0.0f};
    glm::vec3 scale{1.0f};
    float alpha = 64.0f;    // Probe role only: per-object Phong shininess
    Material material;      // None role only: generic objects (e.g. imported models)
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
