#pragma once
#include <glm/glm.hpp>
#include <string>
#include <vector>

enum class ObjectRole
{
    None,
    Cornell
};

struct Light
{
    glm::vec3 position{1.2f, 1.0f, 2.0f};
    glm::vec3 color{1.0f, 1.0f, 1.0f};
    float range = 2.0f;
};

struct Material
{
    glm::vec3 kd{0.8f, 0.3f, 0.2f};  // diffuse
    glm::vec3 ks{0.04f};             // specular / F0
    float alpha = 64.0f;  // Phong shininess exponent (1..256); used as PBS roughness
};

struct SceneObject
{
    std::string meshRef;
    std::string name;
    ObjectRole role = ObjectRole::None;
    bool visible = true;
    glm::vec3 position{0.0f};
    glm::vec3 rotationEulerDegrees{0.0f};
    glm::vec3 scale{1.0f};
    Material material;
};

struct SceneLight : Light
{
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
    float ambient = 0.02f;
};

struct Scene
{
    std::string name;
    std::vector<SceneObject> objects;
    std::vector<SceneLight> lights;
    ScenePipeline pipeline;
};
