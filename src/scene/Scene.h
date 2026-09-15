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
    float alpha = 64.0f;    // Phong shininess exponent (1..256); used as PBS roughness
    float metallic = 0.0f;  // 0 = dielectric (uses ks as F0), 1 = metal (uses kd as F0)

    // Blend factor (0..1) between the scalar fallback above and the sampled texture, per
    // channel; only meaningful when the corresponding texture is present. 1 = full texture
    // strength (default, matches having no control at all).
    float albedoStrength = 1.0f;
    float specularStrength = 1.0f;
    float roughnessStrength = 1.0f;
    float metallicStrength = 1.0f;
    float normalStrength = 1.0f;
};

struct SkeletonBinding
{
    std::string modelFile;  // empty = no skeleton, obj.meshRef is a plain static mesh
    std::vector<std::string> animationFiles;
};

struct SceneObject
{
    std::string meshRef;  // ignored when skeleton.modelFile is set; the skinned mesh comes from there
    std::string name;
    ObjectRole role = ObjectRole::None;
    bool visible = true;
    glm::vec3 position{0.0f};
    glm::vec3 rotationEulerDegrees{0.0f};
    glm::vec3 scale{1.0f};
    Material material;
    SkeletonBinding skeleton;
};

glm::mat4 ComputeModelMatrix(const SceneObject& obj);

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
    bool useSHIrradiance = false;

    // Cel shading (defaults match Renderer.h's hardcoded pre-scene-JSON values).
    bool toonEnabled = true;
    int toonBands = 3;
    float outlineThickness = 1.0f;
    float depthThreshold = 0.05f;
    float normalThreshold = 0.3f;
};

struct Scene
{
    std::string name;
    std::vector<SceneObject> objects;
    std::vector<SceneLight> lights;
    ScenePipeline pipeline;
};
