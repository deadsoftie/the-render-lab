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
    float alpha = 64.0f;             // Phong shininess exponent (1..256); used as PBS roughness
    float metallic = 0.0f;           // 0 = dielectric (uses ks as F0), 1 = metal (uses kd as F0)

    // Blend factor (0..1) between the scalar fallback above and the sampled texture, per channel;
    // only meaningful when that texture is present.
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

struct LocomotionCalibration
{
    std::string clip;           // clip name, i.e. the file stem of one of skeleton.animationFiles
    float strideMeters = 1.4f;  // distance travelled per full animation cycle, at object scale
    float cruiseSpeed =
        1.4f;  // m/s while this clip is active; defaults to the binding's cruiseSpeed
};

// Optional Project 2 binding on the skeletal object; empty controlPointsXZ means no path motion.
struct PathMotionBinding
{
    std::vector<glm::vec2> controlPointsXZ;  // world-space cyclic points on the floor (y = 0), no
                                             // repeated closing point
    float cruiseSpeed = 1.4f;  // m/s, default for locomotion entries that omit their own
    float easeInEnd = 0.2f;
    float easeOutStart = 0.8f;
    float lookAheadMeters = 0.7f;
    float facingYawDegrees = 0.0f;  // asset forward correction applied after path heading
    float floorOffset = 0.0f;       // vertical offset from the floor to the model origin
    std::vector<LocomotionCalibration> locomotion;
    int activeLocomotion = 0;             // index into locomotion
    float locomotionBlendSeconds = 0.5f;  // cross-fade when switching clips, 0 cuts immediately
};

enum class IKLimitType
{
    None,
    Cone,
    Hinge
};

// One joint of the IK chain, as written in the scene's "ikReach.chain" list.
struct IKJointBinding
{
    std::string bone;
    int tier = 1;  // first priority tier that moves this joint; tier k solves joints with tier <= k
    IKLimitType limit = IKLimitType::None;
    // Character space at the bind pose (Y up, +Z forward); converted into the joint's parent
    // frame on load. Cone: center direction. Hinge: fallback axis for a straight animated arm.
    glm::vec3 axis{0.0f, 1.0f, 0.0f};
    bool followAnimation = false;  // cone centered on the animated direction instead of axis
    float minDegrees = 0.0f;       // hinge only
    float maxDegrees = 180.0f;     // cone half-angle or hinge maximum
    float maxTwistDegrees = 0.0f;  // roll allowed to line up the next joint's hinge
    float stiffness = 1.0f;        // (0, 1]; lower moves less per solver pass
};

// Optional Project 3 binding on the skeletal object; empty effectorBone means no IK reach.
struct IKReachBinding
{
    std::string effectorBone;
    std::vector<IKJointBinding> chain;  // root to the effector's parent, each a child of the last
    std::string targetObject;           // name of the scene object to reach for
    std::string idleClip;               // clip played while standing, from skeleton.animationFiles
    float standoffMeters = 0.45f;       // preferred shoulder-to-target distance on the floor
    float reachSeconds = 0.8f;
    glm::vec3 pole{0.0f, -1.0f, 0.0f};  // character-space direction the elbow bends towards
};

struct SceneObject
{
    std::string
        meshRef;  // ignored when skeleton.modelFile is set; the skinned mesh comes from there
    std::string name;
    ObjectRole role = ObjectRole::None;
    bool visible = true;
    glm::vec3 position{0.0f};
    glm::vec3 rotationEulerDegrees{0.0f};
    glm::vec3 scale{1.0f};
    Material material;
    SkeletonBinding skeleton;
    PathMotionBinding pathMotion;
    IKReachBinding ikReach;
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

constexpr int kMaxFluidShapes = 8;

enum class FluidShapeType
{
    Box,
    Sphere
};

struct FluidShape
{
    FluidShapeType type = FluidShapeType::Box;
    glm::vec3 boxMin{0.0f};
    glm::vec3 boxMax{0.0f};
    glm::vec3 center{0.0f};
    float radius = 0.0f;
    glm::vec3 velocity{0.0f};
};

struct FluidBinding
{
    bool enabled = false;
    int gridResolution = 64;
    float particleRadius = 0.01f;
    glm::vec3 domainMin{-1.0f, 0.0f, -0.5f};
    glm::vec3 domainMax{1.0f, 1.0f, 0.5f};
    std::vector<FluidShape> seeds;
    std::vector<FluidShape> obstacles;
    std::vector<FluidShape> emitters;
    int maxParticles = 1000000;
    glm::vec3 color{0.15f, 0.4f, 0.75f};
};

struct Scene
{
    std::string name;
    std::vector<SceneObject> objects;
    std::vector<SceneLight> lights;
    ScenePipeline pipeline;
    FluidBinding fluid;
};
