#include "pch.h"
#include "SceneLoader.h"

#include <cmath>
#include <filesystem>
#include <fstream>
#include <unordered_map>
#include <nlohmann/json.hpp>

namespace
{
    const std::unordered_map<std::string, ObjectRole> kRoleNames = {
        {"None", ObjectRole::None},
        {"Cornell", ObjectRole::Cornell},
    };

    glm::vec3 ReadVec3(const nlohmann::json& j, const char* key, glm::vec3 fallback)
    {
        if (!j.contains(key))
            return fallback;
        const auto& a = j.at(key);
        return glm::vec3(a.at(0).get<float>(), a.at(1).get<float>(), a.at(2).get<float>());
    }

    // Structural checks only; curve validity (point separation, arc tables) is checked by
    // Anim::PathFollower when the renderer binds it.
    bool ReadPathMotion(const nlohmann::json& jp,
                        const SkeletonBinding& skeleton,
                        PathMotionBinding& out,
                        std::string& error)
    {
        PathMotionBinding binding;

        for (const auto& p : jp.at("controlPoints"))
        {
            if (!p.is_array() || p.size() != 2)
            {
                error = "controlPoints entries must be [x, z] pairs";
                return false;
            }
            binding.controlPointsXZ.emplace_back(p.at(0).get<float>(), p.at(1).get<float>());
        }
        if (binding.controlPointsXZ.size() < 8)
        {
            error = "controlPoints needs at least 8 points";
            return false;
        }
        for (const glm::vec2& p : binding.controlPointsXZ)
        {
            if (!std::isfinite(p.x) || !std::isfinite(p.y))
            {
                error = "controlPoints must be finite";
                return false;
            }
        }

        binding.cruiseSpeed = jp.value("cruiseSpeed", binding.cruiseSpeed);
        binding.easeInEnd = jp.value("easeInEnd", binding.easeInEnd);
        binding.easeOutStart = jp.value("easeOutStart", binding.easeOutStart);
        binding.lookAheadMeters = jp.value("lookAheadMeters", binding.lookAheadMeters);
        binding.facingYawDegrees = jp.value("facingYawDegrees", binding.facingYawDegrees);
        binding.floorOffset = jp.value("floorOffset", binding.floorOffset);
        binding.activeLocomotion = jp.value("activeLocomotion", binding.activeLocomotion);
        binding.locomotionBlendSeconds =
            jp.value("locomotionBlendSeconds", binding.locomotionBlendSeconds);

        if (!std::isfinite(binding.cruiseSpeed) || binding.cruiseSpeed < 0.0f)
        {
            error = "cruiseSpeed must be finite and nonnegative";
            return false;
        }
        if (!(binding.easeInEnd > 0.0f && binding.easeInEnd < binding.easeOutStart &&
              binding.easeOutStart < 1.0f))
        {
            error = "ease times must satisfy 0 < easeInEnd < easeOutStart < 1";
            return false;
        }
        if (!std::isfinite(binding.lookAheadMeters) || binding.lookAheadMeters <= 0.0f)
        {
            error = "lookAheadMeters must be finite and positive";
            return false;
        }
        if (!std::isfinite(binding.facingYawDegrees) || !std::isfinite(binding.floorOffset))
        {
            error = "facingYawDegrees and floorOffset must be finite";
            return false;
        }
        if (!std::isfinite(binding.locomotionBlendSeconds) || binding.locomotionBlendSeconds < 0.0f)
        {
            error = "locomotionBlendSeconds must be finite and nonnegative";
            return false;
        }

        for (const auto& jl : jp.at("locomotion"))
        {
            LocomotionCalibration calib;
            calib.clip = jl.at("clip").get<std::string>();
            calib.strideMeters = jl.value("strideMeters", calib.strideMeters);
            calib.cruiseSpeed = jl.value("cruiseSpeed", binding.cruiseSpeed);

            bool clipListed = false;
            for (const std::string& animPath : skeleton.animationFiles)
                clipListed |= std::filesystem::path(animPath).stem().string() == calib.clip;
            if (!clipListed)
            {
                error = "locomotion clip '" + calib.clip + "' is not in skeleton.animations";
                return false;
            }
            if (!std::isfinite(calib.strideMeters) || calib.strideMeters <= 0.0f)
            {
                error = "strideMeters for '" + calib.clip + "' must be finite and positive";
                return false;
            }
            if (!std::isfinite(calib.cruiseSpeed) || calib.cruiseSpeed < 0.0f)
            {
                error = "cruiseSpeed for '" + calib.clip + "' must be finite and nonnegative";
                return false;
            }
            binding.locomotion.push_back(std::move(calib));
        }
        if (binding.activeLocomotion < 0 ||
            binding.activeLocomotion >= static_cast<int>(binding.locomotion.size()))
        {
            error = "activeLocomotion must index a locomotion entry";
            return false;
        }

        out = std::move(binding);
        return true;
    }

    void ReadMaterial(const nlohmann::json& jm, Material& material)
    {
        material.kd = ReadVec3(jm, "kd", material.kd);
        material.ks = ReadVec3(jm, "ks", material.ks);
        material.alpha = jm.value("alpha", material.alpha);
        material.metallic = jm.value("metallic", material.metallic);
        material.albedoStrength = jm.value("albedoStrength", material.albedoStrength);
        material.specularStrength = jm.value("specularStrength", material.specularStrength);
        material.roughnessStrength = jm.value("roughnessStrength", material.roughnessStrength);
        material.metallicStrength = jm.value("metallicStrength", material.metallicStrength);
        material.normalStrength = jm.value("normalStrength", material.normalStrength);
    }

    bool IsFiniteVec3(const glm::vec3& v)
    {
        return std::isfinite(v.x) && std::isfinite(v.y) && std::isfinite(v.z);
    }

    bool ReadSpringEnd(const nlohmann::json& je, PhysicsSpringEndBinding& end, std::string& error)
    {
        end.anchor = je.value("anchor", "");
        end.body = je.value("body", "");
        end.point = ReadVec3(je, "point", end.point);
        if (end.anchor.empty() == end.body.empty())
        {
            error = "each spring end needs exactly one of 'anchor' or 'body'";
            return false;
        }
        return true;
    }

    // "physics" block.
    bool ReadPhysics(const nlohmann::json& jp, PhysicsBinding& out, std::string& error)
    {
        PhysicsBinding binding;
        binding.enabled = true;
        binding.integrator = jp.value("integrator", binding.integrator);
        binding.dt = jp.value("dt", binding.dt);
        binding.maxStepsPerFrame = jp.value("maxStepsPerFrame", binding.maxStepsPerFrame);
        binding.gravity = ReadVec3(jp, "gravity", binding.gravity);
        binding.linearDrag = jp.value("linearDrag", binding.linearDrag);
        binding.angularDrag = jp.value("angularDrag", binding.angularDrag);
        binding.anchorRadius = jp.value("anchorRadius", binding.anchorRadius);
        binding.bodyMaterial.kd = glm::vec3(0.55f, 0.36f, 0.2f);
        binding.bodyMaterial.ks = glm::vec3(0.04f);
        binding.bodyMaterial.alpha = 40.0f;
        binding.anchorMaterial.kd = glm::vec3(0.9f, 0.15f, 0.1f);
        binding.anchorMaterial.ks = glm::vec3(0.04f);
        binding.anchorMaterial.alpha = 120.0f;
        if (jp.contains("bodyMaterial"))
            ReadMaterial(jp.at("bodyMaterial"), binding.bodyMaterial);
        if (jp.contains("anchorMaterial"))
            ReadMaterial(jp.at("anchorMaterial"), binding.anchorMaterial);

        static const char* kIntegrators[] = {"Euler", "Midpoint", "Verlet", "RK2", "RK4"};
        bool knownIntegrator = false;
        for (const char* name : kIntegrators)
            knownIntegrator |= binding.integrator == name;
        if (!knownIntegrator)
        {
            error = "integrator must be one of Euler, Midpoint, Verlet, RK2, RK4";
            return false;
        }
        if (!std::isfinite(binding.dt) || binding.dt <= 0.0f || binding.dt > 0.05f)
        {
            error = "dt must be in (0, 0.05]";
            return false;
        }
        if (binding.maxStepsPerFrame < 1 || !IsFiniteVec3(binding.gravity) ||
            !std::isfinite(binding.linearDrag) || binding.linearDrag < 0.0f ||
            !std::isfinite(binding.angularDrag) || binding.angularDrag < 0.0f)
        {
            error = "maxStepsPerFrame must be positive, gravity finite, drags nonnegative";
            return false;
        }

        if (jp.contains("chain"))
        {
            const auto& jc = jp.at("chain");
            PhysicsChainBinding& c = binding.chain;
            c.enabled = true;
            c.count = jc.value("count", c.count);
            c.anchorA = ReadVec3(jc, "anchorA", c.anchorA);
            c.anchorB = ReadVec3(jc, "anchorB", c.anchorB);
            c.plankHalfExtents = ReadVec3(jc, "plankHalfExtents", c.plankHalfExtents);
            c.plankMass = jc.value("plankMass", c.plankMass);
            c.restLength = jc.value("restLength", c.restLength);
            c.stiffness = jc.value("stiffness", c.stiffness);
            c.damping = jc.value("damping", c.damping);
            c.diagonals = jc.value("diagonals", c.diagonals);
        }

        for (const auto& ja : jp.value("anchors", nlohmann::json::array()))
        {
            PhysicsAnchorBinding a;
            a.name = ja.at("name").get<std::string>();
            a.position = ReadVec3(ja, "position", a.position);
            binding.anchors.push_back(a);
        }

        for (const auto& jb : jp.value("bodies", nlohmann::json::array()))
        {
            PhysicsBodyBinding b;
            b.name = jb.at("name").get<std::string>();
            b.halfExtents = ReadVec3(jb, "halfExtents", b.halfExtents);
            b.mass = jb.value("mass", b.mass);
            b.position = ReadVec3(jb, "position", b.position);
            b.rotationEulerDegrees = ReadVec3(jb, "rotationDegrees", b.rotationEulerDegrees);
            b.velocity = ReadVec3(jb, "velocity", b.velocity);
            b.angularVelocity = ReadVec3(jb, "angularVelocity", b.angularVelocity);
            if (!(b.mass > 0.0f) ||
                !(b.halfExtents.x > 0.0f && b.halfExtents.y > 0.0f && b.halfExtents.z > 0.0f))
            {
                error = "body '" + b.name + "' needs a positive mass and half extents";
                return false;
            }
            binding.bodies.push_back(b);
        }

        for (const auto& js : jp.value("springs", nlohmann::json::array()))
        {
            PhysicsSpringBinding sp;
            if (!ReadSpringEnd(js.at("a"), sp.a, error) || !ReadSpringEnd(js.at("b"), sp.b, error))
                return false;
            sp.stiffness = js.value("stiffness", sp.stiffness);
            sp.damping = js.value("damping", sp.damping);
            sp.restLength = js.value("restLength", sp.restLength);
            if (!(sp.stiffness > 0.0f) || !(sp.damping >= 0.0f))
            {
                error = "spring stiffness must be positive and damping nonnegative";
                return false;
            }
            binding.springs.push_back(sp);
        }

        if (!binding.chain.enabled && binding.bodies.empty())
        {
            error = "physics needs a chain or at least one body";
            return false;
        }

        out = std::move(binding);
        return true;
    }

    bool ReadPbfShape(const nlohmann::json& js, PbfShapeBinding& shape, std::string& error)
    {
        std::string type = js.value("type", std::string("box"));
        if (type == "box")
        {
            shape.type = PbfShapeType::Box;
            shape.boxMin = ReadVec3(js, "min", shape.boxMin);
            shape.boxMax = ReadVec3(js, "max", shape.boxMax);
            if (!IsFiniteVec3(shape.boxMin) || !IsFiniteVec3(shape.boxMax) ||
                shape.boxMin.x >= shape.boxMax.x || shape.boxMin.y >= shape.boxMax.y ||
                shape.boxMin.z >= shape.boxMax.z)
            {
                error = "box shapes need finite min < max";
                return false;
            }
        }
        else if (type == "sphere")
        {
            shape.type = PbfShapeType::Sphere;
            shape.center = ReadVec3(js, "center", shape.center);
            shape.radius = js.value("radius", shape.radius);
            if (!IsFiniteVec3(shape.center) || !(shape.radius > 0.0f))
            {
                error = "sphere shapes need a finite center and positive radius";
                return false;
            }
        }
        else
        {
            error = "unknown shape type '" + type + "'";
            return false;
        }
        shape.velocity = ReadVec3(js, "velocity", shape.velocity);
        shape.rate = js.value("rate", shape.rate);
        return true;
    }

    // "pbf" block.
    bool ReadPbf(const nlohmann::json& jf, PbfBinding& out, std::string& error)
    {
        PbfBinding binding;
        binding.enabled = true;
        binding.particleRadius = jf.value("particleRadius", binding.particleRadius);
        binding.domainMin = ReadVec3(jf, "domainMin", binding.domainMin);
        binding.domainMax = ReadVec3(jf, "domainMax", binding.domainMax);
        binding.maxParticles = jf.value("maxParticles", binding.maxParticles);
        binding.color = ReadVec3(jf, "color", binding.color);
        binding.absorption = jf.value("absorption", binding.absorption);
        binding.iterations = jf.value("iterations", binding.iterations);
        binding.substeps = jf.value("substeps", binding.substeps);
        binding.epsilon = jf.value("epsilon", binding.epsilon);
        binding.vorticity = jf.value("vorticity", binding.vorticity);
        binding.xsph = jf.value("xsph", binding.xsph);
        binding.coupling = jf.value("coupling", binding.coupling);
        binding.couplingStrength = jf.value("couplingStrength", binding.couplingStrength);
        binding.friction = jf.value("friction", binding.friction);

        if (!(binding.particleRadius > 0.0f) || !IsFiniteVec3(binding.domainMin) ||
            !IsFiniteVec3(binding.domainMax) || binding.domainMin.x >= binding.domainMax.x ||
            binding.domainMin.y >= binding.domainMax.y || binding.domainMin.z >= binding.domainMax.z)
        {
            error = "particleRadius must be positive and domainMin < domainMax";
            return false;
        }
        if (binding.maxParticles < 1 || binding.iterations < 1 || binding.substeps < 1)
        {
            error = "maxParticles, iterations and substeps must be positive";
            return false;
        }
        if (binding.coupling != "level" && binding.coupling != "impulse" && binding.coupling != "buoyancy" &&
            binding.coupling != "off")
        {
            error = "coupling must be level, impulse, buoyancy or off";
            return false;
        }

        const std::pair<const char*, std::vector<PbfShapeBinding>*> lists[] = {
            {"seeds", &binding.seeds}, {"obstacles", &binding.obstacles}, {"emitters", &binding.emitters}};
        for (const auto& [key, list] : lists)
        {
            for (const auto& js : jf.value(key, nlohmann::json::array()))
            {
                PbfShapeBinding shape;
                if (!ReadPbfShape(js, shape, error))
                {
                    error = std::string(key) + ": " + error;
                    return false;
                }
                list->push_back(shape);
            }
        }
        if (binding.seeds.empty() && binding.emitters.empty())
        {
            error = "pbf needs at least one seed or emitter";
            return false;
        }

        out = std::move(binding);
        return true;
    }

    bool IsUsableDirection(const glm::vec3& v)
    {
        return std::isfinite(v.x) && std::isfinite(v.y) && std::isfinite(v.z) &&
               glm::dot(v, v) > 1e-8f;
    }

    // Range checks only; whether the bone exists and follows its predecessor is checked against
    // the loaded skeleton by the renderer.
    bool ReadIKJoint(const nlohmann::json& jj, IKJointBinding& joint, std::string& error)
    {
        const std::unordered_map<std::string, IKLimitType> limitNames = {
            {"none", IKLimitType::None},
            {"cone", IKLimitType::Cone},
            {"hinge", IKLimitType::Hinge},
        };

        joint.bone = jj.at("bone").get<std::string>();
        joint.tier = jj.value("tier", joint.tier);
        std::string limit = jj.value("limit", std::string("none"));
        auto it = limitNames.find(limit);
        if (it == limitNames.end())
        {
            error = "unknown limit '" + limit + "' on '" + joint.bone + "'";
            return false;
        }
        joint.limit = it->second;
        joint.axis = ReadVec3(jj, "axis", joint.axis);
        joint.followAnimation = jj.value("followAnimation", joint.followAnimation);
        joint.minDegrees = jj.value("minDegrees", joint.minDegrees);
        joint.maxDegrees = jj.value("maxDegrees", joint.maxDegrees);
        joint.maxTwistDegrees = jj.value("maxTwistDegrees", joint.maxTwistDegrees);
        joint.stiffness = jj.value("stiffness", joint.stiffness);

        if (joint.tier < 1)
        {
            error = "tier on '" + joint.bone + "' must be at least 1";
            return false;
        }
        if (!IsUsableDirection(joint.axis))
        {
            error = "axis on '" + joint.bone + "' must be finite and nonzero";
            return false;
        }
        // A hinge may hyperextend (negative minimum); a cone only has a half-angle.
        if (!std::isfinite(joint.minDegrees) || !std::isfinite(joint.maxDegrees) ||
            joint.minDegrees < (joint.limit == IKLimitType::Hinge ? -180.0f : 0.0f) ||
            joint.minDegrees > joint.maxDegrees || joint.maxDegrees > 180.0f)
        {
            error = "limits on '" + joint.bone + "' must satisfy min <= max within 180 degrees";
            return false;
        }
        if (!std::isfinite(joint.maxTwistDegrees) || joint.maxTwistDegrees < 0.0f ||
            joint.maxTwistDegrees > 180.0f)
        {
            error = "maxTwistDegrees on '" + joint.bone + "' must be in [0, 180]";
            return false;
        }
        if (!std::isfinite(joint.stiffness) || joint.stiffness <= 0.0f || joint.stiffness > 1.0f)
        {
            error = "stiffness on '" + joint.bone + "' must be in (0, 1]";
            return false;
        }
        return true;
    }

    bool ReadIKReach(const nlohmann::json& jr,
                     const SkeletonBinding& skeleton,
                     IKReachBinding& out,
                     std::string& error)
    {
        IKReachBinding binding;
        binding.effectorBone = jr.at("effector").get<std::string>();
        binding.targetObject = jr.at("targetObject").get<std::string>();
        binding.idleClip = jr.at("idleClip").get<std::string>();
        binding.standoffMeters = jr.value("standoffMeters", binding.standoffMeters);
        binding.reachSeconds = jr.value("reachSeconds", binding.reachSeconds);
        binding.pole = ReadVec3(jr, "pole", binding.pole);

        for (const auto& jj : jr.at("chain"))
        {
            IKJointBinding joint;
            if (!ReadIKJoint(jj, joint, error))
                return false;
            binding.chain.push_back(std::move(joint));
        }
        if (binding.chain.empty())
        {
            error = "chain needs at least one joint above the effector";
            return false;
        }
        // Tiers must shrink towards the effector so every tier is a tail of the chain that ends
        // at the effector; the joint next to it always belongs to tier 1.
        for (std::size_t i = 1; i < binding.chain.size(); ++i)
        {
            if (binding.chain[i].tier > binding.chain[i - 1].tier)
            {
                error = "chain tiers must not increase from root to effector";
                return false;
            }
        }
        if (binding.chain.back().tier != 1)
        {
            error = "the joint next to the effector must be in tier 1";
            return false;
        }

        if (!std::isfinite(binding.standoffMeters) || binding.standoffMeters < 0.0f)
        {
            error = "standoffMeters must be finite and nonnegative";
            return false;
        }
        if (!std::isfinite(binding.reachSeconds) || binding.reachSeconds <= 0.0f)
        {
            error = "reachSeconds must be finite and positive";
            return false;
        }
        if (!IsUsableDirection(binding.pole))
        {
            error = "pole must be finite and nonzero";
            return false;
        }

        bool idleListed = false;
        for (const std::string& animPath : skeleton.animationFiles)
            idleListed |= std::filesystem::path(animPath).stem().string() == binding.idleClip;
        if (!idleListed)
        {
            error = "idle clip '" + binding.idleClip + "' is not in skeleton.animations";
            return false;
        }

        out = std::move(binding);
        return true;
    }
}  // namespace

bool SceneLoader::Load(const std::string& path, Scene& outScene)
{
    std::ifstream file(path);
    if (!file.is_open())
    {
        std::cerr << "[SceneLoader] Failed to open " << path << "\n";
        return false;
    }

    try
    {
        nlohmann::json j;
        file >> j;

        Scene scene;
        scene.name = j.value("name", path);

        for (const auto& jo : j.value("objects", nlohmann::json::array()))
        {
            bool hasSkeleton =
                jo.contains("skeleton") && !jo.at("skeleton").value("modelFile", "").empty();

            if (!jo.contains("meshRef") && !hasSkeleton)
            {
                std::cerr << "[SceneLoader] Object missing meshRef in " << path << "\n";
                return false;
            }

            SceneObject obj;
            obj.meshRef = jo.value("meshRef", "");
            obj.name = jo.value("name", obj.meshRef);
            obj.visible = jo.value("visible", true);
            obj.position = ReadVec3(jo, "position", glm::vec3(0.0f));
            obj.rotationEulerDegrees = ReadVec3(jo, "rotationDegrees", glm::vec3(0.0f));
            obj.scale = ReadVec3(jo, "scale", glm::vec3(1.0f));

            if (jo.contains("material"))
                ReadMaterial(jo.at("material"), obj.material);

            if (hasSkeleton)
            {
                const auto& js = jo.at("skeleton");
                obj.skeleton.modelFile = js.value("modelFile", "");
                for (const auto& a : js.value("animations", nlohmann::json::array()))
                    obj.skeleton.animationFiles.push_back(a.get<std::string>());
            }

            if (jo.contains("pathMotion"))
            {
                std::string error;
                if (!hasSkeleton)
                    error = "pathMotion requires a skeleton binding";
                else
                    ReadPathMotion(jo.at("pathMotion"), obj.skeleton, obj.pathMotion, error);
                if (!error.empty())
                {
                    std::cerr << "[SceneLoader] Invalid pathMotion on '" << obj.name << "' in "
                              << path << ": " << error << "\n";
                    return false;
                }
            }

            if (jo.contains("ikReach"))
            {
                std::string error;
                if (!hasSkeleton)
                    error = "ikReach requires a skeleton binding";
                else
                    ReadIKReach(jo.at("ikReach"), obj.skeleton, obj.ikReach, error);
                if (!error.empty())
                {
                    std::cerr << "[SceneLoader] Invalid ikReach on '" << obj.name << "' in " << path
                              << ": " << error << "\n";
                    return false;
                }
            }

            std::string roleStr = jo.value("role", "None");
            auto it = kRoleNames.find(roleStr);
            if (it == kRoleNames.end())
            {
                std::cerr << "[SceneLoader] Unknown role '" << roleStr << "' in " << path << "\n";
                return false;
            }
            obj.role = it->second;

            scene.objects.push_back(obj);
        }

        // Checked after every object is read, since the target may be listed after the character.
        for (const SceneObject& obj : scene.objects)
        {
            if (obj.ikReach.effectorBone.empty())
                continue;
            bool targetFound = false;
            for (const SceneObject& other : scene.objects)
                targetFound |= &other != &obj && other.name == obj.ikReach.targetObject &&
                               other.skeleton.modelFile.empty();
            if (!targetFound)
            {
                std::cerr << "[SceneLoader] Invalid ikReach on '" << obj.name << "' in " << path
                          << ": targetObject '" << obj.ikReach.targetObject
                          << "' must name another non-skeletal object\n";
                return false;
            }
        }

        for (const auto& jl : j.value("lights", nlohmann::json::array()))
        {
            SceneLight light;
            light.position = ReadVec3(jl, "position", glm::vec3(0.0f));
            light.color = ReadVec3(jl, "color", glm::vec3(1.0f));
            light.range = jl.value("range", 1.5f);
            light.intensity = jl.value("intensity", 1.0f);
            light.enabled = jl.value("enabled", true);
            scene.lights.push_back(light);
        }

        if (j.contains("pipeline"))
        {
            const auto& jp = j.at("pipeline");
            scene.pipeline.hdriFile = jp.value("hdriFile", "");
            scene.pipeline.shadowsEnabled = jp.value("shadowsEnabled", true);
            scene.pipeline.useMSM = jp.value("useMSM", true);
            scene.pipeline.aoEnabled = jp.value("aoEnabled", true);
            scene.pipeline.celEnabled = jp.value("celEnabled", false);
            scene.pipeline.useLightVolumes = jp.value("useLightVolumes", false);
            scene.pipeline.exposure = jp.value("exposure", 2.5f);
            scene.pipeline.hdriRotation = jp.value("hdriRotation", 0.0f);
            scene.pipeline.ambient = jp.value("ambient", 0.02f);
            scene.pipeline.useSHIrradiance = jp.value("useSHIrradiance", false);
            scene.pipeline.toonEnabled = jp.value("toonEnabled", true);
            scene.pipeline.toonBands = jp.value("toonBands", 3);
            scene.pipeline.outlineThickness = jp.value("outlineThickness", 1.0f);
            scene.pipeline.depthThreshold = jp.value("depthThreshold", 0.05f);
            scene.pipeline.normalThreshold = jp.value("normalThreshold", 0.3f);
        }

        if (j.contains("physics"))
        {
            std::string error;
            if (!ReadPhysics(j.at("physics"), scene.physics, error))
            {
                std::cerr << "[SceneLoader] Invalid physics in " << path << ": " << error << "\n";
                return false;
            }
        }

        if (j.contains("pbf"))
        {
            std::string error;
            if (!ReadPbf(j.at("pbf"), scene.pbf, error))
            {
                std::cerr << "[SceneLoader] Invalid pbf in " << path << ": " << error << "\n";
                return false;
            }
        }

        outScene = std::move(scene);
        return true;
    }
    catch (const nlohmann::json::exception& e)
    {
        std::cerr << "[SceneLoader] Failed to parse " << path << ": " << e.what() << "\n";
        return false;
    }
}
