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

    // Structural checks only; curve validity (point separation, arc tables) is checked by Anim::PathFollower when the renderer binds it.
    bool ReadPathMotion(const nlohmann::json& jp, const SkeletonBinding& skeleton,
                        PathMotionBinding& out, std::string& error)
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
}

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
            {
                const auto& jm = jo.at("material");
                obj.material.kd = ReadVec3(jm, "kd", obj.material.kd);
                obj.material.ks = ReadVec3(jm, "ks", obj.material.ks);
                obj.material.alpha = jm.value("alpha", obj.material.alpha);
                obj.material.metallic = jm.value("metallic", obj.material.metallic);
                obj.material.albedoStrength = jm.value("albedoStrength", obj.material.albedoStrength);
                obj.material.specularStrength =
                    jm.value("specularStrength", obj.material.specularStrength);
                obj.material.roughnessStrength =
                    jm.value("roughnessStrength", obj.material.roughnessStrength);
                obj.material.metallicStrength =
                    jm.value("metallicStrength", obj.material.metallicStrength);
                obj.material.normalStrength = jm.value("normalStrength", obj.material.normalStrength);
            }

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

        outScene = std::move(scene);
        return true;
    }
    catch (const nlohmann::json::exception& e)
    {
        std::cerr << "[SceneLoader] Failed to parse " << path << ": " << e.what() << "\n";
        return false;
    }
}
