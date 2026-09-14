#include "pch.h"
#include "SceneLoader.h"

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
            }

            if (hasSkeleton)
            {
                const auto& js = jo.at("skeleton");
                obj.skeleton.modelFile = js.value("modelFile", "");
                for (const auto& a : js.value("animations", nlohmann::json::array()))
                    obj.skeleton.animationFiles.push_back(a.get<std::string>());
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
