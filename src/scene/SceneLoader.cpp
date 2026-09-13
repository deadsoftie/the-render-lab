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
        {"Ground", ObjectRole::Ground},
        {"TallCube", ObjectRole::TallCube},
        {"ShortCube", ObjectRole::ShortCube},
        {"SmallCube", ObjectRole::SmallCube},
        {"Sphere", ObjectRole::Sphere},
        {"Probe", ObjectRole::Probe},
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

    nlohmann::json j;
    try
    {
        file >> j;
    }
    catch (const nlohmann::json::exception& e)
    {
        std::cerr << "[SceneLoader] Failed to parse " << path << ": " << e.what() << "\n";
        return false;
    }

    Scene scene;
    scene.name = j.value("name", path);

    for (const auto& jo : j.value("objects", nlohmann::json::array()))
    {
        if (!jo.contains("meshRef"))
        {
            std::cerr << "[SceneLoader] Object missing meshRef in " << path << "\n";
            return false;
        }

        SceneObject obj;
        obj.meshRef = jo.at("meshRef").get<std::string>();
        obj.position = ReadVec3(jo, "position", glm::vec3(0.0f));
        obj.scale = ReadVec3(jo, "scale", glm::vec3(1.0f));
        obj.alpha = jo.value("alpha", 64.0f);

        std::string roleStr = jo.value("role", "None");
        auto it = kRoleNames.find(roleStr);
        if (it == kRoleNames.end())
        {
            std::cerr << "[SceneLoader] Unknown role '" << roleStr << "' in " << path << "\n";
            obj.role = ObjectRole::None;
        }
        else
        {
            obj.role = it->second;
        }

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
    }

    outScene = std::move(scene);
    return true;
}
