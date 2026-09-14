#pragma once
#include <filesystem>
#include <memory>
#include <string>

#include <ufbx.h>

// Shared by ModelLoader and SkinnedModelLoader: scene lifetime, load options, and material/texture resolution are identical between the two loaders.
namespace UfbxSceneUtil
{
    struct SceneDeleter
    {
        void operator()(ufbx_scene* scene) const;
    };
    using ScenePtr = std::unique_ptr<ufbx_scene, SceneDeleter>;

    ufbx_load_opts MakeLoadOpts();
    std::string ToStdString(ufbx_string s);
    glm::vec3 ToGlmVec3(ufbx_vec3 v);
    glm::vec3 GetDiffuseColor(const ufbx_material* mat);

    // logTag prefixes any warning this logs, e.g. "[ModelLoader]".
    std::string ResolveDiffuseTexture(const ufbx_material* mat,
                                      const std::filesystem::path& modelDir, const char* logTag);
}
