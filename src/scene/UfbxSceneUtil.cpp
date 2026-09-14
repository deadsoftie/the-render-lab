#include "pch.h"
#include "scene/UfbxSceneUtil.h"

void UfbxSceneUtil::SceneDeleter::operator()(ufbx_scene* scene) const
{
    ufbx_free_scene(scene);
}

ufbx_load_opts UfbxSceneUtil::MakeLoadOpts()
{
    ufbx_load_opts opts = {};
    opts.target_axes = ufbx_axes_right_handed_y_up;
    opts.target_unit_meters = 1.0f;
    opts.generate_missing_normals = true;
    return opts;
}

std::string UfbxSceneUtil::ToStdString(ufbx_string s)
{
    return std::string(s.data, s.length);
}

glm::vec3 UfbxSceneUtil::ToGlmVec3(ufbx_vec3 v)
{
    return {static_cast<float>(v.x), static_cast<float>(v.y), static_cast<float>(v.z)};
}

glm::vec3 UfbxSceneUtil::GetDiffuseColor(const ufbx_material* mat)
{
    if (mat && mat->fbx.diffuse_color.has_value)
        return ToGlmVec3(mat->fbx.diffuse_color.value_vec3);
    return {1.0f, 1.0f, 1.0f};
}

std::string UfbxSceneUtil::ResolveDiffuseTexture(const ufbx_material* mat,
                                                 const std::filesystem::path& modelDir,
                                                 const char* logTag)
{
    if (!mat || !mat->fbx.diffuse_color.texture)
        return "";

    const ufbx_texture* tex = mat->fbx.diffuse_color.texture;
    if (tex->type != UFBX_TEXTURE_FILE)
    {
        std::cerr << logTag << " Non-file texture type not supported\n";
        return "";
    }

    std::string relative = ToStdString(tex->relative_filename);
    if (relative.empty())
        relative = ToStdString(tex->filename);
    if (relative.empty())
        return "";

    return (modelDir / relative).lexically_normal().string();
}
