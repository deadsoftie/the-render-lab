#include "pch.h"
#include "anim/SkeletalLoader.h"
#include "scene/UfbxSceneUtil.h"

#include <algorithm>
#include <cmath>
#include <unordered_map>

using UfbxSceneUtil::ScenePtr;
using UfbxSceneUtil::ToStdString;

namespace
{
    // Unlike UfbxSceneUtil::MakeLoadOpts, this skips generate_missing_normals since skeleton-only loading never touches mesh geometry.
    ufbx_load_opts MakeLoadOpts()
    {
        ufbx_load_opts opts = {};
        opts.target_axes = ufbx_axes_right_handed_y_up;
        opts.target_unit_meters = 1.0f;
        return opts;
    }

    // ufbx_matrix is 4x3 (cols[0..2]=basis, cols[3]=translation, row 3 implicit [0,0,0,1]); Anim::Mat4 is column-major m[col*4+row], matching GL/glm.
    Anim::Mat4 ConvertMatrix(const ufbx_matrix& m)
    {
        Anim::Mat4 result;
        result.m[0] = static_cast<float>(m.m00);
        result.m[1] = static_cast<float>(m.m10);
        result.m[2] = static_cast<float>(m.m20);
        result.m[3] = 0.0f;
        result.m[4] = static_cast<float>(m.m01);
        result.m[5] = static_cast<float>(m.m11);
        result.m[6] = static_cast<float>(m.m21);
        result.m[7] = 0.0f;
        result.m[8] = static_cast<float>(m.m02);
        result.m[9] = static_cast<float>(m.m12);
        result.m[10] = static_cast<float>(m.m22);
        result.m[11] = 0.0f;
        result.m[12] = static_cast<float>(m.m03);
        result.m[13] = static_cast<float>(m.m13);
        result.m[14] = static_cast<float>(m.m23);
        result.m[15] = 1.0f;
        return result;
    }

    struct WalkState
    {
        Anim::Mat4 nearestBoneWorldMat = Anim::Identity();
        int nearestBoneIndex = -1;
    };

    void CollectBones(const ufbx_node* node, const WalkState& state,
                      const std::unordered_map<std::string, ufbx_matrix>& offsetMatrices,
                      Anim::Skeleton& outSkeleton)
    {
        std::string name = ToStdString(node->name);
        Anim::Mat4 nodeWorldMat = ConvertMatrix(node->node_to_world);

        WalkState childState = state;

        auto offsetIt = offsetMatrices.find(name);
        if (offsetIt != offsetMatrices.end())
        {
            Anim::Bone bone;
            bone.name = name;
            bone.parentIndex = state.nearestBoneIndex;
            bone.inverseBindPose = ConvertMatrix(offsetIt->second);

            Anim::Mat4 relativeToParent = Anim::Inverse(state.nearestBoneWorldMat) * nodeWorldMat;
            Anim::Decompose(relativeToParent, bone.localBindPose.v, bone.localBindPose.q,
                            bone.localBindPose.s);

            int index = static_cast<int>(outSkeleton.bones.size());
            outSkeleton.boneNameToIndex[name] = index;
            outSkeleton.bones.push_back(bone);

            childState.nearestBoneWorldMat = nodeWorldMat;
            childState.nearestBoneIndex = index;
        }

        for (const ufbx_node* child : node->children)
            CollectBones(child, childState, offsetMatrices, outSkeleton);
    }
}

bool SkeletalLoader::LoadSkeleton(const std::string& path, Anim::Skeleton& outSkeleton)
{
    ufbx_load_opts opts = MakeLoadOpts();
    ufbx_error error;
    ScenePtr scene(ufbx_load_file(path.c_str(), &opts, &error));
    if (!scene)
    {
        std::cerr << "[SkeletalLoader] Failed to load " << path << ": "
                  << ToStdString(error.description) << "\n";
        return false;
    }

    std::unordered_map<std::string, ufbx_matrix> offsetMatrices;
    for (const ufbx_skin_deformer* deformer : scene->skin_deformers)
    {
        for (const ufbx_skin_cluster* cluster : deformer->clusters)
        {
            if (!cluster->bone_node)
                continue;
            offsetMatrices.emplace(ToStdString(cluster->bone_node->name), cluster->geometry_to_bone);
        }
    }

    if (offsetMatrices.empty())
    {
        std::cerr << "[SkeletalLoader] No bones found in " << path << "\n";
        return false;
    }

    Anim::Skeleton skeleton;
    CollectBones(scene->root_node, WalkState{}, offsetMatrices, skeleton);

    if (skeleton.bones.empty())
    {
        std::cerr << "[SkeletalLoader] Bone nodes not reachable from root in " << path << "\n";
        return false;
    }

    outSkeleton = std::move(skeleton);
    return true;
}

bool SkeletalLoader::LoadAnimationClip(const std::string& path, const Anim::Skeleton& skeleton,
                                       Anim::AnimationClip& outClip)
{
    ufbx_load_opts opts = MakeLoadOpts();
    ufbx_error error;
    ScenePtr scene(ufbx_load_file(path.c_str(), &opts, &error));
    if (!scene || scene->anim_stacks.count == 0)
    {
        std::cerr << "[SkeletalLoader] No animation found in " << path << "\n";
        return false;
    }

    const ufbx_anim_stack* stack = scene->anim_stacks.data[0];

    Anim::AnimationClip clip;
    clip.name = ToStdString(stack->name);
    if (clip.name.empty())
        clip.name = path;
    clip.duration = static_cast<float>(stack->time_end - stack->time_begin);
    clip.ticksPerSecond = 1.0f;  // ufbx keyframe/evaluation times are already in seconds
    clip.channels.resize(skeleton.bones.size());

    // Bones with no curve here keep an empty channel, so Animator falls back to their bind pose.
    std::vector<bool> animated(skeleton.bones.size(), false);
    std::vector<double> times;

    for (size_t i = 0; i < skeleton.bones.size(); ++i)
    {
        const ufbx_node* node = ufbx_find_node(scene.get(), skeleton.bones[i].name.c_str());
        if (!node)
        {
            std::cerr << "[SkeletalLoader] Bone '" << skeleton.bones[i].name
                      << "' has no matching node in " << path << ", falling back to bind pose\n";
            continue;
        }

        for (const ufbx_anim_layer* layer : stack->layers)
        {
            const char* propNames[3] = {UFBX_Lcl_Translation, UFBX_Lcl_Rotation, UFBX_Lcl_Scaling};
            for (const char* propName : propNames)
            {
                const ufbx_anim_prop* prop = ufbx_find_anim_prop(layer, &node->element, propName);
                if (!prop || !prop->anim_value)
                    continue;
                for (int c = 0; c < 3; ++c)
                {
                    const ufbx_anim_curve* curve = prop->anim_value->curves[c];
                    if (!curve)
                        continue;
                    animated[i] = true;
                    for (const ufbx_keyframe& kf : curve->keyframes)
                        times.push_back(kf.time);
                }
            }
        }
    }

    if (times.empty())
    {
        outClip = std::move(clip);
        return true;
    }

    std::sort(times.begin(), times.end());
    times.erase(std::unique(times.begin(), times.end(),
                            [](double a, double b) { return std::abs(a - b) < 1e-9; }),
               times.end());

    for (size_t i = 0; i < skeleton.bones.size(); ++i)
    {
        if (!animated[i])
            continue;
        Anim::BoneChannel& channel = clip.channels[i];
        channel.positions.reserve(times.size());
        channel.rotations.reserve(times.size());
        channel.scales.reserve(times.size());
    }

    // Evaluate the whole scene per unique time rather than sampling each bone's local transform: default TRANSFORM_ROOT space conversion bakes the unit/axis scale into the true FBX root only, which a per-bone local sample below our skipped non-bone ancestors would miss - it has to come from a full node_to_world, same as the bind pose already gets it.
    for (double t : times)
    {
        ufbx_error evalError;
        ScenePtr evalScene(ufbx_evaluate_scene(scene.get(), stack->anim, t, nullptr, &evalError));
        if (!evalScene)
            continue;

        float localTime = static_cast<float>(t - stack->time_begin);

        for (size_t i = 0; i < skeleton.bones.size(); ++i)
        {
            if (!animated[i])
                continue;

            const ufbx_node* boneNode =
                ufbx_find_node(evalScene.get(), skeleton.bones[i].name.c_str());
            if (!boneNode)
                continue;

            Anim::Mat4 boneWorld = ConvertMatrix(boneNode->node_to_world);
            Anim::Mat4 parentWorld = Anim::Identity();
            int parentIndex = skeleton.bones[i].parentIndex;
            if (parentIndex >= 0)
            {
                const ufbx_node* parentNode = ufbx_find_node(
                    evalScene.get(), skeleton.bones[parentIndex].name.c_str());
                if (parentNode)
                    parentWorld = ConvertMatrix(parentNode->node_to_world);
            }

            Anim::Mat4 relative = Anim::Inverse(parentWorld) * boneWorld;
            Anim::Vec3 pos;
            Anim::Quat rot;
            float scale;
            Anim::Decompose(relative, pos, rot, scale);

            Anim::BoneChannel& channel = clip.channels[i];
            channel.positions.push_back({localTime, pos});
            channel.rotations.push_back({localTime, rot});
            channel.scales.push_back({localTime, {scale, scale, scale}});
        }
    }

    outClip = std::move(clip);
    return true;
}
