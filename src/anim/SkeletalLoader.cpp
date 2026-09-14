#include "pch.h"
#include "anim/SkeletalLoader.h"

#include <unordered_map>

#include <assimp/Importer.hpp>
#include <assimp/anim.h>
#include <assimp/scene.h>

namespace
{
    Anim::Mat4 ConvertMatrix(const aiMatrix4x4& m)
    {
        // aiMatrix4x4 is row-major (a1..a4 = row 0); Anim::Mat4 is column-major.
        Anim::Mat4 result;
        result.m[0] = m.a1;
        result.m[4] = m.a2;
        result.m[8] = m.a3;
        result.m[12] = m.a4;
        result.m[1] = m.b1;
        result.m[5] = m.b2;
        result.m[9] = m.b3;
        result.m[13] = m.b4;
        result.m[2] = m.c1;
        result.m[6] = m.c2;
        result.m[10] = m.c3;
        result.m[14] = m.c4;
        result.m[3] = m.d1;
        result.m[7] = m.d2;
        result.m[11] = m.d3;
        result.m[15] = m.d4;
        return result;
    }

    Anim::Vec3 ConvertVec3(const aiVector3D& v)
    {
        return {v.x, v.y, v.z};
    }

    Anim::Quat ConvertQuat(const aiQuaternion& q)
    {
        return {q.x, q.y, q.z, q.w};
    }

    struct WalkState
    {
        Anim::Mat4 parentWorldBind = Anim::Identity();
        Anim::Mat4 nearestBoneWorldBind = Anim::Identity();
        int nearestBoneIndex = -1;
    };

    void CollectBones(const aiNode* node, const WalkState& state,
                      const std::unordered_map<std::string, aiMatrix4x4>& offsetMatrices,
                      Anim::Skeleton& outSkeleton)
    {
        Anim::Mat4 worldBind = state.parentWorldBind * ConvertMatrix(node->mTransformation);

        WalkState childState = state;
        childState.parentWorldBind = worldBind;

        std::string name = node->mName.C_Str();
        auto offsetIt = offsetMatrices.find(name);
        if (offsetIt != offsetMatrices.end())
        {
            Anim::Bone bone;
            bone.name = name;
            bone.parentIndex = state.nearestBoneIndex;
            bone.inverseBindPose = ConvertMatrix(offsetIt->second);

            Anim::Mat4 relativeToParent = Anim::Inverse(state.nearestBoneWorldBind) * worldBind;
            Anim::Decompose(relativeToParent, bone.localBindPose.v, bone.localBindPose.q,
                            bone.localBindPose.s);

            int index = static_cast<int>(outSkeleton.bones.size());
            outSkeleton.boneNameToIndex[name] = index;
            outSkeleton.bones.push_back(bone);

            childState.nearestBoneWorldBind = worldBind;
            childState.nearestBoneIndex = index;
        }

        for (unsigned int i = 0; i < node->mNumChildren; ++i)
            CollectBones(node->mChildren[i], childState, offsetMatrices, outSkeleton);
    }
}

bool SkeletalLoader::LoadSkeleton(const std::string& path, Anim::Skeleton& outSkeleton)
{
    Assimp::Importer importer;
    const aiScene* scene = importer.ReadFile(path, 0);
    if (!scene || !scene->HasMeshes())
    {
        std::cerr << "[SkeletalLoader] Failed to load " << path << ": "
                  << importer.GetErrorString() << "\n";
        return false;
    }

    std::unordered_map<std::string, aiMatrix4x4> offsetMatrices;
    for (unsigned int m = 0; m < scene->mNumMeshes; ++m)
    {
        const aiMesh* mesh = scene->mMeshes[m];
        for (unsigned int b = 0; b < mesh->mNumBones; ++b)
        {
            const aiBone* bone = mesh->mBones[b];
            offsetMatrices.emplace(bone->mName.C_Str(), bone->mOffsetMatrix);
        }
    }

    if (offsetMatrices.empty())
    {
        std::cerr << "[SkeletalLoader] No bones found in " << path << "\n";
        return false;
    }

    Anim::Skeleton skeleton;
    CollectBones(scene->mRootNode, WalkState{}, offsetMatrices, skeleton);

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
    Assimp::Importer importer;
    const aiScene* scene = importer.ReadFile(path, 0);
    if (!scene || scene->mNumAnimations == 0)
    {
        std::cerr << "[SkeletalLoader] No animation found in " << path << "\n";
        return false;
    }

    const aiAnimation* anim = scene->mAnimations[0];

    Anim::AnimationClip clip;
    clip.name = anim->mName.length > 0 ? anim->mName.C_Str() : path;
    clip.duration = static_cast<float>(anim->mDuration);
    clip.ticksPerSecond =
        anim->mTicksPerSecond > 0.0001 ? static_cast<float>(anim->mTicksPerSecond) : 25.0f;
    clip.channels.resize(skeleton.bones.size());

    for (unsigned int c = 0; c < anim->mNumChannels; ++c)
    {
        const aiNodeAnim* channel = anim->mChannels[c];
        int boneIndex = Anim::FindBone(skeleton, channel->mNodeName.C_Str());
        if (boneIndex < 0)
            continue;

        Anim::BoneChannel& out = clip.channels[boneIndex];

        out.positions.reserve(channel->mNumPositionKeys);
        for (unsigned int k = 0; k < channel->mNumPositionKeys; ++k)
        {
            const aiVectorKey& key = channel->mPositionKeys[k];
            out.positions.push_back({static_cast<float>(key.mTime), ConvertVec3(key.mValue)});
        }

        out.rotations.reserve(channel->mNumRotationKeys);
        for (unsigned int k = 0; k < channel->mNumRotationKeys; ++k)
        {
            const aiQuatKey& key = channel->mRotationKeys[k];
            out.rotations.push_back({static_cast<float>(key.mTime), ConvertQuat(key.mValue)});
        }

        out.scales.reserve(channel->mNumScalingKeys);
        for (unsigned int k = 0; k < channel->mNumScalingKeys; ++k)
        {
            const aiVectorKey& key = channel->mScalingKeys[k];
            out.scales.push_back({static_cast<float>(key.mTime), ConvertVec3(key.mValue)});
        }

        if (!Anim::HasChannel(out))
            std::cerr << "[SkeletalLoader] Channel for '" << channel->mNodeName.C_Str()
                      << "' in " << path << " has no keys\n";
    }

    outClip = std::move(clip);
    return true;
}
