#pragma once
#include <string>

#include "anim/AnimationClip.h"
#include "anim/Skeleton.h"

namespace SkeletalLoader
{
    // Builds a Skeleton from every deforming bone (aiMesh::mBones) reachable through
    // the FBX's node tree. Non-deforming nodes (locators, end sites) are skipped and
    // never become bones; a bone's parentIndex points at its nearest bone ancestor,
    // possibly skipping such nodes in between.
    bool LoadSkeleton(const std::string& path, Anim::Skeleton& outSkeleton);

    // Reads the first aiAnimation in the FBX and matches its channels to skeleton
    // bones by name. Bones with no matching channel are left with an empty
    // BoneChannel (see AnimationClip.h).
    bool LoadAnimationClip(const std::string& path, const Anim::Skeleton& skeleton,
                           Anim::AnimationClip& outClip);
}
