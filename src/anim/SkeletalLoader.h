#pragma once
#include <string>
#include <vector>

#include "anim/AnimationClip.h"
#include "anim/Skeleton.h"

namespace SkeletalLoader
{
    // Builds a Skeleton from every deforming bone reachable through the FBX's node tree
    bool LoadSkeleton(const std::string& path, Anim::Skeleton& outSkeleton);

    // Reads the first animation stack and matches its animated nodes to skeleton bones by name
    bool LoadAnimationClip(const std::string& path,
                           const Anim::Skeleton& skeleton,
                           Anim::AnimationClip& outClip);

    // Reads this file's own skin-cluster bind matrices and matches them to skeleton's bones
    bool LoadInverseBindPoses(const std::string& path,
                              const Anim::Skeleton& skeleton,
                              std::vector<Anim::Mat4>& outInverseBindPoses);
}  // namespace SkeletalLoader
