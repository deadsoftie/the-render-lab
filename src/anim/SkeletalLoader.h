#pragma once
#include <string>

#include "anim/AnimationClip.h"
#include "anim/Skeleton.h"

namespace SkeletalLoader
{
    // Builds a Skeleton from every deforming bone reachable through the FBX's node tree; parentIndex points at the nearest bone ancestor, skipping non-bone nodes (locators, end sites) in between.
    bool LoadSkeleton(const std::string& path, Anim::Skeleton& outSkeleton);

    // Reads the first animation stack and matches its animated nodes to skeleton bones by name; unmatched bones keep an empty BoneChannel (see AnimationClip.h).
    bool LoadAnimationClip(const std::string& path, const Anim::Skeleton& skeleton,
                           Anim::AnimationClip& outClip);
}
