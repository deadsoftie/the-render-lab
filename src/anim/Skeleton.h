#pragma once
#include <string>
#include <unordered_map>
#include <vector>

#include "anim/Math/VQS.h"

namespace Anim
{
    struct Bone
    {
        std::string name;
        int parentIndex = -1;

        // Mesh space -> bone space at rest; builds the final skinning matrix each frame.
        Mat4 inverseBindPose;

        // Rest local transform relative to parentIndex (or world, for root); fallback pose for bones a clip doesn't animate.
        VQS localBindPose;
    };

    struct Skeleton
    {
        std::vector<Bone> bones;
        std::unordered_map<std::string, int> boneNameToIndex;
    };

    int FindBone(const Skeleton& skeleton, const std::string& name);
}
