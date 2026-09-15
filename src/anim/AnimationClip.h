#pragma once
#include <string>
#include <vector>

#include "anim/Math/Quat.h"
#include "anim/Math/Vec3.h"

namespace Anim
{
    struct PositionKey
    {
        float time = 0.0f;  // ticks, same unit as AnimationClip::duration
        Vec3 value;
    };

    struct RotationKey
    {
        float time = 0.0f;
        Quat value;
    };

    struct ScaleKey
    {
        float time = 0.0f;
        Vec3 value;
    };

    struct BoneChannel
    {
        std::vector<PositionKey> positions;
        std::vector<RotationKey> rotations;
        std::vector<ScaleKey> scales;
    };

    struct AnimationClip
    {
        std::string name;
        float duration = 0.0f;
        float ticksPerSecond = 25.0f;

        // Parallel to Skeleton::bones. An empty channel means this clip doesn't
        // animate that bone; Bone::localBindPose should be used instead.
        std::vector<BoneChannel> channels;
    };

    bool HasChannel(const BoneChannel& channel);
}
