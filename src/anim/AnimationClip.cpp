#include "anim/AnimationClip.h"

namespace Anim
{
    bool HasChannel(const BoneChannel& channel)
    {
        return !channel.positions.empty() || !channel.rotations.empty() ||
               !channel.scales.empty();
    }
}
