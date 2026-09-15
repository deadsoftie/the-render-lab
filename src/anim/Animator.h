#pragma once
#include <vector>

#include "anim/AnimationClip.h"
#include "anim/Skeleton.h"

namespace Anim
{
    enum class InterpolationMode
    {
        Lerp,
        Slerp,
        ELerp,
        ISlerp,
        IVQS
    };

    struct Keyframe
    {
        float time = 0.0f;  // ticks
        VQS pose;
    };

    struct Animator
    {
        const Skeleton* skeleton = nullptr;
        const AnimationClip* clip = nullptr;

        // Per-bone pos/rot/scale channels collapsed into one {time, VQS} list each; see BuildUnifiedKeyframes in Animator.cpp.
        std::vector<std::vector<Keyframe>> boneKeyframes;

        float timeTicks = 0.0f;
        bool playing = true;
        bool looping = true;
        InterpolationMode mode = InterpolationMode::Slerp;
        int incrementalSteps = 16;  // segment resolution for ISlerp/IVQS

        // Outputs, rebuilt every Advance()/SeekTo() call; skinningMatrices[i] = worldPose[i]*bone.inverseBindPose, for GPU upload (see DrawSkinnedObject).
        std::vector<VQS> worldPose;           // parallel to skeleton->bones, world space
        std::vector<Mat4> skinningMatrices;  // parallel to skeleton->bones
    };

    // Rebuilds boneKeyframes for the new clip and resets playback to time 0.
    void SetClip(Animator& animator, const Skeleton& skeleton, const AnimationClip& clip);

    // Advances playback (if playing) and re-evaluates the pose.
    void Advance(Animator& animator, float deltaSeconds);

    // Jumps to an explicit time and re-evaluates the pose, without touching `playing`.
    void SeekTo(Animator& animator, float timeTicks);

    float DurationSeconds(const Animator& animator);
    float TimeSeconds(const Animator& animator);
}
