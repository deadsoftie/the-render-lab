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

        // Per-bone pos/rot/scale channels collapsed into one {time, VQS} list each; see
        // BuildUnifiedKeyframes in Animator.cpp.
        std::vector<std::vector<Keyframe>> boneKeyframes;

        // Cross-fade source layer, sampled at the target clip's normalized phase; a null
        // blendFromClip means no blend is running.
        const AnimationClip* blendFromClip = nullptr;
        std::vector<std::vector<Keyframe>> blendFromKeyframes;
        float blendProgress = 0.0f;  // linear 0..1; BlendWeight applies the easing
        float blendSeconds = 0.0f;

        float timeTicks = 0.0f;
        bool playing = true;
        bool looping = true;
        InterpolationMode mode = InterpolationMode::Slerp;
        int incrementalSteps = 16;  // segment resolution for ISlerp/IVQS

        // Per-bone inverse bind pose for whichever mesh is currently equipped on this skeleton
        std::vector<Mat4> meshInverseBindPose;

        std::vector<VQS> worldPose;          // parallel to skeleton->bones, world space
        std::vector<Mat4> skinningMatrices;  // parallel to skeleton->bones
    };

    // Rebuilds boneKeyframes for the new clip, cancels any blend and resets playback to time 0
    void SetClip(Animator& animator, const Skeleton& skeleton, const AnimationClip& clip);

    // Cross-fades to clip, keeping the normalized phase. Choosing the blend's source clip reverses
    // it without a pop; seconds <= 0 cuts immediately.
    void BeginBlendTo(Animator& animator,
                      const Skeleton& skeleton,
                      const AnimationClip& clip,
                      float seconds);

    // Moves the blend clock only; the next Advance or SeekTo evaluates the pose.
    void AdvanceBlend(Animator& animator, float deltaSeconds);

    // Finishes any blend now, leaving the pure target clip.
    void EndBlend(Animator& animator);

    // Eased target weight: 0 is all blendFromClip, 1 is all clip. 1 when no blend is running.
    float BlendWeight(const Animator& animator);

    // Overrides the currently-equipped mesh's per-bone bind pose and re-evaluates the pose
    void SetMeshBindPose(Animator& animator, std::vector<Mat4> inverseBindPoses);

    // Advances playback (if playing) and re-evaluates the pose.
    void Advance(Animator& animator, float deltaSeconds);

    // Jumps to an explicit time and re-evaluates the pose, without touching `playing`.
    void SeekTo(Animator& animator, float timeTicks);

    float DurationSeconds(const Animator& animator);
    float TimeSeconds(const Animator& animator);
}  // namespace Anim
