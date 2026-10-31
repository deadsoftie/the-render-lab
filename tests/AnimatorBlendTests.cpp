#include "MotionTests.h"
#include "TestFramework.h"
#include "anim/Animator.h"

#include <cmath>

namespace
{
    using namespace Anim;

    // Root translation runs from start to start + travel across the clip.
    AnimationClip MakeClip(float duration, const Vec3& start, const Vec3& travel)
    {
        AnimationClip clip;
        clip.duration = duration;
        clip.ticksPerSecond = 1.0f;
        clip.channels.resize(1);
        clip.channels[0].positions.push_back({0.0f, start});
        clip.channels[0].positions.push_back({duration, start + travel});
        return clip;
    }

    bool RootIs(const Animator& animator, const Vec3& expected)
    {
        const Vec3& v = animator.worldPose[0].v;
        return std::abs(v.x - expected.x) < 1e-4f && std::abs(v.y - expected.y) < 1e-4f &&
               std::abs(v.z - expected.z) < 1e-4f;
    }
}

void RunAnimatorBlendTests()
{
    using namespace Anim;

    Skeleton skeleton;
    skeleton.bones.resize(1);
    AnimationClip walk = MakeClip(1.0f, {0, 0, 0}, {1, 0, 0});
    AnimationClip run = MakeClip(2.0f, {0, 10, 0}, {2, 0, 0});

    Animator animator;
    CHECK(BlendWeight(animator) == 1.0f);
    SetClip(animator, skeleton, walk);
    SeekTo(animator, 0.5f);
    CHECK(RootIs(animator, {0.5f, 0, 0}));

    // Starting a blend keeps the phase and the pose.
    BeginBlendTo(animator, skeleton, run, 1.0f);
    CHECK(animator.clip == &run);
    CHECK(animator.blendFromClip == &walk);
    CHECK(std::abs(animator.timeTicks - 1.0f) < 1e-5f);
    CHECK(BlendWeight(animator) == 0.0f);
    CHECK(RootIs(animator, {0.5f, 0, 0}));

    // Halfway, both layers contribute at the shared phase.
    AdvanceBlend(animator, 0.5f);
    CHECK(std::abs(BlendWeight(animator) - 0.5f) < 1e-5f);
    SeekTo(animator, 1.0f);
    CHECK(RootIs(animator, {0.75f, 5, 0}));

    // The source layer follows the target's phase, not its own clock.
    Advance(animator, 0.5f);
    CHECK(std::abs(animator.timeTicks - 1.5f) < 1e-5f);
    CHECK(RootIs(animator, {1.125f, 5, 0}));

    // Reversing mid-blend does not move the pose.
    AdvanceBlend(animator, 0.25f);
    float weightBefore = BlendWeight(animator);
    SeekTo(animator, animator.timeTicks);
    Vec3 poseBefore = animator.worldPose[0].v;
    BeginBlendTo(animator, skeleton, walk, 1.0f);
    CHECK(animator.clip == &walk);
    CHECK(animator.blendFromClip == &run);
    CHECK(std::abs(animator.timeTicks - 0.75f) < 1e-5f);
    CHECK(std::abs(BlendWeight(animator) - (1.0f - weightBefore)) < 1e-5f);
    CHECK(RootIs(animator, poseBefore));

    // Choosing the current target leaves the running blend alone.
    float progressBefore = animator.blendProgress;
    BeginBlendTo(animator, skeleton, walk, 1.0f);
    CHECK(animator.blendFromClip == &run);
    CHECK(animator.blendProgress == progressBefore);

    // A finished blend releases the source layer and plays the pure target.
    AdvanceBlend(animator, 10.0f);
    CHECK(animator.blendFromClip == nullptr);
    CHECK(animator.blendFromKeyframes.empty());
    CHECK(BlendWeight(animator) == 1.0f);
    SeekTo(animator, 0.25f);
    CHECK(RootIs(animator, {0.25f, 0, 0}));

    // A zero-length blend cuts, keeping the phase.
    BeginBlendTo(animator, skeleton, run, 0.0f);
    CHECK(animator.clip == &run);
    CHECK(animator.blendFromClip == nullptr);
    CHECK(RootIs(animator, {0.5f, 10, 0}));

    // EndBlend snaps to the target; SetClip cancels and restarts.
    BeginBlendTo(animator, skeleton, walk, 1.0f);
    CHECK(animator.blendFromClip == &run);
    EndBlend(animator);
    CHECK(animator.blendFromClip == nullptr);
    CHECK(RootIs(animator, {0.25f, 0, 0}));
    BeginBlendTo(animator, skeleton, run, 1.0f);
    SetClip(animator, skeleton, walk);
    CHECK(animator.blendFromClip == nullptr);
    CHECK(animator.timeTicks == 0.0f);
    CHECK(RootIs(animator, {0, 0, 0}));

    // A blend paused mid-way can still be cut on its current target.
    BeginBlendTo(animator, skeleton, run, 1.0f);
    AdvanceBlend(animator, 0.5f);
    BeginBlendTo(animator, skeleton, run, 0.0f);
    CHECK(animator.blendFromClip == nullptr);
    CHECK(RootIs(animator, {0, 10, 0}));
}
