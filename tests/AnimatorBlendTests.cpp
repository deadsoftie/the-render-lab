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

    bool SameVQS(const VQS& a, const VQS& b, float epsilon)
    {
        return std::abs(a.v.x - b.v.x) <= epsilon && std::abs(a.v.y - b.v.y) <= epsilon &&
               std::abs(a.v.z - b.v.z) <= epsilon && std::abs(a.q.x - b.q.x) <= epsilon &&
               std::abs(a.q.y - b.q.y) <= epsilon && std::abs(a.q.z - b.q.z) <= epsilon &&
               std::abs(a.q.w - b.q.w) <= epsilon && std::abs(a.s - b.s) <= epsilon;
    }

    bool SameMatrix(const Mat4& a, const Mat4& b)
    {
        for (int i = 0; i < 16; ++i)
            if (a.m[i] != b.m[i])
                return false;
        return true;
    }

    void CheckRebuildWorldPose()
    {
        Skeleton skeleton;
        skeleton.bones.resize(4);
        skeleton.bones[0].localBindPose = {{0, 1, 0}, FromAxisAngle({0, 1, 0}, 0.3f), 0.01f};
        skeleton.bones[1].parentIndex = 0;
        skeleton.bones[1].localBindPose = {{0, 50, 0}, FromAxisAngle({0, 0, 1}, 0.2f), 1.0f};
        skeleton.bones[2].parentIndex = 1;
        skeleton.bones[2].localBindPose = {{0, 30, 0}, FromAxisAngle({1, 0, 0}, -0.4f), 1.0f};
        skeleton.bones[3].parentIndex = 0;
        skeleton.bones[3].localBindPose = {{20, 0, 0}, Quat{}, 1.0f};

        AnimationClip clip;
        clip.duration = 1.0f;
        clip.ticksPerSecond = 1.0f;
        clip.channels.resize(4);
        clip.channels[1].rotations.push_back({0.0f, FromAxisAngle({0, 0, 1}, 0.0f)});
        clip.channels[1].rotations.push_back({1.0f, FromAxisAngle({0, 0, 1}, 1.0f)});

        Animator animator;
        CHECK(animator.localPose.empty());
        SetClip(animator, skeleton, clip);
        SeekTo(animator, 0.4f);
        CHECK(animator.localPose.size() == skeleton.bones.size());

        for (std::size_t i = 0; i < skeleton.bones.size(); ++i)
        {
            int parent = skeleton.bones[i].parentIndex;
            VQS expected =
                Concat(parent >= 0 ? animator.worldPose[parent] : VQS{}, animator.localPose[i]);
            CHECK(SameVQS(animator.worldPose[i], expected, 0));
        }

        const std::vector<VQS> evaluatedWorld = animator.worldPose;
        const std::vector<Mat4> evaluatedSkinning = animator.skinningMatrices;
        RebuildWorldPose(animator);
        for (std::size_t i = 0; i < skeleton.bones.size(); ++i)
        {
            CHECK(SameVQS(animator.worldPose[i], evaluatedWorld[i], 0));
            CHECK(SameMatrix(animator.skinningMatrices[i], evaluatedSkinning[i]));
        }

        animator.localPose[1].q = FromAxisAngle({1, 0, 0}, 0.8f);
        RebuildWorldPose(animator);
        CHECK(SameVQS(animator.worldPose[0], evaluatedWorld[0], 0));
        CHECK(SameVQS(animator.worldPose[3], evaluatedWorld[3], 0));
        CHECK(SameVQS(animator.worldPose[1],
                      Concat(animator.worldPose[0], animator.localPose[1]),
                      0));
        CHECK(SameVQS(animator.worldPose[2],
                      Concat(animator.worldPose[1], animator.localPose[2]),
                      0));
        CHECK(!SameVQS(animator.worldPose[2], evaluatedWorld[2], 1e-4f));
        CHECK(!SameMatrix(animator.skinningMatrices[2], evaluatedSkinning[2]));

        SeekTo(animator, 0.4f);
        for (std::size_t i = 0; i < skeleton.bones.size(); ++i)
            CHECK(SameVQS(animator.worldPose[i], evaluatedWorld[i], 0));
    }
}  // namespace

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

    CheckRebuildWorldPose();
}
