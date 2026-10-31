#include "anim/Animator.h"

#include <algorithm>
#include <cmath>
#include <utility>

namespace Anim
{
    namespace
    {
        Vec3 SamplePositionAt(const std::vector<PositionKey>& keys, float t, Vec3 fallback)
        {
            if (keys.empty())
                return fallback;
            if (keys.size() == 1 || t <= keys.front().time)
                return keys.front().value;
            if (t >= keys.back().time)
                return keys.back().value;

            for (size_t i = 0; i + 1 < keys.size(); ++i)
            {
                if (t >= keys[i].time && t <= keys[i + 1].time)
                {
                    float span = keys[i + 1].time - keys[i].time;
                    float u = span > 1e-6f ? (t - keys[i].time) / span : 0.0f;
                    return Lerp(keys[i].value, keys[i + 1].value, u);
                }
            }
            return keys.back().value;
        }

        Vec3 SampleScaleAt(const std::vector<ScaleKey>& keys, float t, Vec3 fallback)
        {
            if (keys.empty())
                return fallback;
            if (keys.size() == 1 || t <= keys.front().time)
                return keys.front().value;
            if (t >= keys.back().time)
                return keys.back().value;

            for (size_t i = 0; i + 1 < keys.size(); ++i)
            {
                if (t >= keys[i].time && t <= keys[i + 1].time)
                {
                    float span = keys[i + 1].time - keys[i].time;
                    float u = span > 1e-6f ? (t - keys[i].time) / span : 0.0f;
                    return Lerp(keys[i].value, keys[i + 1].value, u);
                }
            }
            return keys.back().value;
        }

        Quat SampleRotationAt(const std::vector<RotationKey>& keys, float t, Quat fallback)
        {
            if (keys.empty())
                return fallback;
            if (keys.size() == 1 || t <= keys.front().time)
                return keys.front().value;
            if (t >= keys.back().time)
                return keys.back().value;

            for (size_t i = 0; i + 1 < keys.size(); ++i)
            {
                if (t >= keys[i].time && t <= keys[i + 1].time)
                {
                    float span = keys[i + 1].time - keys[i].time;
                    float u = span > 1e-6f ? (t - keys[i].time) / span : 0.0f;
                    return Slerp(keys[i].value, keys[i + 1].value, u);
                }
            }
            return keys.back().value;
        }

        // VQS carries one uniform scale, so non-uniform FBX scale keys are averaged.
        float AverageScale(const Vec3& s)
        {
            return (s.x + s.y + s.z) / 3.0f;
        }

        // Picks whichever of rot/pos/scale is present (preferring rotation) as the canonical
        // timeline and samples the other two against it, collapsing to one VQS keyframe list per
        // bone.
        std::vector<Keyframe> BuildUnifiedKeyframes(const BoneChannel& channel, const VQS& fallback)
        {
            std::vector<Keyframe> result;
            if (!HasChannel(channel))
                return result;

            Vec3 fallbackScale{fallback.s, fallback.s, fallback.s};

            if (!channel.rotations.empty())
            {
                result.reserve(channel.rotations.size());
                for (const RotationKey& rk : channel.rotations)
                {
                    Keyframe kf;
                    kf.time = rk.time;
                    kf.pose.q = rk.value;
                    kf.pose.v = SamplePositionAt(channel.positions, rk.time, fallback.v);
                    kf.pose.s = AverageScale(SampleScaleAt(channel.scales, rk.time, fallbackScale));
                    result.push_back(kf);
                }
                return result;
            }

            if (!channel.positions.empty())
            {
                result.reserve(channel.positions.size());
                for (const PositionKey& pk : channel.positions)
                {
                    Keyframe kf;
                    kf.time = pk.time;
                    kf.pose.v = pk.value;
                    kf.pose.q = SampleRotationAt(channel.rotations, pk.time, fallback.q);
                    kf.pose.s = AverageScale(SampleScaleAt(channel.scales, pk.time, fallbackScale));
                    result.push_back(kf);
                }
                return result;
            }

            result.reserve(channel.scales.size());
            for (const ScaleKey& sk : channel.scales)
            {
                Keyframe kf;
                kf.time = sk.time;
                kf.pose.s = AverageScale(sk.value);
                kf.pose.v = SamplePositionAt(channel.positions, sk.time, fallback.v);
                kf.pose.q = SampleRotationAt(channel.rotations, sk.time, fallback.q);
                result.push_back(kf);
            }
            return result;
        }

        // Finds the keyframe pair around t and the 0..1 blend factor between them; clamps outside
        // the clip.
        void FindBracket(const std::vector<Keyframe>& keys, float t, int& k0, int& k1, float& u)
        {
            int last = static_cast<int>(keys.size()) - 1;
            if (last <= 0 || t <= keys.front().time)
            {
                k0 = k1 = 0;
                u = 0.0f;
                return;
            }
            if (t >= keys.back().time)
            {
                k0 = k1 = last;
                u = 0.0f;
                return;
            }

            for (int i = 0; i < last; ++i)
            {
                if (t >= keys[i].time && t <= keys[i + 1].time)
                {
                    k0 = i;
                    k1 = i + 1;
                    float span = keys[i + 1].time - keys[i].time;
                    u = span > 1e-6f ? (t - keys[i].time) / span : 0.0f;
                    return;
                }
            }
            k0 = k1 = last;
            u = 0.0f;
        }

        // iSlerp/iVQS replay the step chain from the segment start every call rather than
        // persisting across frames, since scrubbing can jump backward and steps can't be undone.
        VQS EvaluateBoneLocalVQS(const Animator& animator,
                                 const std::vector<Keyframe>& keys,
                                 int boneIndex,
                                 float timeTicks)
        {
            if (keys.empty())
                return animator.skeleton->bones[boneIndex].localBindPose;
            if (keys.size() == 1)
                return keys[0].pose;

            int k0, k1;
            float u;
            FindBracket(keys, timeTicks, k0, k1, u);
            if (k0 == k1)
                return keys[k0].pose;

            const VQS& a = keys[k0].pose;
            const VQS& b = keys[k1].pose;

            switch (animator.mode)
            {
                case InterpolationMode::Lerp:
                    return Lerp(a, b, u);
                case InterpolationMode::Slerp:
                    return Slerp(a, b, u);
                case InterpolationMode::ELerp:
                    return ELerp(a, b, u);
                case InterpolationMode::ISlerp:
                {
                    // Take as many fixed-size steps as u has reached.
                    ISlerpSegment seg = PrepareISlerp(a.q, b.q, animator.incrementalSteps);
                    int n = std::clamp(static_cast<int>(u * animator.incrementalSteps),
                                       0,
                                       animator.incrementalSteps);
                    Quat q = a.q;
                    for (int i = 0; i < n; ++i)
                        q = StepISlerp(q, seg);
                    return {Lerp(a.v, b.v, u), q, a.s + (b.s - a.s) * u};
                }
                case InterpolationMode::IVQS:
                {
                    IVQSSegment seg = PrepareIVQS(a, b, animator.incrementalSteps);
                    int n = std::clamp(static_cast<int>(u * animator.incrementalSteps),
                                       0,
                                       animator.incrementalSteps);
                    VQS pose = a;
                    for (int i = 0; i < n; ++i)
                        pose = StepIVQS(pose, seg);
                    return pose;
                }
            }
            return a;
        }

        float NormalizedTime(const Animator& animator)
        {
            if (!animator.clip || animator.clip->duration <= 0.0f)
                return 0.0f;
            return animator.timeTicks / animator.clip->duration;
        }

        void ClearBlend(Animator& animator)
        {
            animator.blendFromClip = nullptr;
            animator.blendFromKeyframes.clear();
            animator.blendProgress = 0.0f;
            animator.blendSeconds = 0.0f;
        }

        void EvaluatePose(Animator& animator)
        {
            if (!animator.skeleton)
                return;

            const Skeleton& skeleton = *animator.skeleton;
            size_t boneCount = skeleton.bones.size();
            if (animator.worldPose.size() != boneCount)
            {
                animator.worldPose.assign(boneCount, VQS{});
                animator.skinningMatrices.assign(boneCount, Identity());
            }

            // A swapped mesh supplies its own inverse bind poses; otherwise use the skeleton's.
            bool hasMeshBindPose = animator.meshInverseBindPose.size() == boneCount;

            // Both layers share one normalized phase, so their foot contacts stay aligned.
            bool blending = animator.blendFromClip != nullptr;
            float blendWeight = BlendWeight(animator);
            float blendFromTime =
                blending ? NormalizedTime(animator) * animator.blendFromClip->duration : 0.0f;

            // SkeletalLoader always appends a bone after its parent, so parentIndex < i always
            // holds - a single forward pass propagates poses correctly.
            for (size_t i = 0; i < boneCount; ++i)
            {
                const Bone& bone = skeleton.bones[i];
                VQS localVQS = EvaluateBoneLocalVQS(
                    animator, animator.boneKeyframes[i], static_cast<int>(i), animator.timeTicks);
                if (blending)
                {
                    VQS fromVQS = EvaluateBoneLocalVQS(animator,
                                                       animator.blendFromKeyframes[i],
                                                       static_cast<int>(i),
                                                       blendFromTime);
                    localVQS = Slerp(fromVQS, localVQS, blendWeight);
                }
                VQS parentWorld =
                    (bone.parentIndex >= 0) ? animator.worldPose[bone.parentIndex] : VQS{};

                const Mat4& inverseBindPose =
                    hasMeshBindPose ? animator.meshInverseBindPose[i] : bone.inverseBindPose;
                animator.worldPose[i] = Concat(parentWorld, localVQS);
                animator.skinningMatrices[i] = ToMat4(animator.worldPose[i]) * inverseBindPose;
            }
        }
    }  // namespace

    void SetClip(Animator& animator, const Skeleton& skeleton, const AnimationClip& clip)
    {
        animator.skeleton = &skeleton;
        animator.clip = &clip;
        animator.timeTicks = 0.0f;
        ClearBlend(animator);

        size_t boneCount = skeleton.bones.size();
        animator.boneKeyframes.assign(boneCount, {});
        for (size_t i = 0; i < boneCount; ++i)
        {
            const BoneChannel& channel =
                (i < clip.channels.size()) ? clip.channels[i] : BoneChannel{};
            animator.boneKeyframes[i] =
                BuildUnifiedKeyframes(channel, skeleton.bones[i].localBindPose);
        }

        if (animator.meshInverseBindPose.size() != boneCount)
        {
            animator.meshInverseBindPose.resize(boneCount);
            for (size_t i = 0; i < boneCount; ++i)
                animator.meshInverseBindPose[i] = skeleton.bones[i].inverseBindPose;
        }

        animator.worldPose.assign(boneCount, VQS{});
        animator.skinningMatrices.assign(boneCount, Identity());
        EvaluatePose(animator);
    }

    void BeginBlendTo(Animator& animator,
                      const Skeleton& skeleton,
                      const AnimationClip& clip,
                      float seconds)
    {
        if (&clip == animator.clip)
        {
            if (seconds <= 0.0f)
                EndBlend(animator);
            return;
        }

        float phase = NormalizedTime(animator);

        if (!animator.clip || animator.skeleton != &skeleton || seconds <= 0.0f)
        {
            SetClip(animator, skeleton, clip);
            SeekTo(animator, phase * clip.duration);
            return;
        }

        if (&clip == animator.blendFromClip)
        {
            // The smoothstep weight is symmetric, so 1 - progress lands on the same pose.
            std::swap(animator.boneKeyframes, animator.blendFromKeyframes);
            std::swap(animator.clip, animator.blendFromClip);
            animator.blendProgress = 1.0f - animator.blendProgress;
            animator.blendSeconds = seconds;
            SeekTo(animator, phase * clip.duration);
            return;
        }

        const AnimationClip* previous = animator.clip;
        std::vector<std::vector<Keyframe>> previousKeyframes = std::move(animator.boneKeyframes);
        SetClip(animator, skeleton, clip);
        animator.blendFromClip = previous;
        animator.blendFromKeyframes = std::move(previousKeyframes);
        animator.blendSeconds = seconds;
        SeekTo(animator, phase * clip.duration);
    }

    void AdvanceBlend(Animator& animator, float deltaSeconds)
    {
        if (!animator.blendFromClip)
            return;

        animator.blendProgress += deltaSeconds / animator.blendSeconds;
        if (animator.blendProgress >= 1.0f)
            ClearBlend(animator);
    }

    void EndBlend(Animator& animator)
    {
        ClearBlend(animator);
        EvaluatePose(animator);
    }

    float BlendWeight(const Animator& animator)
    {
        if (!animator.blendFromClip)
            return 1.0f;
        float t = std::clamp(animator.blendProgress, 0.0f, 1.0f);
        return t * t * (3.0f - 2.0f * t);
    }

    void SetMeshBindPose(Animator& animator, std::vector<Mat4> inverseBindPoses)
    {
        animator.meshInverseBindPose = std::move(inverseBindPoses);
        EvaluatePose(animator);
    }

    void Advance(Animator& animator, float deltaSeconds)
    {
        if (!animator.skeleton)
            return;

        if (animator.playing && animator.clip)
        {
            animator.timeTicks += deltaSeconds * animator.clip->ticksPerSecond;
            float duration = animator.clip->duration;
            if (duration > 0.0f)
            {
                if (animator.looping)
                {
                    animator.timeTicks = std::fmod(animator.timeTicks, duration);
                    if (animator.timeTicks < 0.0f)
                        animator.timeTicks += duration;
                }
                else
                {
                    animator.timeTicks = std::clamp(animator.timeTicks, 0.0f, duration);
                }
            }
        }

        EvaluatePose(animator);
    }

    void SeekTo(Animator& animator, float timeTicks)
    {
        animator.timeTicks = timeTicks;
        EvaluatePose(animator);
    }

    float DurationSeconds(const Animator& animator)
    {
        if (!animator.clip || animator.clip->ticksPerSecond <= 0.0f)
            return 0.0f;
        return animator.clip->duration / animator.clip->ticksPerSecond;
    }

    float TimeSeconds(const Animator& animator)
    {
        if (!animator.clip || animator.clip->ticksPerSecond <= 0.0f)
            return 0.0f;
        return animator.timeTicks / animator.clip->ticksPerSecond;
    }
}  // namespace Anim
