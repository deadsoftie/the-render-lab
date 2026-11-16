#include "anim/IK/Fabrik.h"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <utility>

namespace Anim
{
    namespace
    {
        bool IsFinite(const Vec3& v)
        {
            return std::isfinite(v.x) && std::isfinite(v.y) && std::isfinite(v.z);
        }

        Vec3 Direction(const Vec3& from, const Vec3& to, const Vec3& fallback)
        {
            Vec3 d = to - from;
            float length = Length(d);
            if (!(length > 1e-8f))
                return fallback;
            return d * (1.0f / length);
        }

        void UpdateSubtree(const Skeleton& skeleton,
                           int subtreeRoot,
                           const std::vector<VQS>& localPose,
                           std::vector<VQS>& worldPose)
        {
            std::size_t count = skeleton.bones.size();
            std::vector<char> dirty(count, 0);
            for (std::size_t i = static_cast<std::size_t>(subtreeRoot); i < count; ++i)
            {
                int parent = skeleton.bones[i].parentIndex;
                bool isRoot = static_cast<int>(i) == subtreeRoot;
                if (!isRoot && (parent < 0 || !dirty[parent]))
                    continue;
                dirty[i] = 1;
                VQS parentWorld = parent >= 0 ? worldPose[parent] : VQS{};
                worldPose[i] = Concat(parentWorld, localPose[i]);
            }
        }

        float SignedAngle(const Vec3& from, const Vec3& to, const Vec3& axis)
        {
            return std::atan2(Dot(Cross(from, to), axis), Dot(from, to));
        }

        Vec3 ClampToCone(const Vec3& direction, const Vec3& center, float maxAngle)
        {
            float angle = std::acos(std::clamp(Dot(direction, center), -1.0f, 1.0f));
            if (angle <= maxAngle)
                return direction;
            Vec3 axis = Cross(center, direction);
            if (Dot(axis, axis) < 1e-12f)
            {
                axis = Cross(center, Vec3{1.0f, 0.0f, 0.0f});
                if (Dot(axis, axis) < 1e-6f)
                    axis = Cross(center, Vec3{0.0f, 1.0f, 0.0f});
            }
            return Rotate(FromAxisAngle(axis, maxAngle), center);
        }

        Vec3 RejectFrom(const Vec3& v, const Vec3& unitAxis)
        {
            return v - unitAxis * Dot(v, unitAxis);
        }
    }  // namespace

    FabrikResult SolveFabrik(std::vector<Vec3>& joints,
                             const Vec3& target,
                             const FabrikSettings& settings)
    {
        if (joints.size() < 2)
            throw std::invalid_argument("A FABRIK chain needs at least two joints.");
        if (!IsFinite(target))
            throw std::invalid_argument("The FABRIK target must be finite.");
        if (!std::isfinite(settings.tolerance) || settings.tolerance < 0.0f ||
            settings.maxIterations < 0)
            throw std::invalid_argument(
                "FABRIK tolerance and iteration limit must be nonnegative.");
        for (const Vec3& joint : joints)
            if (!IsFinite(joint))
                throw std::invalid_argument("FABRIK joint positions must be finite.");

        std::size_t last = joints.size() - 1;
        std::vector<float> lengths(last);
        std::vector<Vec3> restDirections(last);
        float total = 0.0f;
        for (std::size_t i = 0; i < last; ++i)
        {
            lengths[i] = Length(joints[i + 1] - joints[i]);
            restDirections[i] = Direction(joints[i], joints[i + 1], Vec3{0.0f, 1.0f, 0.0f});
            total += lengths[i];
        }

        const Vec3 root = joints[0];
        FabrikResult result;
        float distance = Length(target - root);
        result.reachable = distance <= total;

        if (!result.reachable)
        {
            Vec3 direction = Direction(root, target, restDirections[0]);
            for (std::size_t i = 0; i < last; ++i)
                joints[i + 1] = joints[i] + direction * lengths[i];
            result.error = Length(joints[last] - target);
            result.converged = result.error <= settings.tolerance;
            return result;
        }

        result.error = Length(joints[last] - target);
        while (result.error > settings.tolerance && result.iterations < settings.maxIterations)
        {
            joints[last] = target;
            for (std::size_t i = last; i-- > 0;)
                joints[i] = joints[i + 1] +
                            Direction(joints[i + 1], joints[i], -restDirections[i]) * lengths[i];

            joints[0] = root;
            for (std::size_t i = 0; i < last; ++i)
                joints[i + 1] =
                    joints[i] + Direction(joints[i], joints[i + 1], restDirections[i]) * lengths[i];

            result.error = Length(joints[last] - target);
            ++result.iterations;
        }
        result.converged = result.error <= settings.tolerance;
        return result;
    }

    std::vector<int> BuildChain(const Skeleton& skeleton, int rootBone, int effectorBone)
    {
        int count = static_cast<int>(skeleton.bones.size());
        if (rootBone < 0 || rootBone >= count || effectorBone < 0 || effectorBone >= count)
            return {};

        std::vector<int> chain;
        for (int bone = effectorBone; bone >= 0; bone = skeleton.bones[bone].parentIndex)
        {
            chain.push_back(bone);
            if (bone == rootBone)
            {
                std::reverse(chain.begin(), chain.end());
                return chain;
            }
        }
        return {};
    }

    std::vector<Vec3> GatherChainPositions(const std::vector<VQS>& worldPose,
                                           const std::vector<int>& chain)
    {
        std::vector<Vec3> positions;
        positions.reserve(chain.size());
        for (int bone : chain)
            positions.push_back(worldPose.at(bone).v);
        return positions;
    }

    void ApplyChainPositions(const Skeleton& skeleton,
                             const std::vector<int>& chain,
                             const std::vector<Vec3>& positions,
                             std::vector<VQS>& localPose,
                             std::vector<VQS>& worldPose)
    {
        std::size_t boneCount = skeleton.bones.size();
        if (positions.size() != chain.size())
            throw std::invalid_argument("Chain and position counts must match.");
        if (localPose.size() != boneCount || worldPose.size() != boneCount)
            throw std::invalid_argument("Pose arrays must match the skeleton's bone count.");
        for (int bone : chain)
            if (bone < 0 || static_cast<std::size_t>(bone) >= boneCount)
                throw std::invalid_argument("Chain bone index is out of range.");

        for (std::size_t k = 0; k + 1 < chain.size(); ++k)
        {
            int bone = chain[k];
            int child = chain[k + 1];
            Vec3 oldDirection = worldPose[child].v - worldPose[bone].v;
            Vec3 newDirection = positions[k + 1] - worldPose[bone].v;
            Quat world = Normalize(FromTo(oldDirection, newDirection) * worldPose[bone].q);
            int parent = skeleton.bones[bone].parentIndex;
            Quat parentWorld = parent >= 0 ? worldPose[parent].q : Quat{};
            localPose[bone].q = Normalize(Conjugate(parentWorld) * world);
            UpdateSubtree(skeleton, bone, localPose, worldPose);
        }
    }

    bool HingeFrame(const std::vector<VQS>& localPose,
                    int bone,
                    int child,
                    const Vec3& fallbackAxis,
                    Vec3& straight,
                    Vec3& axis)
    {
        Vec3 z = Normalize(localPose.at(bone).v);
        Vec3 bent = Normalize(Rotate(localPose.at(bone).q, localPose.at(child).v));
        Vec3 h = Cross(z, bent);
        bool fromAnimation = Length(h) >= std::sin(3.0f * 3.14159265f / 180.0f);
        axis = Normalize(fromAnimation ? h : fallbackAxis);
        straight = RejectFrom(z, axis);
        straight = Length(straight) > 1e-6f ? Normalize(straight) : bent;
        return fromAnimation;
    }

    ChainSolveResult SolveChain(const Skeleton& skeleton,
                                const std::vector<IKJoint>& joints,
                                const Vec3& target,
                                const ChainSolveSettings& settings,
                                std::vector<VQS>& localPose,
                                std::vector<VQS>& worldPose)
    {
        std::size_t boneCount = skeleton.bones.size();
        if (joints.size() < 2)
            throw std::invalid_argument("An IK chain needs at least two joints.");
        if (localPose.size() != boneCount || worldPose.size() != boneCount)
            throw std::invalid_argument("Pose arrays must match the skeleton's bone count.");
        if (!IsFinite(target) || !IsFinite(settings.pole))
            throw std::invalid_argument("The IK target and pole must be finite.");
        if (!std::isfinite(settings.tolerance) || settings.tolerance < 0.0f ||
            settings.maxIterations < 0)
            throw std::invalid_argument("IK tolerance and iteration limit must be nonnegative.");
        for (std::size_t k = 0; k < joints.size(); ++k)
        {
            int bone = joints[k].bone;
            if (bone < 0 || static_cast<std::size_t>(bone) >= boneCount)
                throw std::invalid_argument("IK joint bone index is out of range.");
            if (k > 0 && skeleton.bones[bone].parentIndex != joints[k - 1].bone)
                throw std::invalid_argument("Each IK joint must be the child of the previous one.");
        }

        std::size_t n = joints.size();
        std::size_t segments = n - 1;
        std::vector<Vec3> p(n);
        for (std::size_t k = 0; k < n; ++k)
            p[k] = worldPose[joints[k].bone].v;

        std::vector<Quat> animated(segments);
        std::vector<Quat> current(segments);
        std::vector<Vec3> offsets(segments);
        std::vector<Vec3> restDirections(segments);
        std::vector<float> lengths(segments);
        std::vector<char> isHinge(segments, 0);
        std::vector<Vec3> hingeStraight(segments);
        std::vector<Vec3> hingeAxis(segments);
        float total = 0.0f;
        for (std::size_t k = 0; k < segments; ++k)
        {
            animated[k] = localPose[joints[k].bone].q;
            current[k] = animated[k];
            offsets[k] = Normalize(localPose[joints[k + 1].bone].v);
            lengths[k] = Length(p[k + 1] - p[k]);
            restDirections[k] = Direction(p[k], p[k + 1], Vec3{0.0f, 1.0f, 0.0f});
            total += lengths[k];
            if (joints[k].limit.type == JointLimitType::Hinge)
            {
                isHinge[k] = 1;
                HingeFrame(localPose,
                           joints[k].bone,
                           joints[k + 1].bone,
                           joints[k].limit.axis,
                           hingeStraight[k],
                           hingeAxis[k]);
            }
        }

        int rootParent = skeleton.bones[joints[0].bone].parentIndex;
        const Quat rootParentRotation = rootParent >= 0 ? worldPose[rootParent].q : Quat{};
        const Vec3 root = p[0];

        ChainSolveResult result;
        result.reachable = Length(target - root) <= total;
        result.error = Length(p[segments] - target);

        while (result.error > settings.tolerance && result.iterations < settings.maxIterations)
        {
            p[segments] = target;
            for (std::size_t k = segments; k-- > 0;)
                p[k] = p[k + 1] + Direction(p[k + 1], p[k], -restDirections[k]) * lengths[k];

            if (settings.usePole)
            {
                for (std::size_t k = 1; k < segments; ++k)
                {
                    if (!isHinge[k])
                        continue;
                    Vec3 axis = Direction(p[k - 1], p[k + 1], Vec3{0.0f, 0.0f, 0.0f});
                    if (Dot(axis, axis) == 0.0f)
                        continue;
                    Vec3 elbow = RejectFrom(p[k] - p[k - 1], axis);
                    Vec3 pole = RejectFrom(settings.pole, axis);
                    if (Length(elbow) <= 1e-6f || Length(pole) <= 1e-6f)
                        continue;
                    float angle = SignedAngle(Normalize(elbow), Normalize(pole), axis);
                    p[k] = p[k - 1] + Rotate(FromAxisAngle(axis, angle), p[k] - p[k - 1]);
                }
            }

            Quat parentRotation = rootParentRotation;
            p[0] = root;
            for (std::size_t k = 0; k < segments; ++k)
            {
                const JointLimit& limit = joints[k].limit;
                const Vec3& offset = offsets[k];
                Vec3 animatedDirection = Rotate(animated[k], offset);
                Vec3 direction =
                    Direction(p[k], p[k + 1], Rotate(parentRotation * current[k], offset));
                Vec3 localDirection = Rotate(Conjugate(parentRotation), direction);

                if (settings.useConstraints && limit.type == JointLimitType::Cone)
                {
                    Vec3 center = limit.followAnimation ? animatedDirection : Normalize(limit.axis);
                    localDirection = ClampToCone(localDirection, center, limit.maxAngle);
                }
                else if (settings.useConstraints && isHinge[k])
                {
                    Vec3 planar = RejectFrom(localDirection, hingeAxis[k]);
                    planar = Length(planar) > 1e-6f ? Normalize(planar) : animatedDirection;
                    float angle = SignedAngle(hingeStraight[k], planar, hingeAxis[k]);
                    angle = std::clamp(angle, limit.minAngle, limit.maxAngle);
                    localDirection = Rotate(FromAxisAngle(hingeAxis[k], angle), hingeStraight[k]);
                }

                Quat desired = Normalize(FromTo(animatedDirection, localDirection) * animated[k]);
                Quat local = joints[k].stiffness < 1.0f
                                 ? Slerp(current[k], desired, std::max(joints[k].stiffness, 0.0f))
                                 : desired;
                Quat world = Normalize(parentRotation * local);

                if (settings.useConstraints && k + 1 < segments && isHinge[k + 1] &&
                    limit.maxTwist > 0.0f)
                {
                    Vec3 boneAxis = Normalize(Rotate(world, offset));
                    Vec3 elbow = p[k] + boneAxis * lengths[k];
                    Vec3 next = Direction(elbow, p[k + 2], Vec3{0.0f, 0.0f, 0.0f});
                    Vec3 normal = Cross(boneAxis, next);
                    if (Length(normal) > 1e-4f)
                    {
                        Vec3 hinge = RejectFrom(Rotate(world, hingeAxis[k + 1]), boneAxis);
                        Vec3 wanted = RejectFrom(Normalize(normal), boneAxis);
                        if (Length(hinge) > 1e-6f && Length(wanted) > 1e-6f)
                        {
                            float twist =
                                SignedAngle(Normalize(hinge), Normalize(wanted), boneAxis);
                            twist = std::clamp(twist, -limit.maxTwist, limit.maxTwist);
                            world = Normalize(FromAxisAngle(boneAxis, twist) * world);
                            local = Normalize(Conjugate(parentRotation) * world);
                        }
                    }
                }

                current[k] = local;
                p[k + 1] = p[k] + Rotate(world, offset) * lengths[k];
                parentRotation = world;
            }

            result.error = Length(p[segments] - target);
            ++result.iterations;
        }

        for (std::size_t k = 0; k < segments; ++k)
            localPose[joints[k].bone].q = current[k];
        UpdateSubtree(skeleton, joints[0].bone, localPose, worldPose);
        result.error = Length(worldPose[joints[segments].bone].v - target);
        result.converged = result.error <= settings.tolerance;
        return result;
    }

    ChainSolveResult SolvePrioritized(const Skeleton& skeleton,
                                      const std::vector<std::vector<IKJoint>>& tiers,
                                      const Vec3& target,
                                      const ChainSolveSettings& settings,
                                      std::vector<VQS>& localPose,
                                      std::vector<VQS>& worldPose)
    {
        if (tiers.empty())
            throw std::invalid_argument("Prioritized IK needs at least one tier.");

        ChainSolveResult best;
        std::vector<VQS> bestLocal;
        std::vector<VQS> bestWorld;
        bool haveBest = false;
        for (std::size_t t = 0; t < tiers.size(); ++t)
        {
            std::vector<VQS> local = localPose;
            std::vector<VQS> world = worldPose;
            ChainSolveResult result =
                SolveChain(skeleton, tiers[t], target, settings, local, world);
            result.tier = static_cast<int>(t) + 1;
            if (!haveBest || result.error < best.error)
            {
                best = result;
                bestLocal = std::move(local);
                bestWorld = std::move(world);
                haveBest = true;
            }
            if (result.converged)
                break;
        }
        localPose = std::move(bestLocal);
        worldPose = std::move(bestWorld);
        return best;
    }
}  // namespace Anim
