#include "anim/IK/Fabrik.h"

#include <algorithm>
#include <cmath>
#include <stdexcept>

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
}  // namespace Anim
