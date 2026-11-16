#pragma once
#include <vector>

#include "anim/Math/VQS.h"
#include "anim/Skeleton.h"

namespace Anim
{
    struct FabrikSettings
    {
        float tolerance = 1e-3f;
        int maxIterations = 32;
    };

    struct FabrikResult
    {
        bool reachable = false;
        bool converged = false;
        int iterations = 0;
        float error = 0.0f;
    };

    FabrikResult SolveFabrik(std::vector<Vec3>& joints,
                             const Vec3& target,
                             const FabrikSettings& settings = {});

    std::vector<int> BuildChain(const Skeleton& skeleton, int rootBone, int effectorBone);

    std::vector<Vec3> GatherChainPositions(const std::vector<VQS>& worldPose,
                                           const std::vector<int>& chain);

    void ApplyChainPositions(const Skeleton& skeleton,
                             const std::vector<int>& chain,
                             const std::vector<Vec3>& positions,
                             std::vector<VQS>& localPose,
                             std::vector<VQS>& worldPose);

    enum class JointLimitType
    {
        None,
        Cone,
        Hinge
    };

    struct JointLimit
    {
        JointLimitType type = JointLimitType::None;
        Vec3 axis{0.0f, 1.0f, 0.0f};
        bool followAnimation = false;
        float minAngle = 0.0f;
        float maxAngle = 3.14159265f;
        float maxTwist = 0.0f;
    };

    struct IKJoint
    {
        int bone = -1;
        JointLimit limit;
        float stiffness = 1.0f;
    };

    struct ChainSolveSettings
    {
        float tolerance = 1e-3f;
        int maxIterations = 32;
        bool useConstraints = true;
        bool usePole = true;
        Vec3 pole{0.0f, -1.0f, 0.0f};
    };

    struct ChainSolveResult
    {
        bool reachable = false;
        bool converged = false;
        int iterations = 0;
        float error = 0.0f;
        int tier = 0;
    };

    ChainSolveResult SolveChain(const Skeleton& skeleton,
                                const std::vector<IKJoint>& joints,
                                const Vec3& target,
                                const ChainSolveSettings& settings,
                                std::vector<VQS>& localPose,
                                std::vector<VQS>& worldPose);

    ChainSolveResult SolvePrioritized(const Skeleton& skeleton,
                                      const std::vector<std::vector<IKJoint>>& tiers,
                                      const Vec3& target,
                                      const ChainSolveSettings& settings,
                                      std::vector<VQS>& localPose,
                                      std::vector<VQS>& worldPose);

    bool HingeFrame(const std::vector<VQS>& localPose,
                    int bone,
                    int child,
                    const Vec3& fallbackAxis,
                    Vec3& straight,
                    Vec3& axis);
}  // namespace Anim
