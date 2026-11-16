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
}  // namespace Anim
