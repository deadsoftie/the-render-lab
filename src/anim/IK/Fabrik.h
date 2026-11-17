#pragma once
#include <cstddef>
#include <limits>
#include <vector>

#include "anim/Math/VQS.h"
#include "anim/Skeleton.h"

// FABRIK (Aristidou and Lasenby, 2011) written from scratch. "World" here means the skeleton's
// model space, the space of Animator::worldPose; callers move targets into it first.
namespace Anim
{
    struct FabrikSettings
    {
        float tolerance = 1e-3f;
        int maxIterations = 32;
    };

    struct FabrikResult
    {
        bool reachable = false;  // target within the chain's total length from the root
        bool converged = false;  // final effector error within tolerance
        int iterations = 0;
        float error = 0.0f;  // effector-to-target distance
    };

    // Reference solver: the textbook, unconstrained, position-only FABRIK on a joint list (root
    // first), with BuildChain/GatherChainPositions/ApplyChainPositions to run it on a skeleton.
    // The app uses SolveChain below; this one is kept as the plain algorithm, with its own tests.
    // The root stays fixed and segment lengths are preserved; an out-of-reach target straightens
    // the chain towards it.
    FabrikResult SolveFabrik(std::vector<Vec3>& joints,
                             const Vec3& target,
                             const FabrikSettings& settings = {});

    // Bone indices from rootBone down to effectorBone, or empty if root is not an ancestor.
    std::vector<int> BuildChain(const Skeleton& skeleton, int rootBone, int effectorBone);

    std::vector<Vec3> GatherChainPositions(const std::vector<VQS>& worldPose,
                                           const std::vector<int>& chain);

    // Converts solved joint positions back into bone rotations: each bone is turned by the
    // shortest arc from its current child direction to the solved one, root first, and its
    // subtree is updated before the next bone is placed.
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

    // Limits are checked on the bone's direction (towards its child) in its parent's frame.
    struct JointLimit
    {
        JointLimitType type = JointLimitType::None;
        // Cone: center direction in the parent frame. Hinge: fallback axis in the parent frame,
        // used only when the animated pose is too straight to give a bend axis.
        Vec3 axis{0.0f, 1.0f, 0.0f};
        bool followAnimation = false;  // cone centered on the animated direction instead of axis
        float minAngle = 0.0f;         // hinge only, radians from straight
        float maxAngle = 3.14159265f;  // cone half-angle or hinge maximum, radians
        float maxTwist = 0.0f;  // roll about the bone allowed to line up a child hinge, radians
    };

    struct IKJoint
    {
        int bone = -1;
        JointLimit limit;
        // Fraction of each pass's rotation this joint takes; below 1 it moves gradually, so
        // looser joints further down the chain do most of the work.
        float stiffness = 1.0f;
    };

    struct ChainSolveSettings
    {
        float tolerance = 1e-3f;
        int maxIterations = 32;
        bool useConstraints = true;
        bool usePole = true;
        Vec3 pole{0.0f, -1.0f, 0.0f};  // model-space direction the elbow bends towards
    };

    struct ChainSolveResult
    {
        bool reachable = false;
        bool converged = false;
        int iterations = 0;
        float error = 0.0f;
        int tier = 0;  // SolvePrioritized: 1-based tier whose result was kept
    };

    // Constrained FABRIK on a skeleton chain (root joint first, effector last, each joint the
    // child of the one before). Reads the animated pose from localPose/worldPose and writes the
    // solved local rotations back, updating every bone below the chain root. Only rotations
    // change, so bone lengths are kept. Invalid chains or inputs throw invalid_argument.
    ChainSolveResult SolveChain(const Skeleton& skeleton,
                                const std::vector<IKJoint>& joints,
                                const Vec3& target,
                                const ChainSolveSettings& settings,
                                std::vector<VQS>& localPose,
                                std::vector<VQS>& worldPose);

    inline constexpr std::size_t kAllTiers = std::numeric_limits<std::size_t>::max();

    // Tries each tier in order from the original pose, stopping at the first that converges, and
    // keeps the result with the lowest error. Tier 1 is the arm alone; later tiers add the spine
    // so it only bends when the arm cannot reach on its own. tierLimit caps how many tiers are
    // tried (e.g. 1 to keep the spine still); zero tiers throw invalid_argument.
    ChainSolveResult SolvePrioritized(const Skeleton& skeleton,
                                      const std::vector<std::vector<IKJoint>>& tiers,
                                      const Vec3& target,
                                      const ChainSolveSettings& settings,
                                      std::vector<VQS>& localPose,
                                      std::vector<VQS>& worldPose,
                                      std::size_t tierLimit = kAllTiers);

    // Hinge frame of bone in its parent's frame: axis is the animated bend axis (or fallbackAxis
    // when the pose is within 3 degrees of straight) and straight is the bone's own direction
    // flattened onto the hinge plane, i.e. zero flexion. Returns whether the axis came from the
    // animation.
    bool HingeFrame(const std::vector<VQS>& localPose,
                    int bone,
                    int child,
                    const Vec3& fallbackAxis,
                    Vec3& straight,
                    Vec3& axis);
}  // namespace Anim
