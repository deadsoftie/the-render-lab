#pragma once

#include <string>
#include <vector>

#include "anim/Math/Vec3.h"
#include "anim/Motion/CubicPath.h"

// Decides how the character gets within arm's reach of a target: stay, turn on the spot, or walk
// an open Hermite path to a stand point. All positions are world space, Y up, meters; only the
// XZ plane is used for walking and facing.
namespace Anim
{
    struct ApproachSettings
    {
        float reachMargin = 0.9f;        // fraction of the arm's length counted as in reach;
                                         // leaves room for the idle's hip sway
        float maxFacingDegrees = 60.0f;  // widest angle off the facing direction still in reach
        float standoff = 0.45f;          // preferred shoulder-to-target distance on the floor
        float minStandoff = 0.15f;       // keeps the body off the target for high or low targets
        float minWalkMeters = 0.5f;      // shorter approaches turn on the spot instead
        float handleFraction = 0.35f;    // inner control points, as a fraction of the distance
        float maxHandleMeters = 1.0f;
    };

    struct ApproachRequest
    {
        Vec3 position;  // character root on the floor
        Vec3 forward{0, 0, 1};
        Vec3 shoulder;  // root of the reaching arm, from the animated pose
        Vec3 target;
        float reach = 0.0f;  // arm length from shoulder to effector
    };

    enum class ApproachKind
    {
        None,  // already in reach (or as close as standing still can get)
        Turn,  // stand point too close to walk to; face the target instead
        Walk
    };

    struct ApproachPlan
    {
        ApproachKind kind = ApproachKind::None;
        Vec3 standPoint;  // root position that puts the shoulder straight behind the target
        Vec3 facing{0, 0, 1};
        float walkDistance = 0.0f;
        std::vector<Vec3> controlPoints;  // Walk only: start, two handles, stand point
        OpenPathEnds ends;
    };

    // True when the target is within reachMargin of the arm and within maxFacingDegrees of the
    // facing direction, both measured from the shoulder.
    bool InReach(const ApproachRequest& request, const ApproachSettings& settings);
    // Invalid requests or settings throw invalid_argument.
    ApproachPlan PlanApproach(const ApproachRequest& request, const ApproachSettings& settings);
    // Builds the open path for a Walk plan; any other plan kind fails with an error.
    bool BuildApproachPath(const ApproachPlan& plan, CubicPath& path, std::string* error = nullptr);
    // Smoothstep-eased yaw from one heading to another along the shorter way round; t is
    // clamped to [0, 1].
    Vec3 TurnTowards(const Vec3& from, const Vec3& to, float t);
}  // namespace Anim
