#pragma once

#include <string>
#include <vector>

#include "anim/Math/Vec3.h"
#include "anim/Motion/CubicPath.h"

namespace Anim
{
    struct ApproachSettings
    {
        float reachMargin = 0.9f;
        float maxFacingDegrees = 60.0f;
        float standoff = 0.45f;
        float minStandoff = 0.15f;
        float minWalkMeters = 0.5f;
        float handleFraction = 0.35f;
        float maxHandleMeters = 1.0f;
    };

    struct ApproachRequest
    {
        Vec3 position;
        Vec3 forward{0, 0, 1};
        Vec3 shoulder;
        Vec3 target;
        float reach = 0.0f;
    };

    enum class ApproachKind
    {
        None,
        Turn,
        Walk
    };

    struct ApproachPlan
    {
        ApproachKind kind = ApproachKind::None;
        Vec3 standPoint;
        Vec3 facing{0, 0, 1};
        float walkDistance = 0.0f;
        std::vector<Vec3> controlPoints;
        OpenPathEnds ends;
    };

    bool InReach(const ApproachRequest& request, const ApproachSettings& settings);
    ApproachPlan PlanApproach(const ApproachRequest& request, const ApproachSettings& settings);
    bool BuildApproachPath(const ApproachPlan& plan, CubicPath& path, std::string* error = nullptr);
    Vec3 TurnTowards(const Vec3& from, const Vec3& to, float t);
}  // namespace Anim
