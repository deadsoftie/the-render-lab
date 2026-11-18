#include "anim/Motion/ApproachPlanner.h"

#include "anim/Math/Easing.h"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace Anim
{
    namespace
    {
        constexpr float kPi = 3.14159265f;

        Vec3 Flat(const Vec3& v)
        {
            return {v.x, 0.0f, v.z};
        }

        // Unit direction on the XZ plane, or the fallback when v is (nearly) vertical.
        Vec3 FlatDirection(const Vec3& v, const Vec3& fallback)
        {
            Vec3 flat = Flat(v);
            float length = Length(flat);
            return length > 1e-5f ? flat * (1.0f / length) : fallback;
        }

        // The character's right on the floor, for a Y-up model facing +Z: forward (0, 0, 1)
        // gives (-1, 0, 0), the side the Mixamo right arm is on.
        Vec3 RightOf(const Vec3& forward)
        {
            return {-forward.z, 0.0f, forward.x};
        }

        float AngleBetween(const Vec3& a, const Vec3& b)
        {
            return std::acos(std::clamp(Dot(a, b), -1.0f, 1.0f));
        }

        void Validate(const ApproachRequest& request, const ApproachSettings& settings)
        {
            if (!IsFinite(request.position) || !IsFinite(request.forward) ||
                !IsFinite(request.shoulder) || !IsFinite(request.target))
                throw std::invalid_argument("Approach positions and facing must be finite.");
            if (!std::isfinite(request.reach) || request.reach <= 0.0f)
                throw std::invalid_argument("Approach reach must be finite and positive.");
            if (!(settings.reachMargin > 0.0f && settings.reachMargin <= 1.0f))
                throw std::invalid_argument("reachMargin must be in (0, 1].");
            if (!(settings.maxFacingDegrees >= 0.0f && settings.maxFacingDegrees <= 180.0f))
                throw std::invalid_argument("maxFacingDegrees must be in [0, 180].");
            if (!(settings.minStandoff >= 0.0f && settings.standoff >= settings.minStandoff &&
                  std::isfinite(settings.standoff)))
                throw std::invalid_argument("Standoffs must satisfy 0 <= minStandoff <= standoff.");
            if (!(settings.minWalkMeters >= 0.0f && std::isfinite(settings.minWalkMeters)))
                throw std::invalid_argument("minWalkMeters must be finite and nonnegative.");
            if (!(settings.handleFraction > 0.0f && settings.handleFraction <= 0.5f))
                throw std::invalid_argument("handleFraction must be in (0, 0.5].");
            if (!(settings.maxHandleMeters > 0.0f && std::isfinite(settings.maxHandleMeters)))
                throw std::invalid_argument("maxHandleMeters must be finite and positive.");
        }
    }  // namespace

    bool InReach(const ApproachRequest& request, const ApproachSettings& settings)
    {
        Validate(request, settings);
        Vec3 forward = FlatDirection(request.forward, {0, 0, 1});
        Vec3 toTarget = request.target - request.shoulder;
        if (Length(toTarget) > request.reach * settings.reachMargin)
            return false;
        Vec3 direction = FlatDirection(toTarget, forward);
        return AngleBetween(forward, direction) <= settings.maxFacingDegrees * kPi / 180.0f;
    }

    ApproachPlan PlanApproach(const ApproachRequest& request, const ApproachSettings& settings)
    {
        Validate(request, settings);

        Vec3 forward = FlatDirection(request.forward, {0, 0, 1});
        Vec3 start = Flat(request.position);
        Vec3 target = Flat(request.target);
        Vec3 facing = FlatDirection(target - start, forward);

        // Shoulder offset from the root in the character's own frame, so it can be re-applied
        // after the character turns to face the target.
        Vec3 offset = request.shoulder - request.position;
        float lateral = Dot(offset, RightOf(forward));
        float along = Dot(offset, forward);

        // Pythagoras on the reach sphere: a target well above or below the shoulder leaves less
        // horizontal reach, so the character stands closer.
        float rise = request.target.y - request.shoulder.y;
        float reach = request.reach * settings.reachMargin;
        float horizontal = std::sqrt(std::max(0.0f, reach * reach - rise * rise));
        float standoff = std::clamp(horizontal, settings.minStandoff, settings.standoff);

        ApproachPlan plan;
        plan.facing = facing;
        // Back off from the target along the facing direction, then shift sideways so the
        // shoulder, not the root, lines up with it.
        plan.standPoint = target - facing * (standoff + along) - RightOf(facing) * lateral;
        plan.walkDistance = Length(plan.standPoint - start);

        if (InReach(request, settings))
            return plan;

        // Not in reach but nowhere useful to walk: turn to face it. When already facing it
        // (target too high or low for any stand point), reach as far as possible from here.
        if (plan.walkDistance < settings.minWalkMeters)
        {
            if (AngleBetween(forward, facing) > 1e-3f)
                plan.kind = ApproachKind::Turn;
            return plan;
        }

        // Handles leave along the current heading and arrive along the final facing, so the walk
        // starts and ends without a sudden turn. Short walks fall back to the two end points.
        plan.kind = ApproachKind::Walk;
        float handle =
            std::min(settings.maxHandleMeters, settings.handleFraction * plan.walkDistance);
        Vec3 leave = start + forward * handle;
        Vec3 arrive = plan.standPoint - facing * handle;
        if (Length(arrive - leave) > 1e-3f)
            plan.controlPoints = {start, leave, arrive, plan.standPoint};
        else
            plan.controlPoints = {start, plan.standPoint};
        plan.ends.hasStartDirection = true;
        plan.ends.startDirection = forward;
        plan.ends.hasEndDirection = true;
        plan.ends.endDirection = facing;
        return plan;
    }

    bool BuildApproachPath(const ApproachPlan& plan, CubicPath& path, std::string* error)
    {
        if (plan.kind != ApproachKind::Walk)
        {
            if (error)
                *error = "Only a walking approach has a path.";
            return false;
        }
        return path.SetOpenControlPoints(plan.controlPoints, plan.ends, error);
    }

    Vec3 TurnTowards(const Vec3& from, const Vec3& to, float t)
    {
        if (!IsFinite(from) || !IsFinite(to) || !std::isfinite(t))
            throw std::invalid_argument("Turn directions and time must be finite.");

        Vec3 a = FlatDirection(from, {0, 0, 1});
        Vec3 b = FlatDirection(to, a);
        float s = Smoothstep(t);

        float startYaw = std::atan2(a.x, a.z);
        float delta = std::atan2(b.x, b.z) - startYaw;
        if (delta > kPi)
            delta -= 2.0f * kPi;
        else if (delta < -kPi)
            delta += 2.0f * kPi;

        float yaw = startYaw + delta * s;
        return {std::sin(yaw), 0.0f, std::cos(yaw)};
    }
}  // namespace Anim
