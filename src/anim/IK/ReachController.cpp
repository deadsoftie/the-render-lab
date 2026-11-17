#include "anim/IK/ReachController.h"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace Anim
{
    const char* ToString(ReachState state)
    {
        switch (state)
        {
            case ReachState::Idle:
                return "Idle";
            case ReachState::Turning:
                return "Turning";
            case ReachState::Walking:
                return "Walking";
            case ReachState::Settling:
                return "Settling";
            case ReachState::Reaching:
                return "Reaching";
            case ReachState::Holding:
                return "Holding";
            case ReachState::Releasing:
                return "Releasing";
        }
        return "Unknown";
    }

    Vec3 ReachArc(const Vec3& start, const Vec3& end, float lift, float s)
    {
        s = std::clamp(s, 0.0f, 1.0f);
        Vec3 control = (start + end) * 0.5f + Vec3{0.0f, lift, 0.0f};
        float u = 1.0f - s;
        return start * (u * u) + control * (2.0f * u * s) + end * (s * s);
    }

    void ReachController::SetSettings(const ReachSettings& settings,
                                      const ApproachSettings& approach)
    {
        auto positive = [](float v) { return std::isfinite(v) && v > 0.0f; };
        auto nonnegative = [](float v) { return std::isfinite(v) && v >= 0.0f; };
        if (!positive(settings.reachSeconds) || !positive(settings.releaseSeconds) ||
            !positive(settings.turnSeconds) || !nonnegative(settings.settleSeconds))
            throw std::invalid_argument("Reach durations must be finite and positive.");
        if (!nonnegative(settings.liftMeters) || !positive(settings.followRate) ||
            !positive(settings.replanDistance))
            throw std::invalid_argument("Reach lift, follow rate and replan distance are invalid.");
        if (!(settings.holdFacingSlackDegrees >= 0.0f && settings.holdFacingSlackDegrees <= 180.0f))
            throw std::invalid_argument("holdFacingSlackDegrees must be in [0, 180].");

        // The planner validates its own settings; a dummy query makes it throw now rather than
        // mid-frame later.
        ApproachRequest probe;
        probe.reach = 1.0f;
        InReach(probe, approach);

        m_settings = settings;
        m_approach = approach;
    }

    void ReachController::Reset()
    {
        ReachSettings settings = m_settings;
        ApproachSettings approach = m_approach;
        *this = ReachController{};
        m_settings = settings;
        m_approach = approach;
    }

    ReachOutput ReachController::Update(float deltaSeconds, const ReachInput& input)
    {
        if (!std::isfinite(deltaSeconds) || deltaSeconds < 0.0f)
            throw std::invalid_argument("Reach update time must be finite and nonnegative.");

        ReachOutput output;
        switch (m_state)
        {
            case ReachState::Idle:
                if (input.hasTarget)
                    Begin(input, output);
                break;

            case ReachState::Turning:
                m_clock += deltaSeconds;
                m_facing = TurnTowards(m_turnFrom, m_plan.facing, m_clock / m_settings.turnSeconds);
                // Plan again rather than reaching straight away: the target may have moved or
                // gone during the turn. The finished heading stands in for the input's, which
                // is still last frame's.
                if (m_clock >= m_settings.turnSeconds)
                {
                    ReachInput turned = input;
                    turned.approach.forward = m_facing;
                    Begin(turned, output);
                }
                break;

            case ReachState::Walking:
                // A target that moves far enough gets a fresh plan from the current spot; if it
                // is now close, the walk stops here and the settle re-plans. Losing the target
                // (hidden, Stop, reach disabled) stops the walk at once.
                if (input.walkFinished || !input.hasTarget)
                {
                    m_state = ReachState::Settling;
                    m_clock = 0.0f;
                    output.event = ReachEvent::Settle;
                }
                else if (input.hasTarget &&
                         Length(input.approach.target - m_planTarget) > m_settings.replanDistance)
                {
                    ApproachPlan plan = PlanApproach(input.approach, m_approach);
                    m_planTarget = input.approach.target;
                    if (plan.kind == ApproachKind::Walk)
                    {
                        m_plan = plan;
                        output.event = ReachEvent::StartWalk;
                    }
                    else
                    {
                        m_state = ReachState::Settling;
                        m_clock = 0.0f;
                        output.event = ReachEvent::Settle;
                    }
                }
                break;

            case ReachState::Settling:
                m_clock += deltaSeconds;
                if (m_clock >= m_settings.settleSeconds)
                    Begin(input, output);
                break;

            case ReachState::Reaching:
                if (ShouldRelease(input))
                {
                    StartRelease();
                    break;
                }
                // The interpolation from the start pose to the end pose: the same eased s drives
                // both the goal along the arc and the blend weight on the joint rotations.
                m_clock += deltaSeconds;
                m_weight = Smoothstep(m_clock / m_settings.reachSeconds);
                m_goal =
                    ReachArc(m_handStart, input.approach.target, m_settings.liftMeters, m_weight);
                if (m_clock >= m_settings.reachSeconds)
                {
                    m_state = ReachState::Holding;
                    m_weight = 1.0f;
                    m_goal = input.approach.target;
                    m_goalVelocity = {};
                }
                break;

            case ReachState::Holding:
                if (ShouldRelease(input))
                {
                    StartRelease();
                    break;
                }
                FollowTarget(input.approach.target, deltaSeconds);
                break;

            case ReachState::Releasing:
                // The goal stays where it was while the weight fades, so the arm returns to the
                // animation smoothly from wherever it was.
                m_clock += deltaSeconds;
                m_weight = m_releaseFrom * (1.0f - Smoothstep(m_clock / m_settings.releaseSeconds));
                if (m_clock >= m_settings.releaseSeconds)
                    Begin(input, output);
                break;
        }

        output.weight = m_weight;
        output.goal = m_goal;
        output.facing = m_facing;
        return output;
    }

    void ReachController::Begin(const ReachInput& input, ReachOutput& output)
    {
        m_clock = 0.0f;
        m_weight = 0.0f;
        if (!input.hasTarget)
        {
            m_state = ReachState::Idle;
            return;
        }

        m_plan = PlanApproach(input.approach, m_approach);
        m_planTarget = input.approach.target;
        switch (m_plan.kind)
        {
            case ApproachKind::Walk:
                m_state = ReachState::Walking;
                output.event = ReachEvent::StartWalk;
                break;
            case ApproachKind::Turn:
                m_state = ReachState::Turning;
                m_turnFrom = input.approach.forward;
                m_facing = TurnTowards(m_turnFrom, m_turnFrom, 0.0f);
                output.event = ReachEvent::StartTurn;
                break;
            case ApproachKind::None:
                StartReach(input);
                break;
        }
    }

    void ReachController::StartReach(const ReachInput& input)
    {
        m_state = ReachState::Reaching;
        m_clock = 0.0f;
        m_weight = 0.0f;
        m_handStart = input.hand;
        m_goal = input.hand;
        m_goalVelocity = {};
        m_reachTarget = input.approach.target;
    }

    void ReachController::StartRelease()
    {
        m_state = ReachState::Releasing;
        m_clock = 0.0f;
        m_releaseFrom = m_weight;
    }

    // Small moves are followed while holding. Releasing needs the target to have moved and to be
    // out of a looser reach test; that hysteresis stops the state flickering at the edge, and the
    // "moved" condition stops a best-effort reach (target too high to ever be in reach) from
    // releasing and re-reaching forever.
    bool ReachController::ShouldRelease(const ReachInput& input) const
    {
        if (!input.hasTarget)
            return true;
        if (Length(input.approach.target - m_reachTarget) <= m_settings.replanDistance)
            return false;

        ApproachSettings hold = m_approach;
        hold.reachMargin = 1.0f;
        hold.maxFacingDegrees =
            std::min(180.0f, m_approach.maxFacingDegrees + m_settings.holdFacingSlackDegrees);
        return !InReach(input.approach, hold);
    }

    // Exact critically damped spring towards a fixed target (closed form, so it is stable at any
    // frame time): smooth follow with no overshoot.
    void ReachController::FollowTarget(const Vec3& target, float deltaSeconds)
    {
        float rate = m_settings.followRate;
        Vec3 offset = m_goal - target;
        Vec3 drive = m_goalVelocity + offset * rate;
        float decay = std::exp(-rate * deltaSeconds);
        m_goal = target + (offset + drive * deltaSeconds) * decay;
        m_goalVelocity = (m_goalVelocity - drive * (rate * deltaSeconds)) * decay;
    }
}  // namespace Anim
