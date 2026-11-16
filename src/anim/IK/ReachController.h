#pragma once

#include "anim/Math/Vec3.h"
#include "anim/Motion/ApproachPlanner.h"

namespace Anim
{
    enum class ReachState
    {
        Idle,
        Turning,
        Walking,
        Settling,
        Reaching,
        Holding,
        Releasing
    };

    enum class ReachEvent
    {
        None,
        StartWalk,
        StartTurn,
        Settle
    };

    struct ReachSettings
    {
        float reachSeconds = 0.8f;
        float releaseSeconds = 0.4f;
        float settleSeconds = 0.5f;
        float turnSeconds = 0.6f;
        float liftMeters = 0.12f;
        float followRate = 12.0f;
        float replanDistance = 0.3f;
        float holdFacingSlackDegrees = 15.0f;
    };

    struct ReachInput
    {
        bool hasTarget = false;
        ApproachRequest approach;
        Vec3 hand;
        bool walkFinished = false;
    };

    struct ReachOutput
    {
        ReachEvent event = ReachEvent::None;
        float weight = 0.0f;
        Vec3 goal;
        Vec3 facing{0, 0, 1};
    };

    const char* ToString(ReachState state);

    float Smoothstep(float t);
    Vec3 ReachArc(const Vec3& start, const Vec3& end, float lift, float s);

    class ReachController
    {
       public:
        void SetSettings(const ReachSettings& settings, const ApproachSettings& approach);
        const ReachSettings& GetSettings() const { return m_settings; }
        const ApproachSettings& GetApproachSettings() const { return m_approach; }

        void Reset();
        ReachOutput Update(float deltaSeconds, const ReachInput& input);

        ReachState GetState() const { return m_state; }
        const ApproachPlan& GetPlan() const { return m_plan; }
        float GetWeight() const { return m_weight; }
        const Vec3& GetGoal() const { return m_goal; }
        const Vec3& GetFacing() const { return m_facing; }

       private:
        void Begin(const ReachInput& input, ReachOutput& output);
        void StartReach(const ReachInput& input);
        void StartRelease();
        bool ShouldRelease(const ReachInput& input) const;
        void FollowTarget(const Vec3& target, float deltaSeconds);

        ReachSettings m_settings;
        ApproachSettings m_approach;
        ReachState m_state = ReachState::Idle;
        ApproachPlan m_plan;
        float m_clock = 0.0f;
        float m_weight = 0.0f;
        float m_releaseFrom = 0.0f;
        Vec3 m_handStart;
        Vec3 m_goal;
        Vec3 m_goalVelocity;
        Vec3 m_reachTarget;
        Vec3 m_planTarget;
        Vec3 m_turnFrom{0, 0, 1};
        Vec3 m_facing{0, 0, 1};
    };
}  // namespace Anim
