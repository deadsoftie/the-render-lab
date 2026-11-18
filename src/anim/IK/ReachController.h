#pragma once

#include "anim/Math/Easing.h"
#include "anim/Math/Vec3.h"
#include "anim/Motion/ApproachPlanner.h"

// State machine for "walk to the object and pick it up". It is pure logic: each frame the caller
// passes the character's state in and acts on the event that comes back (start a walk, start a
// turn, blend to idle), then applies IK towards the returned goal with the returned weight.
namespace Anim
{
    // Idle -> (Turning | Walking -> Settling) -> Reaching -> Holding -> Releasing -> back to the
    // start, which plans again from wherever the character now is.
    enum class ReachState
    {
        Idle,       // no target, no IK
        Turning,    // turning on the spot to face a nearby target
        Walking,    // following the approach path
        Settling,   // walk finished, blending into the idle clip
        Reaching,   // IK weight rising, hand travelling to the target
        Holding,    // full IK, hand follows the target
        Releasing   // IK weight fading back to the animation
    };

    enum class ReachEvent
    {
        None,
        StartWalk,  // build a path from GetPlan() and walk it (also sent to re-plan mid-walk)
        StartTurn,  // stand still; the facing to use comes in ReachOutput::facing
        Settle      // stop walking and blend to idle
    };

    struct ReachSettings
    {
        float reachSeconds = 0.8f;
        float releaseSeconds = 0.4f;
        float settleSeconds = 0.5f;  // matches the Walk to Idle blend
        float turnSeconds = 0.6f;
        float liftMeters = 0.12f;    // how high the hand's path arcs above a straight line
        float followRate = 12.0f;    // spring rate (1/s) while holding a moving target
        float replanDistance = 0.3f; // target movement that counts as "moved"
        float holdFacingSlackDegrees = 15.0f;  // extra facing angle allowed before releasing
    };

    struct ReachInput
    {
        bool hasTarget = false;
        ApproachRequest approach;  // current root, facing, shoulder, target and arm length
        Vec3 hand;                 // animated (pre-IK) effector position
        bool walkFinished = false;
    };

    struct ReachOutput
    {
        ReachEvent event = ReachEvent::None;
        float weight = 0.0f;   // blend from the animated pose (0) to the IK pose (1)
        Vec3 goal;             // where the IK should put the hand this frame
        Vec3 facing{0, 0, 1};  // character heading while Turning
    };

    const char* ToString(ReachState state);

    // Quadratic Bezier from start to end whose middle control point is lifted by lift, so the
    // hand rises over the straight line instead of cutting through the body.
    Vec3 ReachArc(const Vec3& start, const Vec3& end, float lift, float s);

    class ReachController
    {
       public:
        // Throws invalid_argument and keeps the old settings if either set is invalid.
        void SetSettings(const ReachSettings& settings, const ApproachSettings& approach);
        const ReachSettings& GetSettings() const { return m_settings; }
        const ApproachSettings& GetApproachSettings() const { return m_approach; }

        // Back to Idle with no IK; settings are kept.
        void Reset();
        // Advances the state machine; a deltaSeconds of 0 (paused) changes nothing time-based.
        ReachOutput Update(float deltaSeconds, const ReachInput& input);

        ReachState GetState() const { return m_state; }
        const ApproachPlan& GetPlan() const { return m_plan; }
        float GetWeight() const { return m_weight; }
        const Vec3& GetGoal() const { return m_goal; }
        const Vec3& GetFacing() const { return m_facing; }

       private:
        // Plans from the current input and enters Walking, Turning, Reaching or Idle.
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
        float m_releaseFrom = 0.0f;  // weight when Releasing began, faded from there
        Vec3 m_handStart;
        Vec3 m_goal;
        Vec3 m_goalVelocity;
        Vec3 m_reachTarget;  // target when the reach began, for the "moved away" test
        Vec3 m_planTarget;   // target when the walk was planned, for re-planning
        Vec3 m_turnFrom{0, 0, 1};
        Vec3 m_facing{0, 0, 1};
    };
}  // namespace Anim
