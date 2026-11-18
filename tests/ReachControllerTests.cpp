#include "MotionTests.h"
#include "TestFramework.h"
#include "anim/IK/ReachController.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <string>

namespace
{
    using namespace Anim;

    constexpr float kStep = 1.0f / 60.0f;

    bool Near(const Vec3& a, const Vec3& b, float epsilon = 1e-4f)
    {
        return std::abs(a.x - b.x) <= epsilon && std::abs(a.y - b.y) <= epsilon &&
               std::abs(a.z - b.z) <= epsilon;
    }

    template <typename Exception, typename Function>
    bool Throws(Function function)
    {
        try
        {
            function();
        }
        catch (const Exception&)
        {
            return true;
        }
        catch (...)
        {
            return false;
        }
        return false;
    }

    // A stand-in for the renderer: a character whose shoulder and hand follow its root, that
    // reaches the stand point after walkSeconds and applies the controller's facing while turning.
    struct Sim
    {
        ReachController controller;
        Vec3 position{0, 0, 0};
        Vec3 forward{0, 0, 1};
        Vec3 target;
        bool hasTarget = true;
        bool walking = false;
        float walkClock = 0.0f;
        float walkSeconds = 0.5f;
        int walkStarts = 0;
        int turnStarts = 0;
        int settles = 0;
        ReachOutput last;

        Vec3 Shoulder() const
        {
            Vec3 right{-forward.z, 0, forward.x};
            return position + right * 0.2f + forward * 0.05f + Vec3{0, 1.5f, 0};
        }

        Vec3 Hand() const { return Shoulder() + Vec3{0, -0.6f, 0}; }

        ReachOutput Step(float dt = kStep)
        {
            bool walkFinished = false;
            if (walking)
            {
                walkClock += dt;
                if (walkClock >= walkSeconds)
                {
                    position = controller.GetPlan().standPoint;
                    forward = controller.GetPlan().facing;
                    walking = false;
                    walkFinished = true;
                }
            }

            ReachInput input;
            input.hasTarget = hasTarget;
            input.approach.position = position;
            input.approach.forward = forward;
            input.approach.shoulder = Shoulder();
            input.approach.target = target;
            input.approach.reach = 0.9f;
            input.hand = Hand();
            input.walkFinished = walkFinished;

            bool wasTurning = controller.GetState() == ReachState::Turning;
            last = controller.Update(dt, input);
            if (wasTurning || controller.GetState() == ReachState::Turning)
                forward = last.facing;
            if (last.event == ReachEvent::StartWalk)
            {
                walking = true;
                walkClock = 0.0f;
                ++walkStarts;
            }
            if (last.event == ReachEvent::StartTurn)
                ++turnStarts;
            if (last.event == ReachEvent::Settle)
            {
                walking = false;
                ++settles;
            }
            return last;
        }

        bool RunUntil(ReachState state, int maxFrames = 600)
        {
            for (int i = 0; i < maxFrames; ++i)
            {
                if (controller.GetState() == state)
                    return true;
                Step();
            }
            return controller.GetState() == state;
        }
    };

    // Reach arc end points and lift, smoothstep clamping, state names.
    void CheckArc()
    {
        const Vec3 start{0, 1, 0};
        const Vec3 end{1, 1, 1};
        CHECK(Near(ReachArc(start, end, 0.2f, 0.0f), start));
        CHECK(Near(ReachArc(start, end, 0.2f, 1.0f), end));
        CHECK(Near(ReachArc(start, end, 0.2f, 0.5f), Vec3{0.5f, 1.1f, 0.5f}));
        CHECK(Near(ReachArc(start, end, 0.2f, 2.0f), end));
        CHECK(Smoothstep(-1.0f) == 0.0f);
        CHECK(Smoothstep(0.5f) == 0.5f);
        CHECK(Smoothstep(3.0f) == 1.0f);
        CHECK(std::string(ToString(ReachState::Holding)) == "Holding");
    }

    // Near target: the reach weight rises from 0 to 1 without ever dropping, on time; holding
    // then follows a small move with no overshoot, and pausing changes nothing.
    void CheckNearReach()
    {
        Sim sim;
        sim.target = {-0.2f, 1.4f, 0.55f};
        ReachOutput first = sim.Step();
        CHECK(sim.controller.GetState() == ReachState::Reaching);
        CHECK(first.event == ReachEvent::None);
        CHECK(first.weight == 0.0f);
        CHECK(Near(first.goal, sim.Hand()));

        float previous = 0.0f;
        int frames = 0;
        while (sim.controller.GetState() == ReachState::Reaching && frames < 200)
        {
            ReachOutput out = sim.Step();
            CHECK(out.weight >= previous);
            CHECK(out.weight <= 1.0f);
            if (sim.controller.GetState() == ReachState::Reaching)
                CHECK(out.goal.y >= std::min(sim.Hand().y, sim.target.y) - 1e-4f);
            previous = out.weight;
            ++frames;
        }
        CHECK(sim.controller.GetState() == ReachState::Holding);
        CHECK(std::abs(frames * kStep - sim.controller.GetSettings().reachSeconds) <=
              kStep + 1e-4f);
        CHECK(sim.last.weight == 1.0f);
        CHECK(Near(sim.last.goal, sim.target));
        CHECK(sim.walkStarts == 0);

        ReachOutput paused = sim.Step(0.0f);
        CHECK(paused.weight == 1.0f);
        CHECK(sim.controller.GetState() == ReachState::Holding);

        const Vec3 moved = sim.target + Vec3{0.1f, 0.05f, 0};
        sim.target = moved;
        float lastDistance = Length(sim.last.goal - moved);
        for (int i = 0; i < 60; ++i)
        {
            ReachOutput out = sim.Step();
            CHECK(sim.controller.GetState() == ReachState::Holding);
            CHECK(Length(out.goal - moved) <= lastDistance + 1e-6f);
            CHECK(out.goal.x <= moved.x + 1e-6f);
            lastDistance = Length(out.goal - moved);
        }
        CHECK(lastDistance < 1e-3f);
    }

    // Far target: walk, settle, reach and hold, in that order.
    void CheckFarWalk()
    {
        Sim sim;
        sim.target = {3.0f, 1.2f, 4.0f};
        ReachOutput first = sim.Step();
        CHECK(first.event == ReachEvent::StartWalk);
        CHECK(sim.controller.GetState() == ReachState::Walking);
        CHECK(sim.controller.GetPlan().kind == ApproachKind::Walk);
        CHECK(first.weight == 0.0f);

        CHECK(sim.RunUntil(ReachState::Settling));
        CHECK(sim.settles == 1);
        CHECK(Near(sim.position, sim.controller.GetPlan().standPoint));

        CHECK(sim.RunUntil(ReachState::Reaching));
        CHECK(sim.walkStarts == 1);
        CHECK(sim.RunUntil(ReachState::Holding));
        CHECK(Near(sim.last.goal, sim.target));
    }

    // Mid-walk: small moves are ignored, big ones re-plan, a now-near target stops the walk.
    void CheckReplanWhileWalking()
    {
        Sim sim;
        sim.walkSeconds = 2.0f;
        sim.target = {3.0f, 1.2f, 4.0f};
        sim.Step();
        Vec3 firstStand = sim.controller.GetPlan().standPoint;

        sim.target = sim.target + Vec3{0.2f, 0, 0};
        sim.Step();
        CHECK(sim.walkStarts == 1);

        sim.target = {-3.0f, 1.2f, 4.0f};
        ReachOutput replanned = sim.Step();
        CHECK(replanned.event == ReachEvent::StartWalk);
        CHECK(sim.walkStarts == 2);
        CHECK(sim.controller.GetState() == ReachState::Walking);
        CHECK(!Near(sim.controller.GetPlan().standPoint, firstStand, 0.5f));

        sim.target = {-0.2f, 1.4f, 0.55f};
        ReachOutput stop = sim.Step();
        CHECK(stop.event == ReachEvent::Settle);
        CHECK(sim.controller.GetState() == ReachState::Settling);
    }

    // Target moved out of reach: the weight fades to 0, then the character walks again.
    void CheckMovedAway()
    {
        Sim sim;
        sim.target = {-0.2f, 1.4f, 0.55f};
        CHECK(sim.RunUntil(ReachState::Holding));

        sim.target = {3.0f, 1.2f, 4.0f};
        sim.Step();
        CHECK(sim.controller.GetState() == ReachState::Releasing);
        float previous = 1.0f;
        int frames = 0;
        while (sim.controller.GetState() == ReachState::Releasing && frames < 200)
        {
            CHECK(sim.last.weight <= previous);
            previous = sim.last.weight;
            sim.Step();
            ++frames;
        }
        CHECK(sim.controller.GetState() == ReachState::Walking);
        CHECK(sim.last.event == ReachEvent::StartWalk);
        CHECK(sim.last.weight == 0.0f);
        CHECK(sim.RunUntil(ReachState::Holding));
    }

    // Target removed: release to idle with the goal held, and start again when it returns. Removed
    // mid-walk, the walk stops at once and settles to idle.
    void CheckRemoved()
    {
        Sim sim;
        sim.target = {-0.2f, 1.4f, 0.55f};
        CHECK(sim.RunUntil(ReachState::Holding));
        Vec3 heldGoal = sim.last.goal;

        sim.hasTarget = false;
        sim.Step();
        CHECK(sim.controller.GetState() == ReachState::Releasing);
        CHECK(sim.RunUntil(ReachState::Idle));
        CHECK(sim.last.weight == 0.0f);
        CHECK(Near(sim.last.goal, heldGoal));
        for (int i = 0; i < 30; ++i)
            sim.Step();
        CHECK(sim.controller.GetState() == ReachState::Idle);

        sim.hasTarget = true;
        sim.Step();
        CHECK(sim.controller.GetState() == ReachState::Reaching);

        sim.hasTarget = false;
        sim.Step();
        CHECK(sim.controller.GetState() == ReachState::Releasing);

        Sim walker;
        walker.walkSeconds = 5.0f;
        walker.target = {3.0f, 1.2f, 4.0f};
        walker.Step();
        CHECK(walker.controller.GetState() == ReachState::Walking);
        walker.Step();
        walker.hasTarget = false;
        ReachOutput stop = walker.Step();
        CHECK(stop.event == ReachEvent::Settle);
        CHECK(walker.controller.GetState() == ReachState::Settling);
        CHECK(!walker.walking);
        CHECK(walker.RunUntil(ReachState::Idle, 120));
        CHECK(walker.walkStarts == 1);
    }

    // Side target: turn on the spot with steadily increasing yaw, then reach without walking. A
    // target moved far or removed during the turn is planned for again when the turn ends.
    void CheckTurn()
    {
        Sim sim;
        sim.target = {0.6f, 1.4f, 0.2f};
        ReachOutput first = sim.Step();
        CHECK(first.event == ReachEvent::StartTurn);
        CHECK(sim.controller.GetState() == ReachState::Turning);
        CHECK(Near(sim.forward, Vec3{0, 0, 1}));

        float previousYaw = 0.0f;
        while (sim.controller.GetState() == ReachState::Turning)
        {
            sim.Step();
            float yaw = std::atan2(sim.forward.x, sim.forward.z);
            CHECK(yaw >= previousYaw - 1e-6f);
            previousYaw = yaw;
        }
        CHECK(sim.controller.GetState() == ReachState::Reaching);
        CHECK(Near(sim.forward, sim.controller.GetPlan().facing, 1e-3f));
        CHECK(Near(sim.position, Vec3{0, 0, 0}));
        CHECK(sim.walkStarts == 0);
        CHECK(sim.turnStarts == 1);
        CHECK(sim.RunUntil(ReachState::Holding));

        Sim moved;
        moved.target = {0.6f, 1.4f, 0.2f};
        moved.Step();
        CHECK(moved.controller.GetState() == ReachState::Turning);
        moved.Step();
        moved.target = {3.0f, 1.2f, 4.0f};
        while (moved.controller.GetState() == ReachState::Turning)
            moved.Step();
        CHECK(moved.controller.GetState() == ReachState::Walking);
        CHECK(moved.last.event == ReachEvent::StartWalk);
        CHECK(moved.last.weight == 0.0f);

        Sim removed;
        removed.target = {0.6f, 1.4f, 0.2f};
        removed.Step();
        removed.hasTarget = false;
        while (removed.controller.GetState() == ReachState::Turning)
            removed.Step();
        CHECK(removed.controller.GetState() == ReachState::Idle);
        CHECK(removed.last.weight == 0.0f);
    }

    // Bad settings are rejected and kept out, bad times throw, and Reset keeps the settings.
    void CheckInvalid()
    {
        ReachController controller;
        ReachSettings settings;
        settings.reachSeconds = 0.0f;
        CHECK(Throws<std::invalid_argument>([&] { controller.SetSettings(settings, {}); }));
        settings = {};
        settings.followRate = -1.0f;
        CHECK(Throws<std::invalid_argument>([&] { controller.SetSettings(settings, {}); }));
        ApproachSettings approach;
        approach.reachMargin = 0.0f;
        CHECK(Throws<std::invalid_argument>([&] { controller.SetSettings({}, approach); }));
        CHECK(controller.GetSettings().reachSeconds == ReachSettings{}.reachSeconds);

        ReachInput input;
        CHECK(Throws<std::invalid_argument>([&] { controller.Update(-1.0f, input); }));
        CHECK(Throws<std::invalid_argument>(
            [&] { controller.Update(std::numeric_limits<float>::quiet_NaN(), input); }));
        CHECK(controller.Update(kStep, input).event == ReachEvent::None);
        CHECK(controller.GetState() == ReachState::Idle);

        settings = {};
        settings.reachSeconds = 0.5f;
        controller.SetSettings(settings, {});
        controller.Reset();
        CHECK(controller.GetSettings().reachSeconds == 0.5f);
        CHECK(controller.GetState() == ReachState::Idle);
    }
}  // namespace

void RunReachControllerTests()
{
    CheckArc();
    CheckNearReach();
    CheckFarWalk();
    CheckReplanWhileWalking();
    CheckMovedAway();
    CheckRemoved();
    CheckTurn();
    CheckInvalid();
}
