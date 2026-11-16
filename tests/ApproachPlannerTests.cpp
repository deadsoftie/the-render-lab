#include "MotionTests.h"
#include "TestFramework.h"
#include "anim/Motion/ApproachPlanner.h"
#include "anim/Motion/PathFollower.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace
{
    using namespace Anim;

    constexpr float kDegrees = 3.14159265f / 180.0f;

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

    Vec3 Flat(const Vec3& v)
    {
        return {v.x, 0.0f, v.z};
    }

    Vec3 RightOf(const Vec3& forward)
    {
        return {-forward.z, 0.0f, forward.x};
    }

    ApproachRequest Request(const Vec3& target)
    {
        ApproachRequest request;
        request.position = {0, 0, 0};
        request.forward = {0, 0, 1};
        request.shoulder = {-0.2f, 1.5f, 0.05f};
        request.target = target;
        request.reach = 0.9f;
        return request;
    }

    ApproachRequest Arrived(const ApproachRequest& request, const ApproachPlan& plan)
    {
        Vec3 offset = request.shoulder - request.position;
        float lateral = Dot(offset, RightOf(request.forward));
        float along = Dot(offset, request.forward);

        ApproachRequest arrived = request;
        arrived.position = plan.standPoint;
        arrived.forward = plan.facing;
        arrived.shoulder = plan.standPoint + RightOf(plan.facing) * lateral +
                           plan.facing * along + Vec3{0, offset.y, 0};
        return arrived;
    }

    float StandoffAt(const ApproachRequest& arrived)
    {
        return Length(Flat(arrived.target - arrived.shoulder));
    }

    void CheckFollowerLands(const ApproachPlan& plan, const Vec3& target)
    {
        CubicPath path;
        CHECK(BuildApproachPath(plan, path));
        CHECK(!path.IsClosed());

        std::size_t last = path.SegmentCount() - 1;
        CHECK(Near(Normalize(path.Derivative(0, 0.0f)), Normalize(plan.ends.startDirection)));
        CHECK(Near(Normalize(path.Derivative(last, 1.0f)), plan.facing));

        PathFollower follower;
        CHECK(follower.Initialize(path));
        int frames = 0;
        while (!follower.GetState().finished && frames < 100000)
        {
            follower.Advance(1.0 / 60.0);
            ++frames;
        }
        const PathFollowerState& end = follower.GetState();
        CHECK(end.finished);
        CHECK(Near(end.position, plan.standPoint, 1e-5f));
        CHECK(Dot(Flat(target - end.position), plan.facing) > 0.0f);
    }

    void CheckInReach()
    {
        ApproachSettings settings;
        ApproachRequest request = Request(Vec3{-0.2f, 1.4f, 0.55f});
        CHECK(InReach(request, settings));
        ApproachPlan plan = PlanApproach(request, settings);
        CHECK(plan.kind == ApproachKind::None);
        CHECK(plan.controlPoints.empty());

        CubicPath path;
        std::string error;
        CHECK(!BuildApproachPath(plan, path, &error));
        CHECK(!error.empty());

        request.target = request.shoulder + Vec3{0, 0, 1.0f};
        CHECK(!InReach(request, settings));
        request.target = request.shoulder + Vec3{-0.6f, 0, -0.2f};
        CHECK(!InReach(request, settings));
    }

    void CheckFarWalk()
    {
        ApproachSettings settings;
        const Vec3 target{3.0f, 1.2f, 4.0f};
        ApproachRequest request = Request(target);
        CHECK(!InReach(request, settings));

        ApproachPlan plan = PlanApproach(request, settings);
        CHECK(plan.kind == ApproachKind::Walk);
        CHECK(Near(plan.facing, Vec3{0.6f, 0, 0.8f}));
        CHECK(plan.standPoint.y == 0.0f);
        CHECK(plan.controlPoints.size() == 4);
        CHECK(Near(plan.controlPoints.front(), Vec3{0, 0, 0}, 0));
        CHECK(Near(plan.controlPoints.back(), plan.standPoint, 0));
        CHECK(Near(plan.ends.startDirection, Vec3{0, 0, 1}));
        CHECK(Near(plan.ends.endDirection, plan.facing));
        CHECK_NEAR(plan.walkDistance, Length(plan.standPoint), 1e-5f);

        ApproachRequest arrived = Arrived(request, plan);
        CHECK_NEAR(StandoffAt(arrived), settings.standoff, 1e-4f);
        CHECK(Near(Normalize(Flat(target - arrived.shoulder)), plan.facing));
        CHECK(InReach(arrived, settings));
        CHECK(PlanApproach(arrived, settings).kind == ApproachKind::None);

        CheckFollowerLands(plan, target);
    }

    void CheckBehindWalk()
    {
        ApproachSettings settings;
        const Vec3 target{0.0f, 1.2f, -3.0f};
        ApproachRequest request = Request(target);
        ApproachPlan plan = PlanApproach(request, settings);
        CHECK(plan.kind == ApproachKind::Walk);
        CHECK(Near(plan.facing, Vec3{0, 0, -1}));
        CHECK(Near(plan.standPoint, Vec3{-0.2f, 0, -2.5f}));
        CHECK(InReach(Arrived(request, plan), settings));
        CheckFollowerLands(plan, target);
    }

    void CheckTurnInPlace()
    {
        ApproachSettings settings;
        ApproachRequest request = Request(Vec3{0.6f, 1.4f, 0.2f});
        CHECK(!InReach(request, settings));
        ApproachPlan plan = PlanApproach(request, settings);
        CHECK(plan.kind == ApproachKind::Turn);
        CHECK(plan.walkDistance < settings.minWalkMeters);
        CHECK(plan.controlPoints.empty());
        CHECK(Near(plan.facing, Normalize(Vec3{0.6f, 0, 0.2f})));

        ApproachRequest turned = Arrived(request, plan);
        turned.position = request.position;
        turned.shoulder = turned.shoulder - plan.standPoint;
        CHECK(InReach(turned, settings));
    }

    void CheckHighTarget()
    {
        ApproachSettings settings;
        ApproachRequest request = Request(Vec3{-0.2f, 2.6f, 0.3f});
        ApproachPlan plan = PlanApproach(request, settings);
        CHECK(plan.kind == ApproachKind::Turn);
        CHECK_NEAR(StandoffAt(Arrived(request, plan)), settings.minStandoff, 1e-4f);

        request.target = Vec3{3.0f, 2.6f, 4.0f};
        plan = PlanApproach(request, settings);
        CHECK(plan.kind == ApproachKind::Walk);
        CHECK_NEAR(StandoffAt(Arrived(request, plan)), settings.minStandoff, 1e-4f);

        request.target = Vec3{3.0f, 1.9f, 4.0f};
        plan = PlanApproach(request, settings);
        float rise = 1.9f - 1.5f;
        float reach = request.reach * settings.reachMargin;
        CHECK_NEAR(StandoffAt(Arrived(request, plan)),
                   std::min(settings.standoff, std::sqrt(reach * reach - rise * rise)),
                   1e-4f);
        CHECK(InReach(Arrived(request, plan), settings));
    }

    void CheckTurnTowards()
    {
        const Vec3 forward{0, 0, 1};
        const Vec3 side{1, 0, 0};
        CHECK(Near(TurnTowards(forward, side, 0.0f), forward));
        CHECK(Near(TurnTowards(forward, side, 1.0f), side));
        CHECK(Near(TurnTowards(forward, side, 0.5f), Normalize(Vec3{1, 0, 1})));
        CHECK(Near(TurnTowards(forward, side, -1.0f), forward));
        CHECK(Near(TurnTowards(forward, side, 2.0f), side));

        float previous = 0.0f;
        for (int i = 1; i <= 20; ++i)
        {
            Vec3 heading = TurnTowards(forward, side, i / 20.0f);
            float yaw = std::atan2(heading.x, heading.z);
            CHECK(yaw >= previous - 1e-6f);
            CHECK_NEAR(Length(heading), 1.0f, 1e-5f);
            previous = yaw;
        }

        Vec3 from{std::sin(170.0f * kDegrees), 0, std::cos(170.0f * kDegrees)};
        Vec3 to{std::sin(-170.0f * kDegrees), 0, std::cos(-170.0f * kDegrees)};
        CHECK(Near(TurnTowards(from, to, 0.5f), Vec3{0, 0, -1}));
        CHECK(Near(TurnTowards(Vec3{0, 5, 2}, Vec3{0, -3, 0}, 0.0f), forward));
    }

    void CheckInvalid()
    {
        ApproachSettings settings;
        ApproachRequest request = Request(Vec3{3.0f, 1.2f, 4.0f});
        const float nan = std::numeric_limits<float>::quiet_NaN();

        ApproachRequest bad = request;
        bad.reach = 0.0f;
        CHECK(Throws<std::invalid_argument>([&] { PlanApproach(bad, settings); }));
        bad = request;
        bad.target.x = nan;
        CHECK(Throws<std::invalid_argument>([&] { PlanApproach(bad, settings); }));
        CHECK(Throws<std::invalid_argument>([&] { InReach(bad, settings); }));

        ApproachSettings badSettings;
        badSettings.reachMargin = 1.5f;
        CHECK(Throws<std::invalid_argument>([&] { PlanApproach(request, badSettings); }));
        badSettings = {};
        badSettings.minStandoff = 0.6f;
        CHECK(Throws<std::invalid_argument>([&] { PlanApproach(request, badSettings); }));
        badSettings = {};
        badSettings.handleFraction = 0.0f;
        CHECK(Throws<std::invalid_argument>([&] { PlanApproach(request, badSettings); }));
        CHECK(Throws<std::invalid_argument>([&] { TurnTowards({0, 0, 1}, {1, 0, 0}, nan); }));
    }
}  // namespace

void RunApproachPlannerTests()
{
    CheckInReach();
    CheckFarWalk();
    CheckBehindWalk();
    CheckTurnInPlace();
    CheckHighTarget();
    CheckTurnTowards();
    CheckInvalid();
}
