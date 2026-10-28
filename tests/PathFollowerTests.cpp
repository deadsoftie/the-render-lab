#include "MotionTests.h"
#include "TestFramework.h"
#include "anim/Motion/PathFollower.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace
{
    using namespace Anim;

    std::vector<Vec3> Points()
    {
        return {{-6, 0, -3}, {-3, 0, -5}, {1, 0, -5}, {5, 0, -3}, {6, 0, 0},
                {4, 0, 4}, {0, 0, 5}, {-4, 0, 4}, {-6, 0, 2}, {-7, 0, 0}};
    }

    bool Near(const Vec3& a, const Vec3& b, float epsilon = 1e-5f)
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
        return false;
    }

    void CheckEquivalentMotion(const PathFollowerState& a, const PathFollowerState& b)
    {
        CHECK(a.lapCount == b.lapCount);
        CHECK(std::abs(a.normalizedTime - b.normalizedTime) < 1e-10);
        CHECK(std::abs(a.normalizedDistance - b.normalizedDistance) < 1e-10);
        CHECK(std::abs(a.unwrappedDistance - b.unwrappedDistance) < 1e-8);
        CHECK(std::abs(a.gaitCycles - b.gaitCycles) < 1e-8);
        CHECK(std::abs(a.gaitPhase - b.gaitPhase) < 1e-8);
        CHECK(std::abs(a.speed - b.speed) < 1e-9);
        CHECK(Near(a.position, b.position));
        CHECK(Near(a.forward, b.forward));
    }

    PathFollowerState Simulate(const CubicPath& path, double seconds,
                               const std::vector<double>& deltas)
    {
        PathFollower follower;
        CHECK(follower.Initialize(path));
        double elapsed = 0, distance = 0;
        std::size_t i = 0;
        while (elapsed < seconds)
        {
            double dt = std::min(deltas[i++ % deltas.size()], seconds - elapsed);
            follower.Advance(dt);
            CHECK(follower.GetState().distanceDelta >= 0);
            distance += follower.GetState().distanceDelta;
            elapsed += dt;
        }
        CHECK(std::abs(distance - follower.GetState().unwrappedDistance) < 1e-8);
        CHECK(std::abs(follower.GetState().elapsedSeconds - seconds) < 1e-9);
        return follower.GetState();
    }
}

void RunPathFollowerTests()
{
    using namespace Anim;

    CubicPath path;
    CHECK(path.SetControlPoints(Points()));
    PathFollower follower;
    CHECK(!follower.IsInitialized());
    CHECK(Throws<std::logic_error>([&] { follower.Advance(0); }));
    CHECK(Throws<std::logic_error>([&] { follower.Seek(0); }));
    CHECK(Throws<std::logic_error>([&] { follower.Reset(); }));
    CHECK(Throws<std::logic_error>([&] { follower.SetPlaying(false); }));
    CHECK(!follower.SetSettings({}));
    std::string error = "old error";
    CHECK(follower.Initialize(path, {}, {}, &error));
    CHECK(error.empty());
    CHECK(follower.IsInitialized());
    CHECK(follower.GetState().normalizedDistance == 0);
    CHECK(follower.GetState().speed == 0);
    CHECK(Near(follower.GetState().position, Points()[0]));
    CHECK(follower.GetState().headingSource == PathHeadingSource::LookAhead);

    const double duration = follower.GetLapDurationSeconds();
    const double length = follower.GetTables().GetTotalLength();
    const double totalTime = duration * 2.37;
    follower.Advance(totalTime);
    const auto expected = follower.GetState();
    CHECK(expected.lapCount == 2);
    CHECK(std::abs(expected.normalizedTime - 0.37) < 1e-14);
    for (double fps : {30.0, 60.0, 144.0})
        CheckEquivalentMotion(Simulate(path, totalTime, {1 / fps}), expected);
    CheckEquivalentMotion(Simulate(path, totalTime, {0.001, 0.037, 0.004, 0.1, 0.016}), expected);
    CHECK(Near(Rotate(expected.orientation, {0, 0, 1}), expected.forward));
    CHECK(Near(Rotate(expected.orientation, {0, 1, 0}), {0, 1, 0}));
    CHECK(std::abs(Length(expected.orientation) - 1) < 1e-6);
    Vec3 targetDirection = expected.centerOfInterest - expected.position;
    targetDirection.y = 0;
    CHECK(Near(Normalize(targetDirection), expected.forward));

    follower.Reset();
    follower.Advance(duration);
    CHECK(follower.GetState().lapCount == 1);
    CHECK(follower.GetState().normalizedTime == 0);
    CHECK(std::abs(follower.GetState().distanceDelta - length) < 1e-12);
    CHECK(follower.GetState().gaitCycles > 0);
    follower.Advance(duration * 7);
    CHECK(follower.GetState().lapCount == 8);
    CHECK(std::abs(follower.GetState().distanceDelta - length * 7) < 1e-10);
    CHECK(std::abs(follower.GetState().gaitCycles - length * 8 / 1.4) < 1e-10);

    follower.Seek(0.3, 2);
    const auto beforePause = follower.GetState();
    CHECK(std::abs(beforePause.unwrappedDistance - length * 2.3) < 1e-12);
    follower.SetPlaying(false);
    follower.Advance(1000);
    CHECK(follower.GetState().speed == 0);
    CHECK(follower.GetState().elapsedSeconds == 0);
    CHECK(follower.GetState().distanceDelta == 0);
    CHECK(follower.GetState().gaitCycles == beforePause.gaitCycles);
    CHECK(follower.GetState().unwrappedDistance == beforePause.unwrappedDistance);
    follower.SetPlaying(true);
    CHECK(follower.GetState().speed > 0);
    follower.Advance(0.5);
    CHECK(follower.GetState().unwrappedDistance > beforePause.unwrappedDistance);

    auto settings = follower.GetSettings();
    const auto beforeEdit = follower.GetState();
    settings.cruiseSpeed = 3;
    settings.easeInEnd = 0.1;
    settings.easeOutStart = 0.7;
    settings.strideMeters = 2;
    CHECK(follower.SetSettings(settings));
    CHECK(follower.GetState().unwrappedDistance == beforeEdit.unwrappedDistance);
    CHECK(follower.GetState().normalizedDistance == beforeEdit.normalizedDistance);
    CHECK(follower.GetState().gaitCycles == beforeEdit.gaitCycles);
    CHECK(Near(follower.GetState().position, beforeEdit.position, 0));
    CHECK(std::abs(follower.GetProfile().Evaluate(follower.GetState().normalizedTime).
                   normalizedDistance - beforeEdit.normalizedDistance) < 1e-14);
    follower.Advance(0.5);
    CHECK(std::abs((follower.GetState().gaitCycles - beforeEdit.gaitCycles) -
                   (follower.GetState().unwrappedDistance - beforeEdit.unwrappedDistance) / 2) < 1e-12);

    settings.cruiseSpeed = 0;
    CHECK(follower.SetSettings(settings));
    const auto stopped = follower.GetState();
    CHECK(std::isinf(follower.GetLapDurationSeconds()));
    follower.Advance(1000);
    CheckEquivalentMotion(follower.GetState(), stopped);
    CHECK(follower.GetState().speed == 0);
    settings.cruiseSpeed = 3;
    CHECK(follower.SetSettings(settings));
    CHECK(follower.GetState().unwrappedDistance == stopped.unwrappedDistance);
    follower.Advance(0.2);
    CHECK(follower.GetState().unwrappedDistance > stopped.unwrappedDistance);

    follower.Seek(0.37, 1);
    follower.Advance(0.8);
    auto seekResult = follower.GetState();
    follower.Seek(0.8);
    follower.Seek(0.37, 1);
    follower.Advance(0.8);
    CheckEquivalentMotion(follower.GetState(), seekResult);
    follower.Seek(1);
    auto endpoint = follower.GetState();
    CHECK(endpoint.normalizedTime == 1);
    CHECK(endpoint.normalizedDistance == 1);
    CHECK(endpoint.speed == 0);
    follower.Seek(0);
    CHECK(Near(follower.GetState().position, endpoint.position));
    CHECK(Near(follower.GetState().forward, endpoint.forward));
    follower.Seek(1);
    follower.Advance(0.1);
    CHECK(follower.GetState().lapCount == 1);
    CHECK(std::abs(follower.GetState().distanceDelta -
                   follower.GetState().normalizedDistance * length) < 1e-12);
    follower.Seek(-1);
    CHECK(follower.GetState().normalizedDistance == 0);
    follower.Seek(2);
    CHECK(follower.GetState().normalizedDistance == 1);
    follower.SetPlaying(false);
    follower.Reset();
    CHECK(!follower.GetState().playing);
    CHECK(follower.GetState().unwrappedDistance == 0);
    CHECK(follower.GetState().gaitCycles == 0);
    CHECK(follower.GetState().lapCount == 0);

    settings.lookAheadMeters = length;
    CHECK(follower.SetSettings(settings));
    CHECK(follower.GetEffectiveLookAheadMeters() == length * 0.1);
    settings.lookAheadMeters = std::numeric_limits<double>::min();
    CHECK(follower.SetSettings(settings));
    CHECK(follower.GetState().headingSource == PathHeadingSource::Tangent);
    auto cuspPoints = Points();
    cuspPoints.back() = cuspPoints[1];
    CubicPath cusp;
    CHECK(cusp.SetControlPoints(cuspPoints));
    CHECK(follower.Initialize(cusp, settings));
    CHECK(follower.GetState().headingSource == PathHeadingSource::Previous);
    CHECK(Near(follower.GetState().forward, {0, 0, 1}));
    follower.Seek(0.2);
    const Vec3 previousHeading = follower.GetState().forward;
    follower.Seek(0);
    CHECK(follower.GetState().headingSource == PathHeadingSource::Previous);
    CHECK(Near(follower.GetState().forward, previousHeading, 0));

    CHECK(follower.Initialize(path));
    follower.Advance(1);
    auto preserved = follower.GetState();
    for (double invalid : {-1.0, std::numeric_limits<double>::infinity(),
                           std::numeric_limits<double>::quiet_NaN()})
    {
        CHECK(Throws<std::invalid_argument>([&] { follower.Advance(invalid); }));
        CHECK(follower.GetState().elapsedSeconds == preserved.elapsedSeconds);
        CheckEquivalentMotion(follower.GetState(), preserved);
    }
    CHECK(Throws<std::invalid_argument>([&]
    {
        follower.Seek(std::numeric_limits<double>::infinity());
    }));
    CHECK(Throws<std::overflow_error>([&]
    {
        follower.Advance(std::numeric_limits<double>::max());
    }));
    CheckEquivalentMotion(follower.GetState(), preserved);
    for (int field = 0; field < 5; ++field)
    {
        auto invalidSettings = follower.GetSettings();
        if (field == 0)
            invalidSettings.cruiseSpeed = -1;
        if (field == 1)
            invalidSettings.strideMeters = 0;
        if (field == 2)
            invalidSettings.lookAheadMeters = 0;
        if (field == 3)
            invalidSettings.easeInEnd = 0.9;
        if (field == 4)
            invalidSettings.cruiseSpeed = std::numeric_limits<double>::min();
        CHECK(!follower.SetSettings(invalidSettings, &error));
        CHECK(!error.empty());
        CHECK(follower.GetSettings().cruiseSpeed == 1.4);
        CheckEquivalentMotion(follower.GetState(), preserved);
    }
    for (double invalid : {std::numeric_limits<double>::quiet_NaN(),
                           std::numeric_limits<double>::infinity()})
    {
        auto invalidSettings = follower.GetSettings();
        invalidSettings.lookAheadMeters = invalid;
        CHECK(!follower.SetSettings(invalidSettings));
        invalidSettings = follower.GetSettings();
        invalidSettings.strideMeters = invalid;
        CHECK(!follower.SetSettings(invalidSettings));
        invalidSettings = follower.GetSettings();
        invalidSettings.cruiseSpeed = invalid;
        CHECK(!follower.SetSettings(invalidSettings));
        CheckEquivalentMotion(follower.GetState(), preserved);
    }
    CubicPath empty;
    CHECK(!follower.Initialize(empty, {}, {}, &error));
    CheckEquivalentMotion(follower.GetState(), preserved);
    ArcLengthBuildOptions limited;
    limited.maxSamples = 3;
    CHECK(!follower.Initialize(path, {}, limited));
    CheckEquivalentMotion(follower.GetState(), preserved);
    auto otherPoints = Points();
    for (auto& point : otherPoints)
        point = point * 2;
    CHECK(path.SetControlPoints(otherPoints));
    CHECK(follower.GetTables().GetTotalLength() == length);
    CheckEquivalentMotion(follower.GetState(), preserved);

    follower.Seek(0, std::numeric_limits<std::uint64_t>::max());
    auto overflowState = follower.GetState();
    CHECK(Throws<std::overflow_error>([&] { follower.Advance(duration * 2); }));
    CheckEquivalentMotion(follower.GetState(), overflowState);
    follower.Reset();
    CHECK(follower.GetState().lapCount == 0);

    follower.Seek(0.3);
    follower.ReanchorGait(5.25);
    CHECK(std::abs(follower.GetState().gaitPhase - 0.25) < 1e-12);
    double anchoredDistance = follower.GetState().unwrappedDistance;
    follower.Advance(duration * 0.1);
    double expectedCycles =
        5.25 + (follower.GetState().unwrappedDistance - anchoredDistance) /
                   follower.GetSettings().strideMeters;
    CHECK(std::abs(follower.GetState().gaitCycles - expectedCycles) < 1e-9);
    CHECK(Throws<std::overflow_error>([&] { follower.ReanchorGait(-1.0); }));

    auto tinyStride = follower.GetSettings();
    tinyStride.strideMeters = std::numeric_limits<double>::denorm_min();
    CHECK(follower.Initialize(path, tinyStride));
    auto beforeOverflow = follower.GetState();
    CHECK(Throws<std::overflow_error>([&] { follower.Advance(1); }));
    CheckEquivalentMotion(follower.GetState(), beforeOverflow);
}
