#include "MotionTests.h"
#include "TestFramework.h"
#include "anim/Motion/SpeedProfile.h"

#include <cmath>
#include <limits>
#include <stdexcept>
#include <utility>

namespace
{
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

    void CheckProfile(double a, double b)
    {
        Anim::SpeedProfile profile;
        CHECK(profile.SetEaseTimes(a, b));
        const double cruise = profile.GetCruiseRate();
        CHECK(profile.Evaluate(0).normalizedDistance == 0);
        CHECK(profile.Evaluate(0).normalizedSpeed == 0);
        CHECK(profile.Evaluate(1).normalizedDistance == 1);
        CHECK(profile.Evaluate(1).normalizedSpeed == 0);
        CHECK(std::abs(profile.Evaluate(a).normalizedDistance - cruise * a * 0.5) < 1e-14);
        CHECK(std::abs(profile.Evaluate(b).normalizedDistance -
                       (1 - cruise * (1 - b) * 0.5)) < 1e-14);

        for (double boundary : {a, b})
        {
            auto left = profile.Evaluate(std::nextafter(boundary, 0.0));
            auto right = profile.Evaluate(std::nextafter(boundary, 1.0));
            CHECK(std::abs(left.normalizedDistance - right.normalizedDistance) < 1e-12);
            CHECK(std::abs(left.normalizedSpeed - right.normalizedSpeed) < 1e-12);
            CHECK(std::abs(profile.TimeAtDistance(profile.Evaluate(boundary).normalizedDistance) -
                           boundary) < 1e-12);
        }

        double previousDistance = -1;
        for (int i = 0; i <= 1000; ++i)
        {
            double t = i / 1000.0;
            auto sample = profile.Evaluate(t);
            CHECK(std::isfinite(sample.normalizedDistance));
            CHECK(sample.normalizedDistance >= previousDistance);
            CHECK(sample.normalizedDistance >= 0 && sample.normalizedDistance <= 1);
            CHECK(sample.normalizedSpeed >= 0 && sample.normalizedSpeed <= cruise);
            CHECK(std::abs(profile.TimeAtDistance(sample.normalizedDistance) - t) < 1e-11);
            CHECK(std::abs(profile.Evaluate(profile.TimeAtDistance(t)).normalizedDistance - t) < 1e-12);
            previousDistance = sample.normalizedDistance;
        }
        for (int i = 0; i <= 100; ++i)
        {
            double t = a + (b - a) * (i / 100.0);
            CHECK(std::abs(profile.Evaluate(t).normalizedSpeed - cruise) < 1e-12);
        }

        // These phase boundaries align with this grid, so trapezoids integrate linear speed exactly.
        constexpr int steps = 10000;
        double speedIntegral = 0;
        for (int i = 1; i < steps; ++i)
            speedIntegral += profile.Evaluate(i / static_cast<double>(steps)).normalizedSpeed;
        speedIntegral /= steps;
        CHECK(std::abs(speedIntegral - 1) < 1e-12);

        for (double t : {a * 0.5, (a + b) * 0.5, (1 + b) * 0.5})
        {
            const double h = 1e-7;
            double derivative = (profile.Evaluate(t + h).normalizedDistance -
                                 profile.Evaluate(t - h).normalizedDistance) / (2 * h);
            CHECK(std::abs(derivative - profile.Evaluate(t).normalizedSpeed) < 1e-8);
        }

        const double length = 12, speed = 3.5;
        double duration = profile.LapDurationSeconds(length, speed);
        CHECK(std::abs(length / duration * cruise - speed) < 1e-14);
        CHECK(std::abs(length * speedIntegral - length) < 1e-11);
        double cruiseDistance = length * (profile.Evaluate(b).normalizedDistance -
                                          profile.Evaluate(a).normalizedDistance);
        CHECK(std::abs(cruiseDistance - speed * duration * (b - a)) < 1e-13);
        CHECK(profile.Evaluate(123.0 / profile.LapDurationSeconds(length, 0)).normalizedDistance == 0);
    }
}

void RunSpeedProfileTests()
{
    using Anim::SpeedProfile;

    SpeedProfile profile;
    CHECK(profile.GetEaseInEnd() == 0.2);
    CHECK(profile.GetEaseOutStart() == 0.8);
    CHECK(profile.GetCruiseRate() == 1.25);
    CHECK(std::abs(profile.Evaluate(0.2).normalizedDistance - 0.125) < 1e-14);
    CHECK(std::abs(profile.Evaluate(0.5).normalizedDistance - 0.5) < 1e-14);
    CHECK(std::abs(profile.Evaluate(0.8).normalizedDistance - 0.875) < 1e-14);
    CHECK(profile.LapDurationSeconds(10, 2) == 6.25);
    CHECK(profile.TimeAtDistance(0) == 0);
    CHECK(profile.TimeAtDistance(1) == 1);

    for (auto [a, b] : {std::pair{0.2, 0.8}, {0.1, 0.6}, {0.01, 0.99},
                        {0.49, 0.51}, {0.7, 0.9}, {0.001, 0.002}})
        CheckProfile(a, b);

    CHECK(profile.Evaluate(-1).normalizedDistance == 0);
    CHECK(profile.Evaluate(-1).normalizedSpeed == 0);
    CHECK(profile.Evaluate(2).normalizedDistance == 1);
    CHECK(profile.Evaluate(2).normalizedSpeed == 0);
    CHECK(profile.TimeAtDistance(-1) == 0);
    CHECK(profile.TimeAtDistance(2) == 1);
    CHECK(std::isinf(profile.LapDurationSeconds(10, 0)));

    std::string error;
    for (auto [a, b] : {std::pair{0.0, 0.8}, {-0.1, 0.8}, {0.8, 0.2},
                        {0.5, 0.5}, {0.2, 1.0}, {0.2, 1.1}})
    {
        CHECK(!profile.SetEaseTimes(a, b, &error));
        CHECK(!error.empty());
        CHECK(profile.GetEaseInEnd() == 0.2);
        CHECK(profile.GetEaseOutStart() == 0.8);
        CHECK(profile.GetCruiseRate() == 1.25);
    }
    for (double invalid : {std::numeric_limits<double>::quiet_NaN(),
                           std::numeric_limits<double>::infinity(),
                           -std::numeric_limits<double>::infinity()})
    {
        CHECK(!profile.SetEaseTimes(invalid, 0.8));
        CHECK(!profile.SetEaseTimes(0.2, invalid));
        CHECK(profile.GetCruiseRate() == 1.25);
        CHECK(Throws<std::invalid_argument>([&] { profile.Evaluate(invalid); }));
        CHECK(Throws<std::invalid_argument>([&] { profile.TimeAtDistance(invalid); }));
        CHECK(Throws<std::invalid_argument>([&] { profile.LapDurationSeconds(invalid, 1); }));
        CHECK(Throws<std::invalid_argument>([&] { profile.LapDurationSeconds(10, invalid); }));
    }
    for (double invalidLength : {0.0, -1.0})
        CHECK(Throws<std::invalid_argument>([&] { profile.LapDurationSeconds(invalidLength, 1); }));
    CHECK(Throws<std::invalid_argument>([&] { profile.LapDurationSeconds(10, -1); }));
    CHECK(Throws<std::overflow_error>([&]
    {
        profile.LapDurationSeconds(std::numeric_limits<double>::max(),
                                   std::numeric_limits<double>::min());
    }));
    CHECK(Throws<std::overflow_error>([&]
    {
        profile.LapDurationSeconds(std::numeric_limits<double>::min(),
                                   std::numeric_limits<double>::max());
    }));
    CHECK(profile.LapDurationSeconds(std::numeric_limits<double>::max(),
                                     std::numeric_limits<double>::max()) == 1.25);
    CHECK(profile.SetEaseTimes(0.1, 0.6, &error));
    CHECK(error.empty());
    CHECK(profile.GetEaseInEnd() == 0.1);
    CHECK(profile.GetEaseOutStart() == 0.6);
}
