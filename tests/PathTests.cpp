#include "MotionTests.h"
#include "TestFramework.h"
#include "anim/Motion/CubicPath.h"

#include <cmath>
#include <limits>
#include <stdexcept>

namespace
{
    using Anim::CubicPath;
    using Anim::Vec3;

    std::vector<Vec3> DefaultPoints()
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

    void CheckGeometry(const std::vector<Vec3>& points)
    {
        CubicPath path;
        CHECK(path.SetControlPoints(points));
        CHECK(path.SegmentCount() == points.size());

        for (std::size_t i = 0; i < points.size(); ++i)
        {
            std::size_t next = (i + 1) % points.size();
            std::size_t previous = i == 0 ? points.size() - 1 : i - 1;
            CHECK(Near(path.Evaluate(i, 0), points[i], 0));
            CHECK(Near(path.Evaluate(i, 1), points[next], 0));
            CHECK(Near(path.Derivative(i, 0), (points[next] - points[previous]) * 0.5f));
            CHECK(Near(path.Derivative(i, 1), path.Derivative(next, 0), 0));

            constexpr float h = 0.001f;
            for (float t : {0.2f, 0.5f, 0.8f})
            {
                Vec3 finiteDifference =
                    (path.Evaluate(i, t + h) - path.Evaluate(i, t - h)) * (0.5f / h);
                CHECK(Near(path.Derivative(i, t), finiteDifference, 0.003f));
                Vec3 secondDifference =
                    (path.Derivative(i, t + h) - path.Derivative(i, t - h)) * (0.5f / h);
                CHECK(Near(path.SecondDerivative(i, t), secondDifference, 0.005f));
            }

            // One-sided position differences check the actual polynomials on both sides of a join.
            Vec3 left = (path.Evaluate(i, 1) - path.Evaluate(i, 1 - h)) * (1.0f / h);
            Vec3 right = (path.Evaluate(next, h) - path.Evaluate(next, 0)) * (1.0f / h);
            CHECK(Near(left, path.Derivative(i, 1), 0.025f));
            CHECK(Near(right, path.Derivative(next, 0), 0.025f));
        }
    }
}

void RunPathTests()
{
    using namespace Anim;

    const auto points = DefaultPoints();
    CheckGeometry(points);
    auto spatialPoints = points;
    for (std::size_t i = 0; i < spatialPoints.size(); ++i)
        spatialPoints[i].y = static_cast<float>(i % 3) * 0.75f;
    CheckGeometry(spatialPoints);

    auto eightPoints = points;
    eightPoints.resize(8);
    CheckGeometry(eightPoints);

    CubicPath path;
    CHECK(path.SegmentCount() == 0);
    CHECK(Throws<std::logic_error>([&] { path.Evaluate(0, 0); }));
    CHECK(Throws<std::logic_error>([&] { path.Derivative(0, 0); }));
    CHECK(Throws<std::logic_error>([&] { path.SecondDerivative(0, 0); }));
    std::string error = "old error";
    CHECK(path.SetControlPoints(points, &error));
    CHECK(error.empty());
    CHECK(path.GetControlPoints().size() == points.size());

    for (std::size_t i = 0; i < points.size(); ++i)
    {
        CHECK(Near(path.Evaluate(i + points.size() * 3, 0.37f), path.Evaluate(i, 0.37f), 0));
        CHECK(Near(path.Derivative(i + points.size(), 0.37f), path.Derivative(i, 0.37f), 0));
        CHECK(Near(path.Evaluate(i, -1), path.Evaluate(i, 0), 0));
        CHECK(Near(path.Evaluate(i, 2), path.Evaluate(i, 1), 0));
        CHECK(Near(path.Derivative(i, -1), path.Derivative(i, 0), 0));
        CHECK(Near(path.Derivative(i, 2), path.Derivative(i, 1), 0));
        CHECK(Near(path.SecondDerivative(i, -1), path.SecondDerivative(i, 0), 0));
        CHECK(Near(path.SecondDerivative(i, 2), path.SecondDerivative(i, 1), 0));
    }

    // An independent midpoint identity checks shape beyond endpoint interpolation.
    Vec3 m0 = (points[1] - points.back()) * 0.5f;
    Vec3 m1 = (points[2] - points[0]) * 0.5f;
    CHECK(Near(path.Evaluate(0, 0.5f), (points[0] + points[1]) * 0.5f + (m0 - m1) * 0.125f));

    const Vec3 saved = path.Evaluate(3, 0.37f);
    const Vec3 savedDerivative = path.Derivative(3, 0.37f);
    auto reject = [&](const std::vector<Vec3>& invalid)
    {
        CHECK(!path.SetControlPoints(invalid, &error));
        CHECK(!error.empty());
        CHECK(path.SegmentCount() == points.size());
        CHECK(Near(path.GetControlPoints()[3], points[3], 0));
        CHECK(Near(path.Evaluate(3, 0.37f), saved, 0));
        CHECK(Near(path.Derivative(3, 0.37f), savedDerivative, 0));
    };
    reject({});
    auto invalid = points;
    invalid.resize(7);
    reject(invalid);
    invalid = points;
    invalid[1] = invalid[0];
    reject(invalid);
    invalid = points;
    invalid.back() = invalid.front();
    reject(invalid);
    invalid = points;
    invalid.push_back(invalid.front());
    reject(invalid);
    invalid = points;
    invalid[1] = invalid[0] + Vec3{CubicPath::kMinPointSeparation * 0.25f, 0, 0};
    reject(invalid);

    auto thresholdPoints = points;
    thresholdPoints[0] = {0, 0, 0};
    thresholdPoints[1] = {CubicPath::kMinPointSeparation, 0, 0};
    reject(thresholdPoints);
    thresholdPoints[1].x = CubicPath::kMinPointSeparation * 2.0f;
    CubicPath closePath;
    CHECK(closePath.SetControlPoints(thresholdPoints));

    const float nan = std::numeric_limits<float>::quiet_NaN();
    const float infinity = std::numeric_limits<float>::infinity();
    for (float value : {nan, infinity, -infinity})
    {
        for (int axis = 0; axis < 3; ++axis)
        {
            invalid = points;
            if (axis == 0)
                invalid[4].x = value;
            else if (axis == 1)
                invalid[4].y = value;
            else
                invalid[4].z = value;
            reject(invalid);
        }
        CHECK(Throws<std::invalid_argument>([&] { path.Evaluate(0, value); }));
        CHECK(Throws<std::invalid_argument>([&] { path.Derivative(0, value); }));
        CHECK(Throws<std::invalid_argument>([&] { path.SecondDerivative(0, value); }));
    }

    invalid = points;
    invalid[0].x = std::numeric_limits<float>::max();
    reject(invalid);
    CHECK(!path.SetControlPoints({}));
    CHECK(path.SetControlPoints(path.GetControlPoints(), &error));
    CHECK(error.empty());
    CHECK(Near(path.Evaluate(3, 0.37f), saved, 0));
    CHECK(path.SetControlPoints(eightPoints));
    CHECK(path.SegmentCount() == eightPoints.size());
    CHECK(Near(path.Evaluate(7, 1), eightPoints[0], 0));
}
