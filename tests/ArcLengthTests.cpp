#include "MotionTests.h"
#include "TestFramework.h"
#include "anim/Motion/ArcLengthTable.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace
{
    using namespace Anim;

    std::vector<Vec3> DefaultPoints()
    {
        return {{-6, 0, -3}, {-3, 0, -5}, {1, 0, -5}, {5, 0, -3}, {6, 0, 0},
                {4, 0, 4}, {0, 0, 5}, {-4, 0, 4}, {-6, 0, 2}, {-7, 0, 0}};
    }

    double ReferenceSpeed(const std::vector<Vec3>& points, std::size_t segment, double t)
    {
        std::size_t count = points.size();
        const Vec3& before = points[(segment + count - 1) % count];
        const Vec3& start = points[segment];
        const Vec3& end = points[(segment + 1) % count];
        const Vec3& after = points[(segment + 2) % count];
        auto component = [t](double p0, double p1, double p2, double p3)
        {
            double m0 = (p2 - p0) * 0.5;
            double m1 = (p3 - p1) * 0.5;
            return (6 * t * t - 6 * t) * p1 + (3 * t * t - 4 * t + 1) * m0 +
                   (-6 * t * t + 6 * t) * p2 + (3 * t * t - 2 * t) * m1;
        };
        double x = component(before.x, start.x, end.x, after.x);
        double y = component(before.y, start.y, end.y, after.y);
        double z = component(before.z, start.z, end.z, after.z);
        return std::sqrt(x * x + y * y + z * z);
    }

    // Independent quadrature is a test reference, not the production chord-table algorithm.
    double ReferenceLength(const std::vector<Vec3>& points, std::size_t segment, double end = 1)
    {
        constexpr int steps = 4096;
        double h = end / steps;
        double sum = ReferenceSpeed(points, segment, 0) + ReferenceSpeed(points, segment, end);
        for (int i = 1; i < steps; ++i)
            sum += (i % 2 == 0 ? 2 : 4) * ReferenceSpeed(points, segment, i * h);
        return sum * h / 3;
    }

    void CheckTables(const std::vector<Vec3>& points)
    {
        CubicPath path;
        CHECK(path.SetControlPoints(points));
        ArcLengthTable tables;
        bool built = tables.Build(path);
        CHECK(built);
        if (!built)
            return;
        const auto& report = tables.GetLastBuildReport();
        CHECK(report.status == ArcLengthBuildStatus::Success);
        CHECK(report.message.empty());
        CHECK(report.maxAcceptedDiscrepancy <= tables.GetOptions().chordTolerance);
        CHECK(report.maxAcceptedInterval <= tables.GetOptions().maxParameterInterval);
        CHECK(tables.GetSegments().size() == points.size());
        std::size_t sampleCount = 0;
        for (std::size_t i = 0; i < points.size(); ++i)
        {
            const auto& table = tables.GetSegments()[i];
            sampleCount += table.samples.size();
            CHECK(table.samples.size() >= 129);
            CHECK(table.samples.size() % 2 == 1);
            CHECK(table.samples.front().parameter == 0);
            CHECK(table.samples.front().distance == 0);
            CHECK(table.samples.front().normalizedLength == 0);
            CHECK(table.samples.back().parameter == 1);
            CHECK(table.samples.back().normalizedLength == 1);
            CHECK(table.samples.back().distance == table.length);
            CHECK(std::abs(table.length - ReferenceLength(points, i)) < 5e-4);
            for (std::size_t j = 1; j < table.samples.size(); ++j)
            {
                const auto& sample = table.samples[j];
                CHECK(std::isfinite(sample.distance));
                CHECK(sample.parameter > table.samples[j - 1].parameter);
                CHECK(sample.distance > table.samples[j - 1].distance);
                CHECK(sample.normalizedLength > table.samples[j - 1].normalizedLength);
                CHECK(sample.normalizedLength <= 1);
                CHECK(std::abs(sample.normalizedLength - sample.distance / table.length) < 1e-14);
            }
            for (std::size_t j : {table.samples.size() / 4, table.samples.size() / 2})
                CHECK(std::abs(table.samples[j].distance -
                               ReferenceLength(points, i, table.samples[j].parameter)) < 5e-4);
        }
        CHECK(sampleCount == report.sampleCount);
    }
}

void RunArcLengthTests()
{
    using namespace Anim;

    auto points = DefaultPoints();
    CheckTables(points);
    auto spatial = points;
    for (std::size_t i = 0; i < spatial.size(); ++i)
        spatial[i].y = static_cast<float>(i % 3) * 0.75f;
    CheckTables(spatial);

    CubicPath path;
    CHECK(path.SetControlPoints(points));
    ArcLengthBuildOptions loose;
    loose.chordTolerance = 0.01;
    loose.maxParameterInterval = 0.5;
    ArcLengthBuildOptions tight;
    tight.chordTolerance = 1e-7;
    tight.maxParameterInterval = 1.0 / 256;
    ArcLengthTable coarse, fine;
    CHECK(coarse.Build(path, loose));
    CHECK(fine.Build(path, tight));
    CHECK(fine.GetLastBuildReport().sampleCount > coarse.GetLastBuildReport().sampleCount);
    double coarseError = 0, fineError = 0;
    for (std::size_t i = 0; i < points.size(); ++i)
    {
        double reference = ReferenceLength(points, i);
        coarseError += std::abs(coarse.GetSegments()[i].length - reference);
        fineError += std::abs(fine.GetSegments()[i].length - reference);
        CHECK(std::abs(fine.GetSegments()[i].length - reference) < 3e-5);
    }
    CHECK(fineError < coarseError);

    auto toleranceOnly = loose;
    toleranceOnly.maxParameterInterval = 1;
    CHECK(coarse.Build(path, toleranceOnly));
    const auto looseSamples = coarse.GetLastBuildReport().sampleCount;
    double looseCurvedError = std::abs(coarse.GetSegments()[4].length - ReferenceLength(points, 4));
    toleranceOnly.chordTolerance = 1e-7;
    CHECK(fine.Build(path, toleranceOnly));
    CHECK(fine.GetLastBuildReport().sampleCount > looseSamples);
    CHECK(fine.GetLastBuildReport().maxAcceptedDiscrepancy <= toleranceOnly.chordTolerance);
    CHECK(std::abs(fine.GetSegments()[4].length - ReferenceLength(points, 4)) < looseCurvedError);

    // Segment 2 is a unit straight line, then a nonuniform-speed straight cubic, then an S bend.
    std::vector<Vec3> fixture = {{-3, 0, -3}, {-1, 0, 0}, {0, 0, 0}, {1, 0, 0},
                               {2, 0, 0}, {3, 0, 3}, {0, 0, 4}, {-3, 0, 2}};
    ArcLengthBuildOptions unrestricted;
    unrestricted.maxParameterInterval = 1;
    CHECK(path.SetControlPoints(fixture));
    CHECK(coarse.Build(path, unrestricted));
    CHECK(coarse.GetSegments()[2].samples.size() == 3);
    CHECK(coarse.GetSegments()[2].samples[1].parameter == 0.5);
    CHECK(std::abs(coarse.GetSegments()[2].samples[1].distance - 0.5) < 1e-14);
    CHECK(std::abs(coarse.GetSegments()[2].length - 1) < 1e-14);

    fixture[1].x = -3;
    fixture[4].x = 4;
    CHECK(path.SetControlPoints(fixture));
    CHECK(coarse.Build(path, unrestricted));
    CHECK(coarse.GetSegments()[2].samples.size() == 3);
    CHECK(fine.Build(path));
    const auto& straight = fine.GetSegments()[2];
    CHECK(straight.samples.size() == 129);
    CHECK(std::abs(straight.length - 1) < 1e-14);
    auto quarter = std::find_if(straight.samples.begin(), straight.samples.end(),
                               [](const ArcLengthSample& s) { return s.parameter == 0.25; });
    CHECK(quarter != straight.samples.end());
    if (quarter != straight.samples.end())
        CHECK(std::abs(quarter->distance - 0.34375) < 1e-14);

    fixture[1] = {-1, 0, -4};
    fixture[4] = {2, 0, 4};
    CHECK(path.SetControlPoints(fixture));
    CHECK(coarse.Build(path, unrestricted));
    CHECK(coarse.GetSegments()[2].samples.size() == 3);
    CHECK(std::abs(coarse.GetSegments()[2].length - 1) < 1e-14);
    double sReference = ReferenceLength(fixture, 2);
    CHECK(sReference > 1.1);
    CHECK(fine.Build(path));
    CHECK(fine.GetSegments()[2].samples.size() >= 129);
    CHECK(std::abs(fine.GetSegments()[2].length - sReference) < 5e-4);

    ArcLengthTable tables;
    CHECK(tables.GetLastBuildReport().status == ArcLengthBuildStatus::NotBuilt);
    CHECK(tables.GetSegments().empty());
    CubicPath empty;
    CHECK(!tables.Build(empty));
    CHECK(tables.GetLastBuildReport().status == ArcLengthBuildStatus::InvalidPath);
    CHECK(tables.GetSegments().empty());
    CHECK(path.SetControlPoints(points));
    CHECK(tables.Build(path));
    const auto saved = tables.GetSegments();
    auto unchanged = [&]
    {
        CHECK(tables.GetSegments().size() == saved.size());
        for (std::size_t i = 0; i < saved.size(); ++i)
        {
            const auto& current = tables.GetSegments()[i];
            CHECK(current.length == saved[i].length);
            CHECK(current.samples.size() == saved[i].samples.size());
            for (std::size_t j = 0; j < saved[i].samples.size(); ++j)
            {
                CHECK(current.samples[j].parameter == saved[i].samples[j].parameter);
                CHECK(current.samples[j].distance == saved[i].samples[j].distance);
                CHECK(current.samples[j].normalizedLength == saved[i].samples[j].normalizedLength);
            }
        }
        CHECK(tables.GetOptions().maxDepth == 18);
        CHECK(tables.GetOptions().maxParameterInterval == 1.0 / 64);
    };
    auto reject = [&](const ArcLengthBuildOptions& options, ArcLengthBuildStatus status)
    {
        CHECK(!tables.Build(path, options));
        CHECK(tables.GetLastBuildReport().status == status);
        CHECK(!tables.GetLastBuildReport().message.empty());
        unchanged();
    };
    ArcLengthBuildOptions options;
    options.maxDepth = 0;
    reject(options, ArcLengthBuildStatus::DepthLimit);
    options.maxParameterInterval = 1;
    options.chordTolerance = 1e-12;
    reject(options, ArcLengthBuildStatus::DepthLimit);
    options = {};
    options.maxSamples = 3;
    reject(options, ArcLengthBuildStatus::SampleLimit);
    CHECK(tables.GetLastBuildReport().sampleCount <= 3);
    options.maxSamples = 0;
    for (const auto& segment : saved)
        options.maxSamples += segment.samples.size();
    --options.maxSamples;
    reject(options, ArcLengthBuildStatus::SampleLimit);
    CHECK(tables.GetLastBuildReport().segmentIndex == points.size() - 1);
    CHECK(tables.GetLastBuildReport().sampleCount == options.maxSamples);
    options = {};
    options.maxDepth = 23;
    reject(options, ArcLengthBuildStatus::InvalidOptions);
    options = {};
    options.maxSamples = 2;
    reject(options, ArcLengthBuildStatus::InvalidOptions);
    for (double invalid : {0.0, -1.0, std::numeric_limits<double>::infinity(),
                           std::numeric_limits<double>::quiet_NaN()})
    {
        options = {};
        options.chordTolerance = invalid;
        reject(options, ArcLengthBuildStatus::InvalidOptions);
        options = {};
        options.maxParameterInterval = invalid;
        reject(options, ArcLengthBuildStatus::InvalidOptions);
    }
    options = {};
    options.maxParameterInterval = 1.1;
    reject(options, ArcLengthBuildStatus::InvalidOptions);
    CHECK(!tables.Build(empty));
    unchanged();

    auto unresolved = points;
    for (Vec3& point : unresolved)
    {
        point.x += 20000000.0f;
        point.z += 20000000.0f;
    }
    CHECK(path.SetControlPoints(unresolved));
    reject({}, ArcLengthBuildStatus::NumericFailure);
    CHECK(path.SetControlPoints(points));
    CHECK(tables.Build(path, tight));
    CHECK(tables.GetLastBuildReport().status == ArcLengthBuildStatus::Success);
    CHECK(tables.GetLastBuildReport().message.empty());
    CHECK(tables.GetOptions().maxParameterInterval == tight.maxParameterInterval);
}

void RunArcLengthLookupTests()
{
    using namespace Anim;

    const auto points = DefaultPoints();
    CubicPath path;
    CHECK(path.SetControlPoints(points));
    ArcLengthTable tables;
    CHECK(tables.Build(path));
    const auto& prefixes = tables.GetSegmentPrefixes();
    const auto& normalizedPrefixes = tables.GetNormalizedPrefixes();
    const auto& global = tables.GetGlobalSamples();
    CHECK(prefixes.size() == points.size() + 1);
    CHECK(normalizedPrefixes.size() == prefixes.size());
    CHECK(prefixes.front() == 0);
    CHECK(normalizedPrefixes.front() == 0);
    CHECK(prefixes.back() == tables.GetTotalLength());
    CHECK(normalizedPrefixes.back() == 1);
    CHECK(global.size() == tables.GetLastBuildReport().sampleCount - points.size() + 1);
    CHECK(global.front().distance == 0);
    CHECK(global.front().normalizedLength == 0);
    CHECK(global.front().location.segmentIndex == 0);
    CHECK(global.front().location.parameter == 0);
    CHECK(global.back().distance == tables.GetTotalLength());
    CHECK(global.back().normalizedLength == 1);
    CHECK(global.back().location.segmentIndex == points.size() - 1);
    CHECK(global.back().location.parameter == 1);

    bool unequalLengths = false;
    std::vector<double> referencePrefixes{0};
    for (std::size_t i = 0; i < points.size(); ++i)
    {
        CHECK(prefixes[i + 1] > prefixes[i]);
        CHECK(normalizedPrefixes[i + 1] > normalizedPrefixes[i]);
        CHECK(std::abs((prefixes[i + 1] - prefixes[i]) -
                       tables.GetSegments()[i].length) < 1e-13);
        CHECK(std::abs(normalizedPrefixes[i] - prefixes[i] / tables.GetTotalLength()) < 1e-14);
        unequalLengths |= std::abs(normalizedPrefixes[i] -
                                  static_cast<double>(i) / points.size()) > 0.001;
        referencePrefixes.push_back(referencePrefixes.back() + ReferenceLength(points, i));
        PathParameter join = tables.ParameterAtNormalizedLength(normalizedPrefixes[i]);
        CHECK(join.segmentIndex == i);
        CHECK(join.parameter == 0);
        CHECK(std::count_if(global.begin(), global.end(), [&](const PathArcLengthSample& sample)
        {
            return sample.normalizedLength == normalizedPrefixes[i];
        }) == 1);
        if (i > 0)
        {
            auto before = tables.ParameterAtNormalizedLength(
                std::nextafter(normalizedPrefixes[i], 0.0));
            auto after = tables.ParameterAtNormalizedLength(
                std::nextafter(normalizedPrefixes[i], 1.0));
            CHECK(before.segmentIndex == i - 1);
            CHECK(before.parameter > 0.999999);
            CHECK(after.segmentIndex == i);
            CHECK(after.parameter < 0.000001);
        }
    }
    CHECK(unequalLengths);
    for (std::size_t i = 0; i < global.size(); ++i)
    {
        const auto& sample = global[i];
        if (i > 0)
        {
            CHECK(sample.distance > global[i - 1].distance);
            CHECK(sample.normalizedLength > global[i - 1].normalizedLength);
        }
        auto location = tables.ParameterAtNormalizedLength(sample.normalizedLength);
        CHECK(location.segmentIndex == sample.location.segmentIndex);
        CHECK(location.parameter == sample.location.parameter);
    }

    // Compare inverse results to independent quadrature, not a lookup through the same cache.
    double previousParameter = -1;
    for (int i = 0; i <= 400; ++i)
    {
        double u = i / 400.0;
        auto location = tables.ParameterAtNormalizedLength(u);
        CHECK(location.segmentIndex < points.size());
        CHECK(location.parameter >= 0 && location.parameter <= 1);
        double unwrappedParameter = location.segmentIndex + location.parameter;
        CHECK(unwrappedParameter >= previousParameter);
        previousParameter = unwrappedParameter;
        double actualDistance = referencePrefixes[location.segmentIndex] +
            ReferenceLength(points, location.segmentIndex, location.parameter);
        CHECK(std::abs(actualDistance - u * referencePrefixes.back()) < 0.002);
        Vec3 position = path.Evaluate(location.segmentIndex, static_cast<float>(location.parameter));
        Vec3 tangent = path.Derivative(location.segmentIndex, static_cast<float>(location.parameter));
        CHECK(std::isfinite(position.x) && std::isfinite(position.y) && std::isfinite(position.z));
        CHECK(std::isfinite(tangent.x) && std::isfinite(tangent.y) && std::isfinite(tangent.z));
        CHECK(Length(tangent) > 0);
    }

    auto start = tables.ParameterAtNormalizedLength(-2);
    auto end = tables.ParameterAtNormalizedLength(2);
    CHECK(start.segmentIndex == 0 && start.parameter == 0);
    CHECK(end.segmentIndex == points.size() - 1 && end.parameter == 1);
    CHECK(Length(path.Evaluate(end.segmentIndex, 1) - path.Evaluate(start.segmentIndex, 0)) == 0);
    CHECK(Length(path.Derivative(end.segmentIndex, 1) - path.Derivative(start.segmentIndex, 0)) == 0);
    auto seamBefore = tables.ParameterAtNormalizedLength(std::nextafter(1.0, 0.0));
    CHECK(seamBefore.segmentIndex == points.size() - 1);
    CHECK(seamBefore.parameter > 0.999999);
    for (double lap : {-2.0, -1.0, 0.0, 1.0, 2.0})
    {
        auto wrapped = tables.ParameterAtWrappedNormalizedLength(lap);
        CHECK(wrapped.segmentIndex == 0 && wrapped.parameter == 0);
    }
    for (double u : {-2.25, -0.25, 1.25, 3.75})
    {
        double wrappedU = u - std::floor(u);
        auto wrapped = tables.ParameterAtWrappedNormalizedLength(u);
        auto expected = tables.ParameterAtNormalizedLength(wrappedU);
        CHECK(wrapped.segmentIndex == expected.segmentIndex);
        CHECK(wrapped.parameter == expected.parameter);
    }
    auto tinyNegative = tables.ParameterAtWrappedNormalizedLength(
        -std::numeric_limits<double>::denorm_min());
    CHECK(tinyNegative.segmentIndex == points.size() - 1);
    CHECK(tinyNegative.parameter > 0.999999);

    for (double invalid : {std::numeric_limits<double>::infinity(),
                           -std::numeric_limits<double>::infinity(),
                           std::numeric_limits<double>::quiet_NaN()})
    {
        bool clampedThrew = false, wrappedThrew = false;
        try
        {
            tables.ParameterAtNormalizedLength(invalid);
        }
        catch (const std::invalid_argument&)
        {
            clampedThrew = true;
        }
        try
        {
            tables.ParameterAtWrappedNormalizedLength(invalid);
        }
        catch (const std::invalid_argument&)
        {
            wrappedThrew = true;
        }
        CHECK(clampedThrew);
        CHECK(wrappedThrew);
    }
    ArcLengthTable empty;
    CHECK(empty.GetTotalLength() == 0);
    CHECK(empty.GetGlobalSamples().empty());
    CHECK(empty.GetSegmentPrefixes().empty());
    bool emptyThrew = false;
    try
    {
        empty.ParameterAtNormalizedLength(0);
    }
    catch (const std::logic_error&)
    {
        emptyThrew = true;
    }
    CHECK(emptyThrew);

    const auto savedPrefixes = prefixes;
    const auto savedNormalized = normalizedPrefixes;
    const auto savedGlobal = global;
    const double savedLength = tables.GetTotalLength();
    auto savedQuery = tables.ParameterAtNormalizedLength(0.37);
    ArcLengthBuildOptions failure;
    failure.maxDepth = 0;
    CHECK(!tables.Build(path, failure));
    CHECK(tables.GetSegmentPrefixes() == savedPrefixes);
    CHECK(tables.GetNormalizedPrefixes() == savedNormalized);
    CHECK(tables.GetTotalLength() == savedLength);
    CHECK(tables.GetGlobalSamples().size() == savedGlobal.size());
    for (std::size_t i = 0; i < savedGlobal.size(); ++i)
    {
        const auto& current = tables.GetGlobalSamples()[i];
        CHECK(current.distance == savedGlobal[i].distance);
        CHECK(current.normalizedLength == savedGlobal[i].normalizedLength);
        CHECK(current.location.segmentIndex == savedGlobal[i].location.segmentIndex);
        CHECK(current.location.parameter == savedGlobal[i].location.parameter);
    }
    auto preservedQuery = tables.ParameterAtNormalizedLength(0.37);
    CHECK(preservedQuery.segmentIndex == savedQuery.segmentIndex);
    CHECK(preservedQuery.parameter == savedQuery.parameter);

    auto scaled = points;
    for (auto& point : scaled)
        point = point * 2;
    CHECK(path.SetControlPoints(scaled));
    CHECK(tables.Build(path));
    CHECK(std::abs(tables.GetTotalLength() - savedLength * 2) < 1e-12);
    auto scaledQuery = tables.ParameterAtNormalizedLength(0.37);
    CHECK(scaledQuery.segmentIndex == savedQuery.segmentIndex);
    CHECK(std::abs(scaledQuery.parameter - savedQuery.parameter) < 1e-12);

    std::vector<Vec3> straightPoints = {{-3, 0, -3}, {-3, 0, 0}, {0, 0, 0}, {1, 0, 0},
                                      {4, 0, 0}, {3, 0, 3}, {0, 0, 4}, {-3, 0, 2}};
    CHECK(path.SetControlPoints(straightPoints));
    CHECK(tables.Build(path));
    double straightStart = tables.GetSegmentPrefixes()[2];
    for (int i = 1; i < 100; ++i)
    {
        double distance = i / 100.0;
        double u = (straightStart + distance) / tables.GetTotalLength();
        auto location = tables.ParameterAtNormalizedLength(u);
        CHECK(location.segmentIndex == 2);
        double t = location.parameter;
        double actualDistance = 2 * t * t * t - 3 * t * t + 2 * t;
        CHECK(std::abs(actualDistance - distance) < 1e-4);
    }
    auto quarterParameter = tables.ParameterAtNormalizedLength(
        (straightStart + 0.34375) / tables.GetTotalLength());
    CHECK(quarterParameter.segmentIndex == 2);
    CHECK(quarterParameter.parameter == 0.25);
}
