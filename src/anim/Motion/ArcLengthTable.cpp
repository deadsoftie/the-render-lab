#include "anim/Motion/ArcLengthTable.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <utility>

namespace Anim
{
    namespace
    {
        double Distance(const Vec3& a, const Vec3& b)
        {
            double dx = static_cast<double>(a.x) - b.x;
            double dy = static_cast<double>(a.y) - b.y;
            double dz = static_cast<double>(a.z) - b.z;
            return std::sqrt(dx * dx + dy * dy + dz * dz);
        }

        bool Fail(ArcLengthBuildReport& report, ArcLengthBuildStatus status, const char* message)
        {
            report.status = status;
            report.message = message;
            return false;
        }

        struct Builder
        {
            const CubicPath& path;
            const ArcLengthBuildOptions& options;
            ArcLengthBuildReport& report;
            SegmentArcLengthTable& table;

            bool Append(double parameter, double distance)
            {
                if (report.sampleCount >= options.maxSamples)
                    return Fail(report, ArcLengthBuildStatus::SampleLimit,
                                "The path-wide arc-length sample limit was reached.");
                if (!std::isfinite(distance) || (!table.samples.empty() &&
                    (parameter <= table.samples.back().parameter ||
                     distance <= table.samples.back().distance)))
                    return Fail(report, ArcLengthBuildStatus::NumericFailure,
                                "Arc-length samples lost numeric resolution or monotonicity.");
                table.samples.push_back({parameter, distance, 0.0});
                ++report.sampleCount;
                return true;
            }

            bool Subdivide(double ta, const Vec3& pa, double tb, const Vec3& pb,
                           unsigned int depth)
            {
                report.maxDepthReached = std::max(report.maxDepthReached, depth);
                double tm = (ta + tb) * 0.5;
                Vec3 pm = path.Evaluate(report.segmentIndex, static_cast<float>(tm));
                double a = Distance(pa, pm);
                double b = Distance(pm, pb);
                double c = Distance(pa, pb);
                double discrepancy = a + b - c;
                double roundoff = 16.0 * std::numeric_limits<double>::epsilon() * (a + b + c);
                if (!std::isfinite(discrepancy) || discrepancy < -roundoff)
                    return Fail(report, ArcLengthBuildStatus::NumericFailure,
                                "The chord comparison produced an invalid discrepancy.");
                discrepancy = std::max(0.0, discrepancy);

                // ArcLength.pdf p.6: split on chord error OR width, including hidden S bends.
                if (discrepancy > options.chordTolerance ||
                    tb - ta > options.maxParameterInterval)
                {
                    if (depth >= options.maxDepth)
                        return Fail(report, ArcLengthBuildStatus::DepthLimit,
                                    "The depth limit prevented chord/width acceptance.");
                    // Left-first traversal preserves the notes' sorted work-list accumulation.
                    return Subdivide(ta, pa, tm, pm, depth + 1) &&
                           Subdivide(tm, pm, tb, pb, depth + 1);
                }

                report.maxAcceptedDiscrepancy =
                    std::max(report.maxAcceptedDiscrepancy, discrepancy);
                report.maxAcceptedInterval = std::max(report.maxAcceptedInterval, tb - ta);
                double startDistance = table.samples.back().distance;
                // Record both halves, not just the accepted interval's endpoint.
                return Append(tm, startDistance + a) && Append(tb, startDistance + a + b);
            }
        };
    }

    bool ArcLengthTable::Build(const CubicPath& path, const ArcLengthBuildOptions& options)
    {
        ArcLengthBuildReport report;
        auto fail = [&](ArcLengthBuildStatus status, const char* message)
        {
            Fail(report, status, message);
            m_report = std::move(report);
            return false;
        };

        // Dyadic midpoints must remain distinct when passed to the float curve evaluator.
        if (!std::isfinite(options.chordTolerance) || options.chordTolerance <= 0.0 ||
            !std::isfinite(options.maxParameterInterval) || options.maxParameterInterval <= 0.0 ||
            options.maxParameterInterval > 1.0 || options.maxDepth > 22 || options.maxSamples < 3)
            return fail(ArcLengthBuildStatus::InvalidOptions, "Invalid arc-length build options.");
        if (path.SegmentCount() == 0)
            return fail(ArcLengthBuildStatus::InvalidPath, "An initialized cubic path is required.");

        std::vector<SegmentArcLengthTable> candidate;
        candidate.reserve(path.SegmentCount());
        for (std::size_t i = 0; i < path.SegmentCount(); ++i)
        {
            report.segmentIndex = i;
            candidate.emplace_back();
            SegmentArcLengthTable& table = candidate.back();
            Builder builder{path, options, report, table};
            if (!builder.Append(0.0, 0.0) ||
                !builder.Subdivide(0.0, path.Evaluate(i, 0.0f),
                                   1.0, path.Evaluate(i, 1.0f), 0))
            {
                m_report = std::move(report);
                return false;
            }

            table.length = table.samples.back().distance;
            if (!std::isfinite(table.length) || table.length <= 1e-8)
                return fail(ArcLengthBuildStatus::InvalidPath, "A curve segment is too short.");
            for (ArcLengthSample& sample : table.samples)
                sample.normalizedLength = sample.distance / table.length;
            table.samples.front().normalizedLength = 0.0;
            table.samples.back().normalizedLength = 1.0;
            for (std::size_t j = 1; j < table.samples.size(); ++j)
                if (table.samples[j].normalizedLength <= table.samples[j - 1].normalizedLength)
                    return fail(ArcLengthBuildStatus::NumericFailure,
                                "Normalized arc lengths lost numeric resolution.");
        }

        m_segments.swap(candidate);
        m_options = options;
        report.status = ArcLengthBuildStatus::Success;
        m_report = std::move(report);
        return true;
    }
}
