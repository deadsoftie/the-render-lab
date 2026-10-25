#include "anim/Motion/ArcLengthTable.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
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

        std::vector<double> prefixes{0.0};
        prefixes.reserve(candidate.size() + 1);
        for (std::size_t i = 0; i < candidate.size(); ++i)
        {
            report.segmentIndex = i;
            double next = prefixes.back() + candidate[i].length;
            if (!std::isfinite(next) || next <= prefixes.back())
                return fail(ArcLengthBuildStatus::NumericFailure,
                            "Segment prefixes lost numeric resolution.");
            prefixes.push_back(next);
        }
        double totalLength = prefixes.back();
        std::vector<double> normalizedPrefixes;
        normalizedPrefixes.reserve(prefixes.size());
        for (double prefix : prefixes)
            normalizedPrefixes.push_back(prefix / totalLength);

        std::vector<PathArcLengthSample> globalSamples;
        globalSamples.reserve(report.sampleCount - candidate.size() + 1);
        for (std::size_t i = 0; i < candidate.size(); ++i)
        {
            report.segmentIndex = i;
            const auto& samples = candidate[i].samples;
            // Each join is stored once as the following segment's t=0; retain the final t=1.
            std::size_t count = samples.size() - (i + 1 < candidate.size() ? 1 : 0);
            for (std::size_t j = 0; j < count; ++j)
            {
                double distance = prefixes[i] + samples[j].distance;
                double normalized = distance / totalLength;
                if (!std::isfinite(normalized) || (!globalSamples.empty() &&
                    (distance <= globalSamples.back().distance ||
                     normalized <= globalSamples.back().normalizedLength)))
                    return fail(ArcLengthBuildStatus::NumericFailure,
                                "Global arc-length samples lost numeric resolution.");
                globalSamples.push_back({distance, normalized, {i, samples[j].parameter}});
            }
        }

        m_segments.swap(candidate);
        m_globalSamples.swap(globalSamples);
        m_prefixes.swap(prefixes);
        m_normalizedPrefixes.swap(normalizedPrefixes);
        m_totalLength = totalLength;
        m_options = options;
        report.status = ArcLengthBuildStatus::Success;
        m_report = std::move(report);
        return true;
    }

    PathParameter ArcLengthTable::ParameterAtNormalizedLength(double normalizedLength) const
    {
        if (m_globalSamples.empty())
            throw std::logic_error("Cannot query an uninitialized arc-length table.");
        if (!std::isfinite(normalizedLength))
            throw std::invalid_argument("Normalized arc length must be finite.");
        double u = std::clamp(normalizedLength, 0.0, 1.0);
        auto upper = std::lower_bound(m_globalSamples.begin(), m_globalSamples.end(), u,
            [](const PathArcLengthSample& sample, double value)
            {
                return sample.normalizedLength < value;
            });
        if (upper == m_globalSamples.begin() || upper->normalizedLength == u)
            return upper->location;

        // The lower sample owns intervals ending at a join in the following segment.
        std::size_t segment = (upper - 1)->location.segmentIndex;
        const auto& table = m_segments[segment];
        double local = std::clamp((u * m_totalLength - m_prefixes[segment]) / table.length,
                                  0.0, 1.0);
        auto localUpper = std::lower_bound(table.samples.begin(), table.samples.end(), local,
            [](const ArcLengthSample& sample, double value)
            {
                return sample.normalizedLength < value;
            });
        if (localUpper == table.samples.begin() || localUpper->normalizedLength == local)
            return {segment, localUpper->parameter};
        const auto& lower = *(localUpper - 1);
        double span = localUpper->normalizedLength - lower.normalizedLength;
        if (span <= 0.0)
            throw std::logic_error("Arc-length interpolation requires distinct samples.");
        double fraction = (local - lower.normalizedLength) / span;
        return {segment, lower.parameter + fraction * (localUpper->parameter - lower.parameter)};
    }

    PathParameter ArcLengthTable::ParameterAtWrappedNormalizedLength(double normalizedLength) const
    {
        if (!std::isfinite(normalizedLength))
            throw std::invalid_argument("Normalized arc length must be finite.");
        double wrapped = std::fmod(normalizedLength, 1.0);
        if (wrapped < 0.0)
            wrapped += 1.0;
        // Adding one to a tiny negative remainder can round to the excluded endpoint.
        if (wrapped >= 1.0)
            wrapped = std::nextafter(1.0, 0.0);
        return ParameterAtNormalizedLength(wrapped);
    }
}
