#pragma once

#include <cstddef>
#include <string>
#include <vector>

#include "anim/Motion/CubicPath.h"

namespace Anim
{
    struct ArcLengthBuildOptions
    {
        double chordTolerance = 1e-5;
        double maxParameterInterval = 1.0 / 64.0;
        unsigned int maxDepth = 18;
        std::size_t maxSamples = 262144;
    };

    struct ArcLengthSample
    {
        double parameter = 0.0;
        double distance = 0.0;
        double normalizedLength = 0.0;
    };

    struct SegmentArcLengthTable
    {
        double length = 0.0;
        std::vector<ArcLengthSample> samples;
    };

    struct PathParameter
    {
        std::size_t segmentIndex = 0;
        double parameter = 0.0;
    };

    struct PathArcLengthSample
    {
        double distance = 0.0;
        double normalizedLength = 0.0;
        PathParameter location;
    };

    enum class ArcLengthBuildStatus
    {
        NotBuilt,
        Success,
        InvalidOptions,
        InvalidPath,
        DepthLimit,
        SampleLimit,
        NumericFailure
    };

    struct ArcLengthBuildReport
    {
        ArcLengthBuildStatus status = ArcLengthBuildStatus::NotBuilt;
        std::string message;
        std::size_t segmentIndex = 0;
        std::size_t sampleCount = 0;
        unsigned int maxDepthReached = 0;
        double maxAcceptedDiscrepancy = 0.0;
        double maxAcceptedInterval = 0.0;
    };

    class ArcLengthTable
    {
       public:
        // Failed builds replace the report, but preserve all last-valid caches and options.
        bool Build(const CubicPath& path, const ArcLengthBuildOptions& options = {});

        const std::vector<SegmentArcLengthTable>& GetSegments() const { return m_segments; }
        const std::vector<PathArcLengthSample>& GetGlobalSamples() const { return m_globalSamples; }
        // Prefix arrays have segmentCount+1 entries, including zero and the path endpoint.
        const std::vector<double>& GetSegmentPrefixes() const { return m_prefixes; }
        const std::vector<double>& GetNormalizedPrefixes() const { return m_normalizedPrefixes; }
        double GetTotalLength() const { return m_totalLength; }
        const ArcLengthBuildOptions& GetOptions() const { return m_options; }
        const ArcLengthBuildReport& GetLastBuildReport() const { return m_report; }

        // Queries throw logic_error before building, invalid_argument for nonfinite input.
        // Clamped queries preserve u=1 as the final segment's t=1; joins use the next t=0.
        PathParameter ParameterAtNormalizedLength(double normalizedLength) const;
        PathParameter ParameterAtWrappedNormalizedLength(double normalizedLength) const;

       private:
        std::vector<SegmentArcLengthTable> m_segments;
        std::vector<PathArcLengthSample> m_globalSamples;
        std::vector<double> m_prefixes;
        std::vector<double> m_normalizedPrefixes;
        double m_totalLength = 0.0;
        ArcLengthBuildOptions m_options;
        ArcLengthBuildReport m_report;
    };
}
