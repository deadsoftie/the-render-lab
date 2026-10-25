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
        // Failed builds replace the report, but preserve the last valid tables and options.
        bool Build(const CubicPath& path, const ArcLengthBuildOptions& options = {});

        const std::vector<SegmentArcLengthTable>& GetSegments() const { return m_segments; }
        const ArcLengthBuildOptions& GetOptions() const { return m_options; }
        const ArcLengthBuildReport& GetLastBuildReport() const { return m_report; }

       private:
        std::vector<SegmentArcLengthTable> m_segments;
        ArcLengthBuildOptions m_options;
        ArcLengthBuildReport m_report;
    };
}
