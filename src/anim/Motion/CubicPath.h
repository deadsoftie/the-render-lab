#pragma once

#include <cstddef>
#include <string>
#include <vector>

#include "anim/Math/Vec3.h"

namespace Anim
{
    class CubicPath
    {
       public:
        static constexpr std::size_t kMinControlPoints = 8;
        static constexpr float kMinPointSeparation = 1e-4f;

        // Points are cyclic vertices; do not repeat the first point at the end.
        bool SetControlPoints(const std::vector<Vec3>& points, std::string* error = nullptr);

        const std::vector<Vec3>& GetControlPoints() const { return m_points; }
        std::size_t SegmentCount() const { return m_segments.size(); }

        // Segment indices wrap; finite t clamps to [0,1], preserving the final endpoint.
        // Empty paths throw logic_error; nonfinite parameters throw invalid_argument.
        Vec3 Evaluate(std::size_t segment, float t) const;
        // Derivatives are with respect to local t, evaluated at its clamped value.
        Vec3 Derivative(std::size_t segment, float t) const;
        Vec3 SecondDerivative(std::size_t segment, float t) const;

       private:
        struct Segment
        {
            Vec3 a, b, c, d;
        };

        std::size_t ResolveSegment(std::size_t segment, float t) const;

        std::vector<Vec3> m_points;
        std::vector<Vec3> m_tangents;
        std::vector<Segment> m_segments;
    };
}
