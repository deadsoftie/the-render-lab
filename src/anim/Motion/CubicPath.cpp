#include "anim/Motion/CubicPath.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <utility>

namespace Anim
{
    namespace
    {
        bool TooClose(const Vec3& a, const Vec3& b)
        {
            double dx = static_cast<double>(a.x) - b.x;
            double dy = static_cast<double>(a.y) - b.y;
            double dz = static_cast<double>(a.z) - b.z;
            double minimum = CubicPath::kMinPointSeparation;
            return dx * dx + dy * dy + dz * dz <= minimum * minimum;
        }

        bool HasEvaluationHeadroom(const Vec3& a, const Vec3& b, const Vec3& c, const Vec3& d)
        {
            // Bound Horner intermediates and both derivatives before accepting float coefficients.
            double limit = static_cast<double>(std::numeric_limits<float>::max()) / 8.0;
            for (int axis = 0; axis < 3; ++axis)
            {
                double av = axis == 0 ? a.x : (axis == 1 ? a.y : a.z);
                double bv = axis == 0 ? b.x : (axis == 1 ? b.y : b.z);
                double cv = axis == 0 ? c.x : (axis == 1 ? c.y : c.z);
                double dv = axis == 0 ? d.x : (axis == 1 ? d.y : d.z);
                if (std::abs(av) + std::abs(bv) + std::abs(cv) + std::abs(dv) > limit)
                    return false;
            }
            return true;
        }
    }  // namespace

    bool CubicPath::SetControlPoints(const std::vector<Vec3>& points, std::string* error)
    {
        auto fail = [error](const char* message)
        {
            if (error)
                *error = message;
            return false;
        };

        if (points.size() < kMinControlPoints)
            return fail("A closed path requires at least eight control points.");
        for (const Vec3& point : points)
            if (!IsFinite(point))
                return fail("Control point coordinates must be finite.");

        std::size_t count = points.size();
        for (std::size_t i = 0; i < count; ++i)
            if (TooClose(points[i], points[(i + 1) % count]))
                return fail("Neighboring points, including the closing pair, must be separated.");

        std::vector<Vec3> tangents(count);
        for (std::size_t i = 0; i < count; ++i)
        {
            std::size_t previous = i == 0 ? count - 1 : i - 1;
            // Uniform Catmull-Rom tangent: half the chord between the two cyclic neighbours.
            tangents[i] = (points[(i + 1) % count] - points[previous]) * 0.5f;
            if (!IsFinite(tangents[i]))
                return fail("Control point magnitudes exceed the curve's numeric range.");
        }

        return Commit(points, std::move(tangents), true, error);
    }

    bool CubicPath::SetOpenControlPoints(const std::vector<Vec3>& points,
                                         const OpenPathEnds& ends,
                                         std::string* error)
    {
        auto fail = [error](const char* message)
        {
            if (error)
                *error = message;
            return false;
        };

        if (points.size() < kMinOpenControlPoints)
            return fail("An open path requires at least two control points.");
        for (const Vec3& point : points)
            if (!IsFinite(point))
                return fail("Control point coordinates must be finite.");

        std::size_t count = points.size();
        for (std::size_t i = 0; i + 1 < count; ++i)
            if (TooClose(points[i], points[i + 1]))
                return fail("Neighboring points must be separated.");

        // Scaling to the chord length keeps an end tangent in proportion to its segment, so a
        // short final segment does not overshoot or loop.
        auto endTangent = [](const Vec3& direction, const Vec3& chord, Vec3& tangent)
        {
            double length = std::sqrt(static_cast<double>(direction.x) * direction.x +
                                      static_cast<double>(direction.y) * direction.y +
                                      static_cast<double>(direction.z) * direction.z);
            if (!IsFinite(direction) || !std::isfinite(length) || length <= 1e-8)
                return false;
            double scale = std::sqrt(static_cast<double>(chord.x) * chord.x +
                                     static_cast<double>(chord.y) * chord.y +
                                     static_cast<double>(chord.z) * chord.z) /
                           length;
            tangent = direction * static_cast<float>(scale);
            return IsFinite(tangent);
        };

        // Interior tangents are the same Catmull-Rom rule as the closed path; ends with no
        // neighbour on one side fall back to their single chord.
        std::vector<Vec3> tangents(count);
        Vec3 firstChord = points[1] - points[0];
        Vec3 lastChord = points[count - 1] - points[count - 2];
        tangents.front() = firstChord;
        tangents.back() = lastChord;
        if (ends.hasStartDirection &&
            !endTangent(ends.startDirection, firstChord, tangents.front()))
            return fail("The start direction must be finite and nonzero.");
        if (ends.hasEndDirection && !endTangent(ends.endDirection, lastChord, tangents.back()))
            return fail("The end direction must be finite and nonzero.");
        for (std::size_t i = 1; i + 1 < count; ++i)
            tangents[i] = (points[i + 1] - points[i - 1]) * 0.5f;
        for (const Vec3& tangent : tangents)
            if (!IsFinite(tangent))
                return fail("Control point magnitudes exceed the curve's numeric range.");

        return Commit(points, std::move(tangents), false, error);
    }

    bool CubicPath::Commit(const std::vector<Vec3>& points,
                           std::vector<Vec3> tangents,
                           bool closed,
                           std::string* error)
    {
        std::size_t count = points.size();
        // A closed path has a segment per point (the last one closes the loop); an open one
        // has one fewer.
        std::vector<Segment> segments(closed ? count : count - 1);
        for (std::size_t i = 0; i < segments.size(); ++i)
        {
            std::size_t next = (i + 1) % count;
            Segment& s = segments[i];
            // Shared endpoint tangents make every join C1, including the closing seam.
            s.a = (points[i] - points[next]) * 2.0f + tangents[i] + tangents[next];
            s.b = (points[next] - points[i]) * 3.0f - tangents[i] * 2.0f - tangents[next];
            s.c = tangents[i];
            s.d = points[i];
            if (!IsFinite(s.a) || !IsFinite(s.b) || !HasEvaluationHeadroom(s.a, s.b, s.c, s.d))
            {
                if (error)
                    *error = "Control point magnitudes exceed the curve's numeric range.";
                return false;
            }
        }

        // Commit all caches together so an invalid edit leaves the previous curve intact.
        std::vector<Vec3> candidatePoints = points;
        m_points.swap(candidatePoints);
        m_tangents.swap(tangents);
        m_segments.swap(segments);
        m_closed = closed;
        if (error)
            error->clear();
        return true;
    }

    std::size_t CubicPath::ResolveSegment(std::size_t segment, float t) const
    {
        if (m_segments.empty())
            throw std::logic_error("Cannot evaluate an uninitialized cubic path.");
        if (!std::isfinite(t))
            throw std::invalid_argument("The curve parameter must be finite.");
        return segment % m_segments.size();
    }

    // Endpoints return the stored control points exactly so adjacent segments meet bit-for-bit.
    Vec3 CubicPath::Evaluate(std::size_t segment, float t) const
    {
        segment = ResolveSegment(segment, t);
        t = std::clamp(t, 0.0f, 1.0f);
        if (t == 0.0f)
            return m_points[segment];
        if (t == 1.0f)
            return m_points[(segment + 1) % m_points.size()];
        const Segment& s = m_segments[segment];
        return ((s.a * t + s.b) * t + s.c) * t + s.d;
    }

    Vec3 CubicPath::Derivative(std::size_t segment, float t) const
    {
        segment = ResolveSegment(segment, t);
        t = std::clamp(t, 0.0f, 1.0f);
        if (t == 0.0f)
            return m_tangents[segment];
        if (t == 1.0f)
            return m_tangents[(segment + 1) % m_tangents.size()];
        const Segment& s = m_segments[segment];
        // C'(t) = 3a t^2 + 2b t + c in Horner form.
        return (s.a * (3.0f * t) + s.b * 2.0f) * t + s.c;
    }

    Vec3 CubicPath::SecondDerivative(std::size_t segment, float t) const
    {
        segment = ResolveSegment(segment, t);
        t = std::clamp(t, 0.0f, 1.0f);
        const Segment& s = m_segments[segment];
        return s.a * (6.0f * t) + s.b * 2.0f;
    }
}  // namespace Anim
