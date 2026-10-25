#include "anim/Motion/SpeedProfile.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace Anim
{
    namespace
    {
        double ClampNormalized(double value)
        {
            if (!std::isfinite(value))
                throw std::invalid_argument("The normalized profile input must be finite.");
            return std::clamp(value, 0.0, 1.0);
        }
    }

    bool SpeedProfile::SetEaseTimes(double easeInEnd, double easeOutStart, std::string* error)
    {
        if (!std::isfinite(easeInEnd) || !std::isfinite(easeOutStart) ||
            easeInEnd <= 0.0 || easeInEnd >= easeOutStart || easeOutStart >= 1.0)
        {
            if (error)
                *error = "Ease times must be finite and satisfy 0 < easeInEnd < easeOutStart < 1.";
            return false;
        }

        // Normalize the trapezoidal speed's area to one full lap of distance.
        double cruiseRate = 2.0 / (1.0 + easeOutStart - easeInEnd);
        m_easeInEnd = easeInEnd;
        m_easeOutStart = easeOutStart;
        m_cruiseRate = cruiseRate;
        if (error)
            error->clear();
        return true;
    }

    SpeedProfileSample SpeedProfile::Evaluate(double normalizedTime) const
    {
        double t = ClampNormalized(normalizedTime);
        if (t == 0.0)
            return {0.0, 0.0};
        if (t == 1.0)
            return {1.0, 0.0};
        if (t < m_easeInEnd)
        {
            double ramp = t / m_easeInEnd;
            return {0.5 * m_cruiseRate * t * ramp, m_cruiseRate * ramp};
        }
        if (t <= m_easeOutStart)
            return {m_cruiseRate * (t - m_easeInEnd * 0.5), m_cruiseRate};

        double remaining = 1.0 - t;
        double ramp = remaining / (1.0 - m_easeOutStart);
        return {1.0 - 0.5 * m_cruiseRate * remaining * ramp, m_cruiseRate * ramp};
    }

    double SpeedProfile::TimeAtDistance(double normalizedDistance) const
    {
        double u = ClampNormalized(normalizedDistance);
        if (u == 0.0 || u == 1.0)
            return u;
        double easeInDistance = 0.5 * m_cruiseRate * m_easeInEnd;
        double easeOutDistance = 1.0 - 0.5 * m_cruiseRate * (1.0 - m_easeOutStart);
        if (u < easeInDistance)
            return std::sqrt(m_easeInEnd) * std::sqrt(2.0 * u / m_cruiseRate);
        if (u <= easeOutDistance)
            return u / m_cruiseRate + m_easeInEnd * 0.5;
        // Split square roots avoid an unnecessarily tiny product near an ease endpoint.
        return 1.0 - std::sqrt(1.0 - m_easeOutStart) *
                         std::sqrt(2.0 * (1.0 - u) / m_cruiseRate);
    }

    double SpeedProfile::LapDurationSeconds(double pathLength, double cruiseSpeed) const
    {
        if (!std::isfinite(pathLength) || pathLength <= 0.0 ||
            !std::isfinite(cruiseSpeed) || cruiseSpeed < 0.0)
            throw std::invalid_argument("Path length must be positive and cruise speed nonnegative.");
        if (cruiseSpeed == 0.0)
            return std::numeric_limits<double>::infinity();

        double duration = (pathLength / cruiseSpeed) * m_cruiseRate;
        if (!std::isfinite(duration) || duration <= 0.0)
            throw std::overflow_error("The lap duration cannot be represented as a positive double.");
        return duration;
    }
}
