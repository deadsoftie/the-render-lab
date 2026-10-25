#pragma once

#include <string>

namespace Anim
{
    struct SpeedProfileSample
    {
        double normalizedDistance = 0.0;
        double normalizedSpeed = 0.0;
    };

    class SpeedProfile
    {
       public:
        bool SetEaseTimes(double easeInEnd, double easeOutStart, std::string* error = nullptr);

        double GetEaseInEnd() const { return m_easeInEnd; }
        double GetEaseOutStart() const { return m_easeOutStart; }
        double GetCruiseRate() const { return m_cruiseRate; }

        // Finite normalized inputs clamp to [0,1]; nonfinite inputs throw invalid_argument.
        SpeedProfileSample Evaluate(double normalizedTime) const;
        double TimeAtDistance(double normalizedDistance) const;

        // Zero cruise speed gives infinite duration; positive speeds require a finite duration.
        double LapDurationSeconds(double pathLength, double cruiseSpeed) const;

       private:
        double m_easeInEnd = 0.2;
        double m_easeOutStart = 0.8;
        double m_cruiseRate = 1.25;
    };
}
