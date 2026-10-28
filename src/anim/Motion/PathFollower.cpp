#include "anim/Motion/PathFollower.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <utility>

namespace Anim
{
    namespace
    {
        bool Validate(const PathFollowerSettings& settings,
                      double length,
                      SpeedProfile& profile,
                      double& duration,
                      std::string& error)
        {
            if (!std::isfinite(settings.lookAheadMeters) || settings.lookAheadMeters <= 0.0 ||
                !std::isfinite(settings.strideMeters) || settings.strideMeters <= 0.0)
            {
                error = "Look-ahead and stride distances must be positive and finite.";
                return false;
            }
            if (!profile.SetEaseTimes(settings.easeInEnd, settings.easeOutStart, &error))
                return false;
            try
            {
                duration = profile.LapDurationSeconds(length, settings.cruiseSpeed);
            }
            catch (const std::exception& exception)
            {
                error = exception.what();
                return false;
            }
            return true;
        }

        // Projects onto the floor plane and normalizes; false when the XZ part is too short to give
        // a heading.
        bool HorizontalDirection(const Vec3& v, Vec3& direction)
        {
            double length = std::hypot(static_cast<double>(v.x), static_cast<double>(v.z));
            if (!std::isfinite(length) || length <= 1e-8)
                return false;
            direction = {static_cast<float>(v.x / length), 0, static_cast<float>(v.z / length)};
            return true;
        }

        double UnwrappedDistance(const PathFollowerState& state, double length)
        {
            // Whole laps plus the current lap's distance, without wrap-around.
            double distance = std::fma(static_cast<double>(state.lapCount),
                                       length,
                                       state.normalizedDistance * length);
            if (!std::isfinite(distance))
                throw std::overflow_error("Unwrapped path distance cannot be represented.");
            return distance;
        }

        void SetGait(PathFollowerState& state, double cycles)
        {
            if (!std::isfinite(cycles) || cycles < 0.0)
                throw std::overflow_error("Gait cycles cannot be represented.");
            state.gaitCycles = cycles;
            state.gaitPhase = std::fmod(cycles, 1.0);
        }
    }  // namespace

    bool PathFollower::Initialize(const CubicPath& path,
                                  const PathFollowerSettings& settings,
                                  const ArcLengthBuildOptions& tableOptions,
                                  std::string* error)
    {
        PathFollower candidate;
        candidate.m_path = path;
        if (!candidate.m_tables.Build(candidate.m_path, tableOptions))
        {
            if (error)
                *error = candidate.m_tables.GetLastBuildReport().message;
            return false;
        }
        std::string validationError;
        if (!Validate(settings,
                      candidate.m_tables.GetTotalLength(),
                      candidate.m_profile,
                      candidate.m_duration,
                      validationError))
        {
            if (error)
                *error = validationError;
            return false;
        }
        candidate.m_settings = settings;
        candidate.m_lookAhead =
            std::min(settings.lookAheadMeters, candidate.m_tables.GetTotalLength() * 0.1);
        candidate.RefreshPose(candidate.m_state);
        *this = std::move(candidate);
        if (error)
            error->clear();
        return true;
    }

    bool PathFollower::SetSettings(const PathFollowerSettings& settings, std::string* error)
    {
        SpeedProfile profile;
        double duration = 0.0;
        std::string validationError;
        bool valid = false;
        if (IsInitialized())
            valid =
                Validate(settings, m_tables.GetTotalLength(), profile, duration, validationError);
        else
            validationError = "Initialize a path before changing follower settings.";
        if (!valid)
        {
            if (error)
                *error = validationError;
            return false;
        }

        // Retiming preserves traveled distance; stride edits preserve the current gait phase.
        m_profile = profile;
        m_settings = settings;
        m_duration = duration;
        m_lookAhead = std::min(settings.lookAheadMeters, m_tables.GetTotalLength() * 0.1);
        m_state.normalizedTime = m_profile.TimeAtDistance(m_state.normalizedDistance);
        m_state.distanceDelta = 0.0;
        m_gaitAnchorDistance = m_state.unwrappedDistance;
        m_gaitAnchorCycles = m_state.gaitCycles;
        AnchorTime();
        RefreshPose(m_state);
        if (error)
            error->clear();
        return true;
    }

    void PathFollower::RequireInitialized() const
    {
        if (!IsInitialized())
            throw std::logic_error("Initialize the path follower before playback.");
    }

    void PathFollower::AnchorTime()
    {
        m_anchorSeconds = m_state.elapsedSeconds;
        m_anchorTime = m_state.normalizedTime;
        m_anchorLap = m_state.lapCount;
    }

    void PathFollower::RefreshPose(PathFollowerState& state) const
    {
        double length = m_tables.GetTotalLength();
        auto location = m_tables.ParameterAtNormalizedLength(state.normalizedDistance);
        state.position =
            m_path.Evaluate(location.segmentIndex, static_cast<float>(location.parameter));
        // Center of interest: a fixed arc distance ahead, wrapping past the seam.
        auto target = m_tables.ParameterAtWrappedNormalizedLength(state.normalizedDistance +
                                                                  m_lookAhead / length);
        state.centerOfInterest =
            m_path.Evaluate(target.segmentIndex, static_cast<float>(target.parameter));
        Vec3 direction;
        if (HorizontalDirection(state.centerOfInterest - state.position, direction))
            state.headingSource = PathHeadingSource::LookAhead;
        else if (HorizontalDirection(m_path.Derivative(location.segmentIndex,
                                                       static_cast<float>(location.parameter)),
                                     direction))
            state.headingSource = PathHeadingSource::Tangent;
        else
        {
            direction = state.forward;
            state.headingSource = PathHeadingSource::Previous;
        }
        state.forward = direction;
        float yaw = std::atan2(direction.x, direction.z);
        state.orientation = FromAxisAngle({0, 1, 0}, yaw);
        // Profile speed is normalized so cruise equals k; rescale to the requested m/s.
        state.speed = state.playing && m_settings.cruiseSpeed > 0.0
                          ? m_settings.cruiseSpeed *
                                (m_profile.Evaluate(state.normalizedTime).normalizedSpeed /
                                 m_profile.GetCruiseRate())
                          : 0.0;
    }

    void PathFollower::Advance(double deltaSeconds)
    {
        RequireInitialized();
        if (!std::isfinite(deltaSeconds) || deltaSeconds < 0.0)
            throw std::invalid_argument("Playback delta must be finite and nonnegative.");
        if (!m_state.playing || m_settings.cruiseSpeed == 0.0 || deltaSeconds == 0.0)
        {
            m_state.distanceDelta = 0.0;
            return;
        }

        PathFollowerState next = m_state;
        // Compensated active time avoids accumulating a normalized-time increment each frame.
        double increment = deltaSeconds - m_elapsedCompensation;
        next.elapsedSeconds = m_state.elapsedSeconds + increment;
        double compensation = (next.elapsedSeconds - m_state.elapsedSeconds) - increment;
        // Lap phase measured from the last anchor; its integer part counts laps crossed this step.
        double phase = m_anchorTime + (next.elapsedSeconds - m_anchorSeconds) / m_duration;
        if (!std::isfinite(next.elapsedSeconds) || !std::isfinite(phase))
            throw std::overflow_error("Playback time cannot be represented.");
        double wholeLaps = std::floor(phase);
        if (wholeLaps >= std::ldexp(1.0, 64))
            throw std::overflow_error("Playback lap count cannot be represented.");
        auto crossed = static_cast<std::uint64_t>(wholeLaps);
        if (crossed > std::numeric_limits<std::uint64_t>::max() - m_anchorLap)
            throw std::overflow_error("Playback lap count cannot be represented.");
        next.lapCount = m_anchorLap + crossed;
        next.normalizedTime = phase - wholeLaps;
        next.normalizedDistance = m_profile.Evaluate(next.normalizedTime).normalizedDistance;
        if (next.lapCount < m_state.lapCount)
            throw std::overflow_error("Playback lost time resolution.");
        // Never step backward within a lap because of rounding.
        if (next.lapCount == m_state.lapCount)
            next.normalizedDistance = std::max(next.normalizedDistance, m_state.normalizedDistance);
        double length = m_tables.GetTotalLength();
        // Distance travelled includes any whole laps crossed, not just the wrapped difference.
        next.distanceDelta = (static_cast<double>(next.lapCount - m_state.lapCount) +
                              next.normalizedDistance - m_state.normalizedDistance) *
                             length;
        next.unwrappedDistance = UnwrappedDistance(next, length);
        // Gait advances one cycle per stride of distance since the last anchor.
        SetGait(next,
                m_gaitAnchorCycles +
                    (next.unwrappedDistance - m_gaitAnchorDistance) / m_settings.strideMeters);
        RefreshPose(next);
        if (!std::isfinite(next.distanceDelta))
            throw std::overflow_error("Playback distance delta cannot be represented.");
        m_state = next;
        m_elapsedCompensation = compensation;
    }

    void PathFollower::SetPlaying(bool playing)
    {
        RequireInitialized();
        m_state.playing = playing;
        m_state.distanceDelta = 0.0;
        RefreshPose(m_state);
    }

    void PathFollower::Reset()
    {
        RequireInitialized();
        bool playing = m_state.playing;
        m_state = {};
        m_state.playing = playing;
        m_elapsedCompensation = 0.0;
        m_gaitAnchorDistance = 0.0;
        m_gaitAnchorCycles = 0.0;
        AnchorTime();
        RefreshPose(m_state);
    }

    void PathFollower::Seek(double normalizedDistance, std::uint64_t lapCount)
    {
        RequireInitialized();
        if (!std::isfinite(normalizedDistance))
            throw std::invalid_argument("Seek distance must be finite.");
        PathFollowerState next = m_state;
        next.lapCount = lapCount;
        next.normalizedDistance = std::clamp(normalizedDistance, 0.0, 1.0);
        next.normalizedTime = m_profile.TimeAtDistance(next.normalizedDistance);
        next.elapsedSeconds = 0.0;
        next.distanceDelta = 0.0;
        next.unwrappedDistance = UnwrappedDistance(next, m_tables.GetTotalLength());
        // Seeks use absolute distance so scrubbing to the same spot always gives the same pose.
        SetGait(next, next.unwrappedDistance / m_settings.strideMeters);
        RefreshPose(next);
        m_state = next;
        m_elapsedCompensation = 0.0;
        m_gaitAnchorDistance = 0.0;
        m_gaitAnchorCycles = 0.0;
        AnchorTime();
    }

    void PathFollower::ReanchorGait(double gaitCycles)
    {
        RequireInitialized();
        SetGait(m_state, gaitCycles);
        m_gaitAnchorDistance = m_state.unwrappedDistance;
        m_gaitAnchorCycles = gaitCycles;
    }
}  // namespace Anim
