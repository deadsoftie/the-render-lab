#pragma once

#include <cstdint>
#include <string>

#include "anim/Math/Quat.h"
#include "anim/Motion/ArcLengthTable.h"
#include "anim/Motion/SpeedProfile.h"

namespace Anim
{
    struct PathFollowerSettings
    {
        double cruiseSpeed = 1.4;
        double easeInEnd = 0.2;
        double easeOutStart = 0.8;
        double lookAheadMeters = 0.7;
        double strideMeters = 1.4;
    };

    enum class PathHeadingSource
    {
        LookAhead,
        Tangent,
        Previous
    };

    struct PathFollowerState
    {
        bool playing = true;
        std::uint64_t lapCount = 0;
        double normalizedTime = 0.0;
        double normalizedDistance = 0.0;
        double elapsedSeconds = 0.0;
        double unwrappedDistance = 0.0;
        double distanceDelta = 0.0;
        double speed = 0.0;
        double gaitCycles = 0.0;
        double gaitPhase = 0.0;
        Vec3 position;
        Vec3 centerOfInterest;
        Vec3 forward{0, 0, 1};
        Quat orientation;
        PathHeadingSource headingSource = PathHeadingSource::LookAhead;
    };

    class PathFollower
    {
       public:
        // Own snapshots of curve/tables; a failed initialization preserves the previous follower.
        bool Initialize(const CubicPath& path, const PathFollowerSettings& settings = {},
                        const ArcLengthBuildOptions& tableOptions = {}, std::string* error = nullptr);
        bool SetSettings(const PathFollowerSettings& settings, std::string* error = nullptr);
        bool IsInitialized() const { return m_tables.GetTotalLength() > 0.0; }

        // Finite nonnegative dt is required; oversized/unrepresentable updates throw without commit.
        void Advance(double deltaSeconds);
        void SetPlaying(bool playing);
        void Reset();
        // Seek clamps finite distance to [0,1], resets the elapsed clock, and samples absolute gait.
        void Seek(double normalizedDistance, std::uint64_t lapCount = 0);
        // Sets the current gait cycle count and continues it by distance from here; keeps the pose phase across a path rebuild.
        void ReanchorGait(double gaitCycles);

        const PathFollowerState& GetState() const { return m_state; }
        const PathFollowerSettings& GetSettings() const { return m_settings; }
        const CubicPath& GetPath() const { return m_path; }
        const ArcLengthTable& GetTables() const { return m_tables; }
        const SpeedProfile& GetProfile() const { return m_profile; }
        double GetLapDurationSeconds() const { return m_duration; }
        double GetEffectiveLookAheadMeters() const { return m_lookAhead; }

       private:
        void RequireInitialized() const;
        void RefreshPose(PathFollowerState& state) const;
        void AnchorTime();

        CubicPath m_path;
        ArcLengthTable m_tables;
        SpeedProfile m_profile;
        PathFollowerSettings m_settings;
        PathFollowerState m_state;
        double m_duration = 0.0;
        double m_lookAhead = 0.0;
        double m_elapsedCompensation = 0.0;
        double m_anchorSeconds = 0.0;
        double m_anchorTime = 0.0;
        std::uint64_t m_anchorLap = 0;
        double m_gaitAnchorDistance = 0.0;
        double m_gaitAnchorCycles = 0.0;
    };
}
