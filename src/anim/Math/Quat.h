#pragma once
#include "anim/Math/Vec3.h"

namespace Anim
{
    struct Quat
    {
        float x = 0.0f;
        float y = 0.0f;
        float z = 0.0f;
        float w = 1.0f;
    };

    Quat operator*(const Quat& a, const Quat& b);
    Quat operator+(const Quat& a, const Quat& b);
    Quat operator-(const Quat& a, const Quat& b);
    Quat operator*(const Quat& q, float s);
    Quat operator-(const Quat& q);

    float Dot(const Quat& a, const Quat& b);
    float Length(const Quat& q);
    Quat Normalize(const Quat& q);
    Quat Conjugate(const Quat& q);
    Quat FromAxisAngle(const Vec3& axis, float angleRadians);
    Vec3 Rotate(const Quat& q, const Vec3& v);

    // Exponential map: pure quaternion (w=0, xyz=axis*halfAngle) <-> unit quaternion.
    Quat Log(const Quat& q);
    Quat Exp(const Quat& q);

    Quat Lerp(const Quat& a, const Quat& b, float t);
    Quat Slerp(const Quat& a, const Quat& b, float t);
    Quat ELerp(const Quat& a, const Quat& b, float t);

    // Incremental slerp: PrepareISlerp bakes the per-step delta rotation once per
    // keyframe segment; StepISlerp then advances by one multiply per frame instead
    // of recomputing the closed-form Slerp from segment-local time each frame.
    struct ISlerpSegment
    {
        Quat delta;
    };

    ISlerpSegment PrepareISlerp(const Quat& a, const Quat& b, int steps);
    Quat StepISlerp(const Quat& current, const ISlerpSegment& segment);
}
