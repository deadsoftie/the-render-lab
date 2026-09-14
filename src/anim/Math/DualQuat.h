#pragma once
#include "anim/Math/Quat.h"
#include "anim/Math/Vec3.h"

namespace Anim
{
    // Encodes a rigid (rotation+translation, no scale) transform as a pair of quaternions; unlike a matrix, weighted-sum-then-renormalize blending interpolates rotation correctly even when inputs diverge sharply.
    struct DualQuat
    {
        Quat real{0.0f, 0.0f, 0.0f, 1.0f};
        Quat dual{0.0f, 0.0f, 0.0f, 0.0f};
    };

    DualQuat MakeDualQuat(const Vec3& translation, const Quat& rotation);
    DualQuat operator+(const DualQuat& a, const DualQuat& b);
    DualQuat operator*(const DualQuat& dq, float s);
    DualQuat Normalize(const DualQuat& dq);
    Vec3 ExtractTranslation(const DualQuat& dq);
    Vec3 TransformPoint(const DualQuat& dq, const Vec3& p);
}
