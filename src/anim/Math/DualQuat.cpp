#include "anim/Math/DualQuat.h"

namespace Anim
{
    DualQuat MakeDualQuat(const Vec3& translation, const Quat& rotation)
    {
        Quat real = Normalize(rotation);
        Quat t{translation.x, translation.y, translation.z, 0.0f};
        return {real, (t * real) * 0.5f};
    }

    DualQuat operator+(const DualQuat& a, const DualQuat& b)
    {
        return {a.real + b.real, a.dual + b.dual};
    }

    DualQuat operator*(const DualQuat& dq, float s)
    {
        return {dq.real * s, dq.dual * s};
    }

    DualQuat Normalize(const DualQuat& dq)
    {
        float len = Length(dq.real);
        if (len < 1e-8f)
            return {Quat{0.0f, 0.0f, 0.0f, 1.0f}, Quat{0.0f, 0.0f, 0.0f, 0.0f}};
        float invLen = 1.0f / len;
        return {dq.real * invLen, dq.dual * invLen};
    }

    Vec3 ExtractTranslation(const DualQuat& dq)
    {
        Quat t = (dq.dual * Conjugate(dq.real)) * 2.0f;
        return {t.x, t.y, t.z};
    }

    Vec3 TransformPoint(const DualQuat& dq, const Vec3& p)
    {
        return Rotate(dq.real, p) + ExtractTranslation(dq);
    }
}
