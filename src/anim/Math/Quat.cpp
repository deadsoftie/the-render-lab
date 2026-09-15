#include "anim/Math/Quat.h"

#include <cmath>

namespace Anim
{
    namespace
    {
        constexpr float kEpsilon = 1e-6f;
    }

    Quat operator*(const Quat& a, const Quat& b)
    {
        return {
            a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y,
            a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x,
            a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w,
            a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z,
        };
    }

    Quat operator+(const Quat& a, const Quat& b)
    {
        return {a.x + b.x, a.y + b.y, a.z + b.z, a.w + b.w};
    }

    Quat operator-(const Quat& a, const Quat& b)
    {
        return {a.x - b.x, a.y - b.y, a.z - b.z, a.w - b.w};
    }

    Quat operator*(const Quat& q, float s)
    {
        return {q.x * s, q.y * s, q.z * s, q.w * s};
    }

    Quat operator-(const Quat& q)
    {
        return {-q.x, -q.y, -q.z, -q.w};
    }

    float Dot(const Quat& a, const Quat& b)
    {
        return a.x * b.x + a.y * b.y + a.z * b.z + a.w * b.w;
    }

    float Length(const Quat& q)
    {
        return std::sqrt(Dot(q, q));
    }

    Quat Normalize(const Quat& q)
    {
        float len = Length(q);
        if (len < kEpsilon)
            return {0.0f, 0.0f, 0.0f, 1.0f};
        return q * (1.0f / len);
    }

    Quat Conjugate(const Quat& q)
    {
        return {-q.x, -q.y, -q.z, q.w};
    }

    Quat FromAxisAngle(const Vec3& axis, float angleRadians)
    {
        Vec3 n = Normalize(axis);
        float half = angleRadians * 0.5f;
        float s = std::sin(half);
        return Normalize(Quat{n.x * s, n.y * s, n.z * s, std::cos(half)});
    }

    Vec3 Rotate(const Quat& q, const Vec3& v)
    {
        Quat p{v.x, v.y, v.z, 0.0f};
        Quat r = q * p * Conjugate(q);
        return {r.x, r.y, r.z};
    }

    Quat Log(const Quat& q)
    {
        Vec3 axis{q.x, q.y, q.z};
        float sinHalf = Length(axis);
        if (sinHalf < kEpsilon)
            return {0.0f, 0.0f, 0.0f, 0.0f};

        float theta = std::atan2(sinHalf, q.w);
        Vec3 scaled = axis * (theta / sinHalf);
        return {scaled.x, scaled.y, scaled.z, 0.0f};
    }

    Quat Exp(const Quat& q)
    {
        Vec3 axis{q.x, q.y, q.z};
        float theta = Length(axis);
        if (theta < kEpsilon)
            return {0.0f, 0.0f, 0.0f, 1.0f};

        Vec3 scaled = axis * (std::sin(theta) / theta);
        return {scaled.x, scaled.y, scaled.z, std::cos(theta)};
    }

    Quat Lerp(const Quat& a, const Quat& b, float t)
    {
        Quat bAdj = (Dot(a, b) < 0.0f) ? -b : b;
        return Normalize(a + (bAdj - a) * t);
    }

    Quat Slerp(const Quat& a, const Quat& b, float t)
    {
        float cosHalfTheta = Dot(a, b);
        Quat bAdj = b;
        if (cosHalfTheta < 0.0f)
        {
            bAdj = -b;
            cosHalfTheta = -cosHalfTheta;
        }

        if (cosHalfTheta > 1.0f - kEpsilon)
            return Lerp(a, bAdj, t);

        float halfTheta = std::acos(cosHalfTheta);
        float sinHalfTheta = std::sin(halfTheta);
        float wa = std::sin((1.0f - t) * halfTheta) / sinHalfTheta;
        float wb = std::sin(t * halfTheta) / sinHalfTheta;
        return Normalize(a * wa + bAdj * wb);
    }

    Quat ELerp(const Quat& a, const Quat& b, float t)
    {
        Quat bAdj = (Dot(a, b) < 0.0f) ? -b : b;
        Quat relative = Conjugate(a) * bAdj;
        return Normalize(a * Exp(Log(relative) * t));
    }

    ISlerpSegment PrepareISlerp(const Quat& a, const Quat& b, int steps)
    {
        Quat bAdj = (Dot(a, b) < 0.0f) ? -b : b;
        Quat relative = Conjugate(a) * bAdj;
        float invSteps = 1.0f / static_cast<float>(steps > 0 ? steps : 1);
        return {Normalize(Exp(Log(relative) * invSteps))};
    }

    Quat StepISlerp(const Quat& current, const ISlerpSegment& segment)
    {
        return Normalize(current * segment.delta);
    }
}
