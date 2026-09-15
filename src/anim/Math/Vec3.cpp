#include "anim/Math/Vec3.h"

#include <cmath>

namespace Anim
{
    Vec3 operator+(const Vec3& a, const Vec3& b)
    {
        return {a.x + b.x, a.y + b.y, a.z + b.z};
    }

    Vec3 operator-(const Vec3& a, const Vec3& b)
    {
        return {a.x - b.x, a.y - b.y, a.z - b.z};
    }

    Vec3 operator-(const Vec3& a)
    {
        return {-a.x, -a.y, -a.z};
    }

    Vec3 operator*(const Vec3& a, float s)
    {
        return {a.x * s, a.y * s, a.z * s};
    }

    Vec3 operator*(float s, const Vec3& a)
    {
        return a * s;
    }

    float Dot(const Vec3& a, const Vec3& b)
    {
        return a.x * b.x + a.y * b.y + a.z * b.z;
    }

    Vec3 Cross(const Vec3& a, const Vec3& b)
    {
        return {a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x};
    }

    float Length(const Vec3& a)
    {
        return std::sqrt(Dot(a, a));
    }

    Vec3 Normalize(const Vec3& a)
    {
        float len = Length(a);
        if (len < 1e-8f)
            return {0.0f, 0.0f, 0.0f};
        return a * (1.0f / len);
    }

    Vec3 Lerp(const Vec3& a, const Vec3& b, float t)
    {
        return a + (b - a) * t;
    }
}
