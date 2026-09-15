#pragma once

namespace Anim
{
    struct Vec3
    {
        float x = 0.0f;
        float y = 0.0f;
        float z = 0.0f;
    };

    Vec3 operator+(const Vec3& a, const Vec3& b);
    Vec3 operator-(const Vec3& a, const Vec3& b);
    Vec3 operator-(const Vec3& a);
    Vec3 operator*(const Vec3& a, float s);
    Vec3 operator*(float s, const Vec3& a);

    float Dot(const Vec3& a, const Vec3& b);
    Vec3 Cross(const Vec3& a, const Vec3& b);
    float Length(const Vec3& a);
    Vec3 Normalize(const Vec3& a);
    Vec3 Lerp(const Vec3& a, const Vec3& b, float t);
}
