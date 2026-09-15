#pragma once
#include "anim/Math/Quat.h"
#include "anim/Math/Vec3.h"

namespace Anim
{
    struct Mat4
    {
        // Column-major, m[col*4+row], matches OpenGL/glm upload layout.
        float m[16] = {1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1};
    };

    Mat4 Identity();
    Mat4 operator*(const Mat4& a, const Mat4& b);
    Mat4 Inverse(const Mat4& in);
    Mat4 Compose(const Vec3& translation, const Quat& rotation, float scale);
    void Decompose(const Mat4& m, Vec3& outTranslation, Quat& outRotation, float& outScale);
    Vec3 TransformPoint(const Mat4& m, const Vec3& v);
}
