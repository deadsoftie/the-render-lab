#pragma once
#include "anim/Math/Mat4.h"
#include "anim/Math/Vec3.h"

namespace Anim
{
    struct Mat3
    {
        float m[9] = {1, 0, 0, 0, 1, 0, 0, 0, 1};
    };

    float At(const Mat3& a, int row, int col);
    void Set(Mat3& a, int row, int col, float value);
    Vec3 Column(const Mat3& a, int col);
    Mat3 FromColumns(const Vec3& c0, const Vec3& c1, const Vec3& c2);
    Mat3 Diagonal(const Vec3& d);

    Mat3 operator+(const Mat3& a, const Mat3& b);
    Mat3 operator-(const Mat3& a, const Mat3& b);
    Mat3 operator*(const Mat3& a, float s);
    Mat3 operator*(float s, const Mat3& a);
    Mat3 operator*(const Mat3& a, const Mat3& b);
    Vec3 operator*(const Mat3& a, const Vec3& v);

    Mat3 Transpose(const Mat3& a);
    float Determinant(const Mat3& a);
    Mat3 Inverse(const Mat3& a);

    Mat3 Skew(const Vec3& w);
    Mat3 Outer(const Vec3& a, const Vec3& b);
    Mat3 Orthonormalize(const Mat3& r);
    Mat4 ToMat4(const Mat3& r, const Vec3& x);
}
