#pragma once
#include "anim/Math/Mat4.h"
#include "anim/Math/Vec3.h"

// 3x3 matrix for the rigid-body engine. Defaults to identity.
namespace Anim
{
    struct Mat3
    {
        float m[9] = {1, 0, 0, 0, 1, 0, 0, 0, 1};  // column major
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
    // Identity if singular.
    Mat3 Inverse(const Mat3& a);

    // Skew(w) * v == Cross(w, v).
    Mat3 Skew(const Vec3& w);
    // Inverse of Skew.
    Vec3 Unskew(const Mat3& w);
    // Rodrigues rotation by |v| about v.
    Mat3 RotationFromVector(const Vec3& rotationVector);
    Mat3 Outer(const Vec3& a, const Vec3& b);
    // Gram-Schmidt on the columns.
    Mat3 Orthonormalize(const Mat3& r);
    // [r | x].
    Mat4 ToMat4(const Mat3& r, const Vec3& x);
}
