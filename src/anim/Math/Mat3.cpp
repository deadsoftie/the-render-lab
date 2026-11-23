#include "anim/Math/Mat3.h"

#include <cmath>
#include <iostream>

namespace Anim
{
    float At(const Mat3& a, int row, int col)
    {
        return a.m[col * 3 + row];
    }

    void Set(Mat3& a, int row, int col, float value)
    {
        a.m[col * 3 + row] = value;
    }

    Vec3 Column(const Mat3& a, int col)
    {
        return Vec3{a.m[col * 3 + 0], a.m[col * 3 + 1], a.m[col * 3 + 2]};
    }

    Mat3 FromColumns(const Vec3& c0, const Vec3& c1, const Vec3& c2)
    {
        Mat3 r;
        const Vec3* cols[3] = {&c0, &c1, &c2};
        for (int col = 0; col < 3; ++col)
        {
            r.m[col * 3 + 0] = cols[col]->x;
            r.m[col * 3 + 1] = cols[col]->y;
            r.m[col * 3 + 2] = cols[col]->z;
        }
        return r;
    }

    Mat3 Diagonal(const Vec3& d)
    {
        Mat3 r;
        Set(r, 0, 0, d.x);
        Set(r, 1, 1, d.y);
        Set(r, 2, 2, d.z);
        return r;
    }

    Mat3 operator+(const Mat3& a, const Mat3& b)
    {
        Mat3 r;
        for (int i = 0; i < 9; ++i)
            r.m[i] = a.m[i] + b.m[i];
        return r;
    }

    Mat3 operator-(const Mat3& a, const Mat3& b)
    {
        Mat3 r;
        for (int i = 0; i < 9; ++i)
            r.m[i] = a.m[i] - b.m[i];
        return r;
    }

    Mat3 operator*(const Mat3& a, float s)
    {
        Mat3 r;
        for (int i = 0; i < 9; ++i)
            r.m[i] = a.m[i] * s;
        return r;
    }

    Mat3 operator*(float s, const Mat3& a)
    {
        return a * s;
    }

    Mat3 operator*(const Mat3& a, const Mat3& b)
    {
        Mat3 r;
        for (int col = 0; col < 3; ++col)
        {
            for (int row = 0; row < 3; ++row)
            {
                float sum = 0.0f;
                for (int k = 0; k < 3; ++k)
                    sum += At(a, row, k) * At(b, k, col);
                Set(r, row, col, sum);
            }
        }
        return r;
    }

    Vec3 operator*(const Mat3& a, const Vec3& v)
    {
        return Vec3{At(a, 0, 0) * v.x + At(a, 0, 1) * v.y + At(a, 0, 2) * v.z,
                    At(a, 1, 0) * v.x + At(a, 1, 1) * v.y + At(a, 1, 2) * v.z,
                    At(a, 2, 0) * v.x + At(a, 2, 1) * v.y + At(a, 2, 2) * v.z};
    }

    Mat3 Transpose(const Mat3& a)
    {
        Mat3 r;
        for (int col = 0; col < 3; ++col)
            for (int row = 0; row < 3; ++row)
                Set(r, row, col, At(a, col, row));
        return r;
    }

    float Determinant(const Mat3& a)
    {
        return Dot(Column(a, 0), Cross(Column(a, 1), Column(a, 2)));
    }

    Mat3 Inverse(const Mat3& a)
    {
        Vec3 c0 = Column(a, 0);
        Vec3 c1 = Column(a, 1);
        Vec3 c2 = Column(a, 2);
        Vec3 r0 = Cross(c1, c2);
        Vec3 r1 = Cross(c2, c0);
        Vec3 r2 = Cross(c0, c1);
        float det = Dot(c0, r0);
        float scale = Length(c0) * Length(c1) * Length(c2);
        if (!(std::abs(det) > 1e-6f * scale))
        {
            std::cerr << "[Anim::Mat3] Inverse: singular matrix, returning identity\n";
            return Mat3{};
        }
        return Transpose(FromColumns(r0, r1, r2)) * (1.0f / det);
    }

    Mat3 Skew(const Vec3& w)
    {
        Mat3 r;
        Set(r, 0, 0, 0.0f);
        Set(r, 0, 1, -w.z);
        Set(r, 0, 2, w.y);
        Set(r, 1, 0, w.z);
        Set(r, 1, 1, 0.0f);
        Set(r, 1, 2, -w.x);
        Set(r, 2, 0, -w.y);
        Set(r, 2, 1, w.x);
        Set(r, 2, 2, 0.0f);
        return r;
    }

    Vec3 Unskew(const Mat3& w)
    {
        return Vec3{0.5f * (At(w, 2, 1) - At(w, 1, 2)),
                    0.5f * (At(w, 0, 2) - At(w, 2, 0)),
                    0.5f * (At(w, 1, 0) - At(w, 0, 1))};
    }

    Mat3 RotationFromVector(const Vec3& rotationVector)
    {
        float angle = Length(rotationVector);
        if (angle < 1e-8f)
            return Mat3{} + Skew(rotationVector);
        Mat3 k = Skew(rotationVector * (1.0f / angle));
        return Mat3{} + k * std::sin(angle) + (k * k) * (1.0f - std::cos(angle));
    }

    Mat3 Outer(const Vec3& a, const Vec3& b)
    {
        return FromColumns(a * b.x, a * b.y, a * b.z);
    }

    Mat3 Orthonormalize(const Mat3& r)
    {
        Vec3 c0 = Normalize(Column(r, 0));
        Vec3 c1 = Column(r, 1);
        c1 = Normalize(c1 - c0 * Dot(c0, c1));
        Vec3 c2 = Cross(c0, c1);
        return FromColumns(c0, c1, c2);
    }

    Mat4 ToMat4(const Mat3& r, const Vec3& x)
    {
        Mat4 out;
        for (int col = 0; col < 3; ++col)
            for (int row = 0; row < 3; ++row)
                out.m[col * 4 + row] = At(r, row, col);
        out.m[12] = x.x;
        out.m[13] = x.y;
        out.m[14] = x.z;
        out.m[15] = 1.0f;
        return out;
    }
}
