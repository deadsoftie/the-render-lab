#include "anim/Math/Mat4.h"

#include <cmath>
#include <utility>

namespace Anim
{
    namespace
    {
        float At(const Mat4& mat, int row, int col)
        {
            return mat.m[col * 4 + row];
        }

        void Set(Mat4& mat, int row, int col, float value)
        {
            mat.m[col * 4 + row] = value;
        }
    }

    Mat4 Identity()
    {
        return Mat4{};
    }

    Mat4 operator*(const Mat4& a, const Mat4& b)
    {
        Mat4 result;
        for (int col = 0; col < 4; ++col)
        {
            for (int row = 0; row < 4; ++row)
            {
                float sum = 0.0f;
                for (int k = 0; k < 4; ++k)
                    sum += At(a, row, k) * At(b, k, col);
                Set(result, row, col, sum);
            }
        }
        return result;
    }

    // Gauss-Jordan elimination on the [M|I] augmented matrix; general inverse,
    // does not assume the bind-pose matrices imported from FBX are pure TRS.
    Mat4 Inverse(const Mat4& in)
    {
        float aug[4][8];
        for (int row = 0; row < 4; ++row)
        {
            for (int col = 0; col < 4; ++col)
                aug[row][col] = At(in, row, col);
            for (int col = 0; col < 4; ++col)
                aug[row][4 + col] = (row == col) ? 1.0f : 0.0f;
        }

        for (int pivot = 0; pivot < 4; ++pivot)
        {
            int pivotRow = pivot;
            float pivotMax = std::abs(aug[pivot][pivot]);
            for (int row = pivot + 1; row < 4; ++row)
            {
                float v = std::abs(aug[row][pivot]);
                if (v > pivotMax)
                {
                    pivotMax = v;
                    pivotRow = row;
                }
            }
            if (pivotRow != pivot)
                std::swap(aug[pivot], aug[pivotRow]);

            float pivotVal = aug[pivot][pivot];
            if (std::abs(pivotVal) < 1e-8f)
                return Identity();

            float invPivot = 1.0f / pivotVal;
            for (int col = 0; col < 8; ++col)
                aug[pivot][col] *= invPivot;

            for (int row = 0; row < 4; ++row)
            {
                if (row == pivot)
                    continue;
                float factor = aug[row][pivot];
                for (int col = 0; col < 8; ++col)
                    aug[row][col] -= factor * aug[pivot][col];
            }
        }

        Mat4 result;
        for (int row = 0; row < 4; ++row)
            for (int col = 0; col < 4; ++col)
                Set(result, row, col, aug[row][4 + col]);
        return result;
    }

    Mat4 Compose(const Vec3& translation, const Quat& rotation, float scale)
    {
        Quat q = Normalize(rotation);
        float xx = q.x * q.x, yy = q.y * q.y, zz = q.z * q.z;
        float xy = q.x * q.y, xz = q.x * q.z, yz = q.y * q.z;
        float wx = q.w * q.x, wy = q.w * q.y, wz = q.w * q.z;

        Mat4 result;
        Set(result, 0, 0, (1.0f - 2.0f * (yy + zz)) * scale);
        Set(result, 1, 0, (2.0f * (xy + wz)) * scale);
        Set(result, 2, 0, (2.0f * (xz - wy)) * scale);
        Set(result, 3, 0, 0.0f);

        Set(result, 0, 1, (2.0f * (xy - wz)) * scale);
        Set(result, 1, 1, (1.0f - 2.0f * (xx + zz)) * scale);
        Set(result, 2, 1, (2.0f * (yz + wx)) * scale);
        Set(result, 3, 1, 0.0f);

        Set(result, 0, 2, (2.0f * (xz + wy)) * scale);
        Set(result, 1, 2, (2.0f * (yz - wx)) * scale);
        Set(result, 2, 2, (1.0f - 2.0f * (xx + yy)) * scale);
        Set(result, 3, 2, 0.0f);

        Set(result, 0, 3, translation.x);
        Set(result, 1, 3, translation.y);
        Set(result, 2, 3, translation.z);
        Set(result, 3, 3, 1.0f);

        return result;
    }

    // Ken Shoemake's branch-on-trace matrix->quaternion conversion: picking the
    // largest denominator avoids dividing by a near-zero term near 180 degree
    // rotations. Scale is averaged across the three columns since VQS only
    // carries a single uniform scale factor.
    void Decompose(const Mat4& m, Vec3& outTranslation, Quat& outRotation, float& outScale)
    {
        outTranslation = {At(m, 0, 3), At(m, 1, 3), At(m, 2, 3)};

        Vec3 col0{At(m, 0, 0), At(m, 1, 0), At(m, 2, 0)};
        Vec3 col1{At(m, 0, 1), At(m, 1, 1), At(m, 2, 1)};
        Vec3 col2{At(m, 0, 2), At(m, 1, 2), At(m, 2, 2)};

        float sx = Length(col0);
        float sy = Length(col1);
        float sz = Length(col2);
        outScale = (sx + sy + sz) / 3.0f;

        if (sx < 1e-8f || sy < 1e-8f || sz < 1e-8f)
        {
            outRotation = {0.0f, 0.0f, 0.0f, 1.0f};
            return;
        }

        float r00 = col0.x / sx, r10 = col0.y / sx, r20 = col0.z / sx;
        float r01 = col1.x / sy, r11 = col1.y / sy, r21 = col1.z / sy;
        float r02 = col2.x / sz, r12 = col2.y / sz, r22 = col2.z / sz;

        float trace = r00 + r11 + r22;
        Quat q;
        if (trace > 0.0f)
        {
            float s = std::sqrt(trace + 1.0f) * 2.0f;
            q.w = 0.25f * s;
            q.x = (r21 - r12) / s;
            q.y = (r02 - r20) / s;
            q.z = (r10 - r01) / s;
        }
        else if (r00 > r11 && r00 > r22)
        {
            float s = std::sqrt(1.0f + r00 - r11 - r22) * 2.0f;
            q.w = (r21 - r12) / s;
            q.x = 0.25f * s;
            q.y = (r01 + r10) / s;
            q.z = (r02 + r20) / s;
        }
        else if (r11 > r22)
        {
            float s = std::sqrt(1.0f + r11 - r00 - r22) * 2.0f;
            q.w = (r02 - r20) / s;
            q.x = (r01 + r10) / s;
            q.y = 0.25f * s;
            q.z = (r12 + r21) / s;
        }
        else
        {
            float s = std::sqrt(1.0f + r22 - r00 - r11) * 2.0f;
            q.w = (r10 - r01) / s;
            q.x = (r02 + r20) / s;
            q.y = (r12 + r21) / s;
            q.z = 0.25f * s;
        }
        outRotation = Normalize(q);
    }

    Vec3 TransformPoint(const Mat4& m, const Vec3& v)
    {
        return {
            At(m, 0, 0) * v.x + At(m, 0, 1) * v.y + At(m, 0, 2) * v.z + At(m, 0, 3),
            At(m, 1, 0) * v.x + At(m, 1, 1) * v.y + At(m, 1, 2) * v.z + At(m, 1, 3),
            At(m, 2, 0) * v.x + At(m, 2, 1) * v.y + At(m, 2, 2) * v.z + At(m, 2, 3),
        };
    }
}
