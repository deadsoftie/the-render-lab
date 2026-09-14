#include "pch.h"
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

    Vec3 TransformPoint(const Mat4& m, const Vec3& v)
    {
        return {
            At(m, 0, 0) * v.x + At(m, 0, 1) * v.y + At(m, 0, 2) * v.z + At(m, 0, 3),
            At(m, 1, 0) * v.x + At(m, 1, 1) * v.y + At(m, 1, 2) * v.z + At(m, 1, 3),
            At(m, 2, 0) * v.x + At(m, 2, 1) * v.y + At(m, 2, 2) * v.z + At(m, 2, 3),
        };
    }
}
