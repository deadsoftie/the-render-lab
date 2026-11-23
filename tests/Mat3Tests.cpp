#include "MathTests.h"
#include "TestFramework.h"
#include "anim/Math/Mat3.h"

using namespace Anim;

namespace
{
    bool NearVec3(const Vec3& a, const Vec3& b, float eps)
    {
        return std::abs(a.x - b.x) <= eps && std::abs(a.y - b.y) <= eps &&
               std::abs(a.z - b.z) <= eps;
    }

    bool NearMat3(const Mat3& a, const Mat3& b, float eps)
    {
        for (int i = 0; i < 9; ++i)
            if (std::abs(a.m[i] - b.m[i]) > eps)
                return false;
        return true;
    }

    Mat3 RotationFromQuat(const Quat& q)
    {
        return FromColumns(Rotate(q, Vec3{1, 0, 0}), Rotate(q, Vec3{0, 1, 0}), Rotate(q, Vec3{0, 0, 1}));
    }
}

void RunMat3Tests()
{
    Mat3 id;
    CHECK(NearMat3(id * id, id, 1e-6f));
    CHECK(NearMat3(Inverse(id), id, 1e-6f));
    CHECK_NEAR(Determinant(id), 1.0f, 1e-6f);

    Mat3 a = FromColumns(Vec3{2, 1, 0}, Vec3{-1, 3, 2}, Vec3{0.5f, 0, 4});
    CHECK_NEAR(At(a, 0, 1), -1.0f, 1e-6f);
    CHECK_NEAR(At(a, 2, 1), 2.0f, 1e-6f);
    CHECK(NearVec3(a * Vec3{1, 0, 0}, Vec3{2, 1, 0}, 1e-6f));
    CHECK(NearMat3(a * Inverse(a), id, 1e-5f));
    CHECK(NearMat3(Inverse(a) * a, id, 1e-5f));
    CHECK(NearMat3(Transpose(Transpose(a)), a, 1e-6f));
    CHECK_NEAR(Determinant(Transpose(a)), Determinant(a), 1e-4f);

    Mat3 b = FromColumns(Vec3{1, -2, 0.5f}, Vec3{0, 1, 3}, Vec3{2, 0, -1});
    Vec3 v{0.3f, -1.2f, 2.5f};
    CHECK(NearVec3((a * b) * v, a * (b * v), 1e-4f));
    CHECK(NearMat3(Transpose(a * b), Transpose(b) * Transpose(a), 1e-5f));

    Vec3 w{0.7f, -1.3f, 2.1f};
    Vec3 u{-0.4f, 2.2f, 0.9f};
    CHECK(NearVec3(Skew(w) * u, Cross(w, u), 1e-5f));
    CHECK(NearMat3(Transpose(Skew(w)), Skew(w) * -1.0f, 1e-6f));

    CHECK(NearVec3(Outer(w, u) * v, w * Dot(u, v), 1e-5f));
    CHECK(NearMat3(Diagonal(Vec3{2, 3, 4}) * Inverse(Diagonal(Vec3{2, 3, 4})), id, 1e-6f));
    Mat3 small = Diagonal(Vec3{2.7e-5f, 3.1e-5f, 2.2e-5f});
    CHECK(NearMat3(small * Inverse(small), id, 1e-5f));
    CHECK(NearMat3(Inverse(FromColumns(Vec3{1, 2, 3}, Vec3{2, 4, 6}, Vec3{0, 1, 0})), id, 0.0f));

    Mat3 rot = RotationFromQuat(FromAxisAngle(Normalize(Vec3{1, 2, -1}), 0.9f));
    CHECK(NearMat3(Transpose(rot) * rot, id, 1e-5f));
    CHECK(NearMat3(Orthonormalize(rot), rot, 1e-5f));

    CHECK(NearVec3(Unskew(Skew(w)), w, 1e-6f));
    CHECK(NearMat3(RotationFromVector(Normalize(Vec3{1, 2, -1}) * 0.9f), rot, 1e-5f));
    CHECK(NearMat3(RotationFromVector(Vec3{}), id, 0.0f));
    Mat3 expRot = RotationFromVector(w);
    CHECK(NearMat3(Transpose(expRot) * expRot, id, 1e-5f));
    CHECK(NearVec3(expRot * w, w, 1e-5f));

    Mat3 drifted = rot + Skew(w) * rot * 0.01f + Mat3{} * 0.01f;
    Mat3 fixedRot = Orthonormalize(drifted);
    CHECK(NearMat3(Transpose(fixedRot) * fixedRot, id, 1e-5f));
    CHECK_NEAR(Determinant(fixedRot), 1.0f, 1e-5f);
    CHECK(NearMat3(fixedRot, rot, 0.05f));

    Mat4 m = ToMat4(rot, Vec3{1, -2, 3});
    Vec3 p{0.5f, 1.5f, -2.0f};
    CHECK(NearVec3(TransformPoint(m, p), rot * p + Vec3{1, -2, 3}, 1e-5f));
}
