#include "MathTests.h"
#include "TestFramework.h"
#include "anim/Math/Mat4.h"

using namespace Anim;

namespace
{
    constexpr float kPi = 3.14159265358979323846f;

    bool NearVec3(const Vec3& a, const Vec3& b, float eps)
    {
        return std::abs(a.x - b.x) <= eps && std::abs(a.y - b.y) <= eps &&
               std::abs(a.z - b.z) <= eps;
    }

    bool NearMat4(const Mat4& a, const Mat4& b, float eps)
    {
        for (int i = 0; i < 16; ++i)
            if (std::abs(a.m[i] - b.m[i]) > eps)
                return false;
        return true;
    }
}

void RunMat4Tests()
{
    Mat4 id = Identity();
    CHECK(NearMat4(id * id, id, 1e-6f));
    CHECK(NearMat4(Inverse(id), id, 1e-6f));

    Mat4 translateOnly = Compose(Vec3{1, 2, 3}, Quat{0, 0, 0, 1}, 1.0f);
    CHECK(NearVec3(TransformPoint(translateOnly, Vec3{0, 0, 0}), Vec3{1, 2, 3}, 1e-6f));
    CHECK(NearVec3(TransformPoint(translateOnly, Vec3{1, 1, 1}), Vec3{2, 3, 4}, 1e-6f));

    Quat rotZ90 = FromAxisAngle(Vec3{0, 0, 1}, kPi * 0.5f);
    Mat4 rotated = Compose(Vec3{0, 0, 0}, rotZ90, 1.0f);
    CHECK(NearVec3(TransformPoint(rotated, Vec3{1, 0, 0}), Vec3{0, 1, 0}, 1e-5f));

    Mat4 scaled = Compose(Vec3{0, 0, 0}, Quat{0, 0, 0, 1}, 2.0f);
    CHECK(NearVec3(TransformPoint(scaled, Vec3{1, 1, 1}), Vec3{2, 2, 2}, 1e-6f));

    Mat4 trs = Compose(Vec3{3, -1, 2}, FromAxisAngle(Vec3{0, 1, 0}, 0.7f), 1.5f);
    Mat4 trsInv = Inverse(trs);
    CHECK(NearMat4(trs * trsInv, id, 1e-4f));
    CHECK(NearMat4(trsInv * trs, id, 1e-4f));

    Vec3 p{2, 5, -3};
    CHECK(NearVec3(TransformPoint(trsInv, TransformPoint(trs, p)), p, 1e-3f));

    Vec3 srcT{3, -1, 2};
    Quat srcQ = FromAxisAngle(Vec3{0, 1, 0}, 0.7f);
    float srcS = 1.5f;
    Mat4 composed = Compose(srcT, srcQ, srcS);

    Vec3 dT;
    Quat dQ;
    float dS;
    Decompose(composed, dT, dQ, dS);

    CHECK(NearVec3(dT, srcT, 1e-4f));
    CHECK_NEAR(dS, srcS, 1e-4f);
    CHECK(NearMat4(Compose(dT, dQ, dS), composed, 1e-4f));
}
