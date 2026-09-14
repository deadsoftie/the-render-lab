#include "MathTests.h"
#include "TestFramework.h"
#include "anim/Math/Vec3.h"

using namespace Anim;

namespace
{
    bool NearVec3(const Vec3& a, const Vec3& b, float eps)
    {
        return std::abs(a.x - b.x) <= eps && std::abs(a.y - b.y) <= eps &&
               std::abs(a.z - b.z) <= eps;
    }
}

void RunVec3Tests()
{
    Vec3 a{1.0f, 2.0f, 3.0f};
    Vec3 b{4.0f, -1.0f, 0.5f};

    CHECK(NearVec3(a + b, Vec3{5.0f, 1.0f, 3.5f}, 1e-6f));
    CHECK(NearVec3(a - b, Vec3{-3.0f, 3.0f, 2.5f}, 1e-6f));
    CHECK(NearVec3(-a, Vec3{-1.0f, -2.0f, -3.0f}, 1e-6f));
    CHECK(NearVec3(a * 2.0f, Vec3{2.0f, 4.0f, 6.0f}, 1e-6f));
    CHECK(NearVec3(2.0f * a, a * 2.0f, 1e-6f));

    CHECK_NEAR(Dot(Vec3{1, 0, 0}, Vec3{0, 1, 0}), 0.0f, 1e-6f);
    CHECK_NEAR(Dot(Vec3{1, 2, 3}, Vec3{4, 5, 6}), 32.0f, 1e-6f);

    CHECK(NearVec3(Cross(Vec3{1, 0, 0}, Vec3{0, 1, 0}), Vec3{0, 0, 1}, 1e-6f));
    CHECK(NearVec3(Cross(Vec3{0, 1, 0}, Vec3{1, 0, 0}), Vec3{0, 0, -1}, 1e-6f));

    CHECK_NEAR(Length(Vec3{3, 4, 0}), 5.0f, 1e-6f);
    CHECK(NearVec3(Normalize(Vec3{0, 0, 0}), Vec3{0, 0, 0}, 1e-6f));
    CHECK_NEAR(Length(Normalize(Vec3{3, 4, 0})), 1.0f, 1e-6f);

    CHECK(NearVec3(Lerp(a, b, 0.0f), a, 1e-6f));
    CHECK(NearVec3(Lerp(a, b, 1.0f), b, 1e-6f));
    CHECK(NearVec3(Lerp(a, b, 0.5f), (a + b) * 0.5f, 1e-6f));
}
