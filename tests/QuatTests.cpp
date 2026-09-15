#include "MathTests.h"
#include "TestFramework.h"
#include "anim/Math/Quat.h"

using namespace Anim;

namespace
{
    constexpr float kPi = 3.14159265358979323846f;

    bool NearVec3(const Vec3& a, const Vec3& b, float eps)
    {
        return std::abs(a.x - b.x) <= eps && std::abs(a.y - b.y) <= eps &&
               std::abs(a.z - b.z) <= eps;
    }

    bool NearQuat(const Quat& a, const Quat& b, float eps)
    {
        return std::abs(a.x - b.x) <= eps && std::abs(a.y - b.y) <= eps &&
               std::abs(a.z - b.z) <= eps && std::abs(a.w - b.w) <= eps;
    }
}

void RunQuatTests()
{
    Quat identity{0, 0, 0, 1};
    Quat rotZ90 = FromAxisAngle(Vec3{0, 0, 1}, kPi * 0.5f);
    Quat rotZ180 = FromAxisAngle(Vec3{0, 0, 1}, kPi);

    CHECK(NearQuat(identity * rotZ90, rotZ90, 1e-6f));
    CHECK_NEAR(Length(rotZ90), 1.0f, 1e-6f);
    CHECK(NearQuat(Conjugate(rotZ90), Quat{0, 0, -rotZ90.z, rotZ90.w}, 1e-6f));

    CHECK(NearVec3(Rotate(rotZ90, Vec3{1, 0, 0}), Vec3{0, 1, 0}, 1e-5f));
    CHECK(NearVec3(Rotate(rotZ180, Vec3{1, 0, 0}), Vec3{-1, 0, 0}, 1e-5f));
    CHECK(NearVec3(Rotate(identity, Vec3{5, -2, 3}), Vec3{5, -2, 3}, 1e-6f));

    CHECK(NearQuat(Slerp(identity, rotZ90, 0.0f), identity, 1e-6f));
    CHECK(NearQuat(Slerp(identity, rotZ90, 1.0f), rotZ90, 1e-6f));
    CHECK(NearQuat(Slerp(identity, rotZ180, 0.5f), rotZ90, 1e-5f));

    CHECK(NearQuat(Lerp(identity, rotZ90, 0.0f), identity, 1e-6f));
    CHECK(NearQuat(Lerp(identity, rotZ90, 1.0f), rotZ90, 1e-6f));

    // eLerp is the exponential-map formulation of the same curve as Slerp.
    CHECK(NearQuat(ELerp(identity, rotZ180, 0.5f), Slerp(identity, rotZ180, 0.5f), 1e-4f));
    CHECK(NearQuat(ELerp(identity, rotZ90, 0.0f), identity, 1e-6f));
    CHECK(NearQuat(ELerp(identity, rotZ90, 1.0f), rotZ90, 1e-5f));

    Quat roundTrip = Exp(Log(rotZ90));
    CHECK(NearQuat(roundTrip, rotZ90, 1e-5f));

    // Shortest-path: negated target represents the same rotation, Slerp must
    // still take the short way rather than the long way around.
    Quat negRotZ90{-rotZ90.x, -rotZ90.y, -rotZ90.z, -rotZ90.w};
    CHECK(NearVec3(Rotate(Slerp(identity, negRotZ90, 0.5f), Vec3{1, 0, 0}),
                   Rotate(Slerp(identity, rotZ90, 0.5f), Vec3{1, 0, 0}), 1e-5f));

    int steps = 8;
    ISlerpSegment seg = PrepareISlerp(identity, rotZ180, steps);
    Quat q = identity;
    for (int i = 0; i < steps; ++i)
        q = StepISlerp(q, seg);
    CHECK(NearQuat(q, rotZ180, 1e-4f));

    q = identity;
    for (int i = 0; i < steps / 2; ++i)
        q = StepISlerp(q, seg);
    CHECK(NearQuat(q, Slerp(identity, rotZ180, 0.5f), 1e-4f));
}
