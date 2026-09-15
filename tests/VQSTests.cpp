#include "MathTests.h"
#include "TestFramework.h"
#include "anim/Math/VQS.h"

using namespace Anim;

namespace
{
    constexpr float kPi = 3.14159265358979323846f;

    bool NearVec3(const Vec3& a, const Vec3& b, float eps)
    {
        return std::abs(a.x - b.x) <= eps && std::abs(a.y - b.y) <= eps &&
               std::abs(a.z - b.z) <= eps;
    }

    bool NearVQS(const VQS& a, const VQS& b, float eps)
    {
        return NearVec3(a.v, b.v, eps) && std::abs(a.q.x - b.q.x) <= eps &&
               std::abs(a.q.y - b.q.y) <= eps && std::abs(a.q.z - b.q.z) <= eps &&
               std::abs(a.q.w - b.q.w) <= eps && std::abs(a.s - b.s) <= eps;
    }
}

void RunVQSTests()
{
    VQS identity{Vec3{0, 0, 0}, Quat{0, 0, 0, 1}, 1.0f};
    VQS child{Vec3{1, 0, 0}, FromAxisAngle(Vec3{0, 0, 1}, kPi * 0.5f), 2.0f};

    CHECK(NearVQS(Concat(identity, child), child, 1e-5f));
    CHECK(NearVQS(Concat(child, identity), child, 1e-5f));

    // Parent rotates 90 deg around Z and offsets by (5,0,0); child sits at local
    // (1,0,0). World position should be parent.v + parent.q.Rotate((1,0,0)) * parent.s.
    VQS parent{Vec3{5, 0, 0}, FromAxisAngle(Vec3{0, 0, 1}, kPi * 0.5f), 1.0f};
    VQS localChild{Vec3{1, 0, 0}, Quat{0, 0, 0, 1}, 1.0f};
    VQS world = Concat(parent, localChild);
    CHECK(NearVec3(world.v, Vec3{5, 1, 0}, 1e-5f));

    CHECK(NearVec3(Transform(identity, Vec3{2, 3, 4}), Vec3{2, 3, 4}, 1e-6f));
    Mat4 m = ToMat4(child);
    CHECK(NearVec3(Transform(child, Vec3{1, 2, 3}), TransformPoint(m, Vec3{1, 2, 3}), 1e-5f));

    VQS a{Vec3{0, 0, 0}, Quat{0, 0, 0, 1}, 1.0f};
    VQS b{Vec3{4, 2, 0}, FromAxisAngle(Vec3{0, 0, 1}, kPi), 3.0f};
    CHECK(NearVQS(Lerp(a, b, 0.0f), a, 1e-6f));
    CHECK(NearVQS(Lerp(a, b, 1.0f), b, 1e-6f));
    CHECK(NearVQS(Slerp(a, b, 0.0f), a, 1e-6f));
    CHECK(NearVQS(Slerp(a, b, 1.0f), b, 1e-6f));
    CHECK(NearVQS(ELerp(a, b, 0.0f), a, 1e-6f));
    CHECK(NearVQS(ELerp(a, b, 1.0f), b, 1e-5f));

    int steps = 10;
    IVQSSegment seg = PrepareIVQS(a, b, steps);
    VQS cur = a;
    for (int i = 0; i < steps; ++i)
        cur = StepIVQS(cur, seg);
    CHECK(NearVQS(cur, b, 1e-4f));
}
