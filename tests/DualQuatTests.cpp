#include "MathTests.h"
#include "TestFramework.h"
#include "anim/Math/DualQuat.h"

using namespace Anim;

namespace
{
    constexpr float kPi = 3.14159265358979323846f;

    bool NearVec3(const Vec3& a, const Vec3& b, float eps)
    {
        return std::abs(a.x - b.x) <= eps && std::abs(a.y - b.y) <= eps &&
               std::abs(a.z - b.z) <= eps;
    }
}

void RunDualQuatTests()
{
    Vec3 t{2.0f, 3.0f, -1.0f};
    Quat q = FromAxisAngle(Vec3{0, 1, 0}, kPi * 0.5f);
    DualQuat dq = MakeDualQuat(t, q);

    CHECK(NearVec3(ExtractTranslation(dq), t, 1e-4f));

    Vec3 p{1.0f, 0.0f, 0.0f};
    Vec3 expected = t + Rotate(q, p);
    CHECK(NearVec3(TransformPoint(dq, p), expected, 1e-4f));

    DualQuat identity = MakeDualQuat(Vec3{0, 0, 0}, Quat{0, 0, 0, 1});
    CHECK(NearVec3(TransformPoint(identity, p), p, 1e-5f));

    // Rescale-then-renormalize must reproduce the same transform - what the shader's blend relies on.
    DualQuat rescaled = Normalize(dq * 5.0f);
    CHECK(NearVec3(TransformPoint(rescaled, p), expected, 1e-4f));

    // Two equal-weight influences with the same rotation should blend to that rotation exactly.
    DualQuat a = MakeDualQuat(Vec3{1, 0, 0}, FromAxisAngle(Vec3{0, 0, 1}, kPi * 0.25f));
    DualQuat blended = Normalize(a * 0.5f + a * 0.5f);
    CHECK(NearVec3(TransformPoint(blended, p), TransformPoint(a, p), 1e-4f));
}
