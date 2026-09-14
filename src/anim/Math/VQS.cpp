#include "pch.h"
#include "anim/Math/VQS.h"

namespace Anim
{
    VQS Concat(const VQS& parent, const VQS& child)
    {
        VQS result;
        result.v = parent.v + Rotate(parent.q, child.v) * parent.s;
        result.q = Normalize(parent.q * child.q);
        result.s = parent.s * child.s;
        return result;
    }

    Mat4 ToMat4(const VQS& vqs)
    {
        return Compose(vqs.v, vqs.q, vqs.s);
    }

    Vec3 Transform(const VQS& vqs, const Vec3& p)
    {
        return vqs.v + Rotate(vqs.q, p) * vqs.s;
    }

    VQS Lerp(const VQS& a, const VQS& b, float t)
    {
        return {Lerp(a.v, b.v, t), Lerp(a.q, b.q, t), a.s + (b.s - a.s) * t};
    }

    VQS Slerp(const VQS& a, const VQS& b, float t)
    {
        return {Lerp(a.v, b.v, t), Slerp(a.q, b.q, t), a.s + (b.s - a.s) * t};
    }

    VQS ELerp(const VQS& a, const VQS& b, float t)
    {
        return {Lerp(a.v, b.v, t), ELerp(a.q, b.q, t), a.s + (b.s - a.s) * t};
    }

    IVQSSegment PrepareIVQS(const VQS& a, const VQS& b, int steps)
    {
        float invSteps = 1.0f / static_cast<float>(steps > 0 ? steps : 1);
        IVQSSegment segment;
        segment.dv = (b.v - a.v) * invSteps;
        segment.dq = PrepareISlerp(a.q, b.q, steps);
        segment.ds = (b.s - a.s) * invSteps;
        return segment;
    }

    VQS StepIVQS(const VQS& current, const IVQSSegment& segment)
    {
        return {current.v + segment.dv, StepISlerp(current.q, segment.dq), current.s + segment.ds};
    }
}
