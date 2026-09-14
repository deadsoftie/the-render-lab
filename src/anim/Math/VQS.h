#pragma once
#include "anim/Math/Mat4.h"
#include "anim/Math/Quat.h"
#include "anim/Math/Vec3.h"

namespace Anim
{
    struct VQS
    {
        Vec3 v;
        Quat q;
        float s = 1.0f;
    };

    // Hierarchical composition: applies child's local VQS inside parent's frame.
    VQS Concat(const VQS& parent, const VQS& child);
    Mat4 ToMat4(const VQS& vqs);
    Vec3 Transform(const VQS& vqs, const Vec3& p);

    VQS Lerp(const VQS& a, const VQS& b, float t);
    VQS Slerp(const VQS& a, const VQS& b, float t);
    VQS ELerp(const VQS& a, const VQS& b, float t);

    // Incremental VQS: bakes per-step deltas once per keyframe segment, then
    // StepIVQS advances translation/scale linearly and rotation via iSlerp,
    // one cheap step per frame instead of re-evaluating from segment-local time.
    struct IVQSSegment
    {
        Vec3 dv;
        ISlerpSegment dq;
        float ds = 0.0f;
    };

    IVQSSegment PrepareIVQS(const VQS& a, const VQS& b, int steps);
    VQS StepIVQS(const VQS& current, const IVQSSegment& segment);
}
