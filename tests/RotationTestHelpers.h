#pragma once

#include <cmath>

#include "anim/Math/Quat.h"

namespace Test
{
    inline bool NearRotation(const Anim::Quat& a, const Anim::Quat& b, float epsilon)
    {
        auto near = [epsilon](const Anim::Quat& first, const Anim::Quat& second)
        {
            return std::abs(first.x - second.x) <= epsilon &&
                   std::abs(first.y - second.y) <= epsilon &&
                   std::abs(first.z - second.z) <= epsilon &&
                   std::abs(first.w - second.w) <= epsilon;
        };
        // Opposite unit quaternion signs encode the same rotation.
        return near(a, b) || near(a, -b);
    }
}
