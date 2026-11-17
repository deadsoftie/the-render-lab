#include "anim/Math/Easing.h"

#include <algorithm>

namespace Anim
{
    float Smoothstep(float t)
    {
        t = std::clamp(t, 0.0f, 1.0f);
        return t * t * (3.0f - 2.0f * t);
    }
}  // namespace Anim
