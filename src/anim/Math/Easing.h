#pragma once

namespace Anim
{
    // 3t^2 - 2t^3 with t clamped to [0, 1]: zero velocity at both ends. Shared by the clip
    // cross-fade weight, turn-on-the-spot easing and the IK reach weight.
    float Smoothstep(float t);
}  // namespace Anim
