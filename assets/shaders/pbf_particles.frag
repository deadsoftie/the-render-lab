#version 430 core

in vec3 vViewCenter;

layout(location = 0) out vec4 gWorldPos;
layout(location = 1) out vec4 gNormal;
layout(location = 2) out vec4 gKd;
layout(location = 3) out vec4 gKsAlpha;

uniform mat4 uProj;
uniform mat4 uInvView;
uniform float uRadius;
uniform vec3 uColor;

// Sphere impostor into the G-buffer.
void main()
{
    vec2 xy = vec2(gl_PointCoord.x * 2.0 - 1.0, 1.0 - gl_PointCoord.y * 2.0);
    float r2 = dot(xy, xy);
    if (r2 > 1.0)
        discard;

    // Sphere surface point and its real depth.
    vec3 n = vec3(xy, sqrt(1.0 - r2));
    vec3 viewPos = vViewCenter + n * uRadius;
    vec4 clip = uProj * vec4(viewPos, 1.0);
    gl_FragDepth = clip.z / clip.w * 0.5 + 0.5;

    gWorldPos = vec4((uInvView * vec4(viewPos, 1.0)).xyz, 1.0);
    gNormal = vec4(normalize(mat3(uInvView) * n), 1.0);
    gKd = vec4(uColor, 0.0);
    gKsAlpha = vec4(vec3(0.02), 0.15);
}
