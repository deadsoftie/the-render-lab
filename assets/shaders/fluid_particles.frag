#version 330 core

in vec3 vViewCenter;

layout(location=0) out vec4 gWorldPos;
layout(location=1) out vec4 gNormal;
layout(location=2) out vec4 gKd;
layout(location=3) out vec4 gKsAlpha;

uniform mat4 uProj;
uniform mat4 uInvView;
uniform float uRadius;
uniform vec3 uColor;

const vec3 kWaterF0 = vec3(0.02);
const float kRoughness = 0.1;

void main()
{
    vec2 d = gl_PointCoord * 2.0 - 1.0;
    d.y = -d.y;
    float r2 = dot(d, d);
    if (r2 > 1.0)
        discard;

    vec3 viewNormal = vec3(d, sqrt(1.0 - r2));
    vec3 viewPos = vViewCenter + viewNormal * uRadius;
    vec4 clip = uProj * vec4(viewPos, 1.0);
    gl_FragDepth = clip.z / clip.w * 0.5 + 0.5;

    gWorldPos = vec4((uInvView * vec4(viewPos, 1.0)).xyz, 1.0);
    gNormal   = vec4(normalize(mat3(uInvView) * viewNormal), 1.0);
    gKd       = vec4(uColor, 0.0);
    gKsAlpha  = vec4(kWaterF0, kRoughness);
}
