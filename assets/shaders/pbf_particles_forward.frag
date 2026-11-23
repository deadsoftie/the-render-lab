#version 430 core
#include "brdf.glsl"

in vec3 vViewCenter;

out vec4 FragColor;

uniform mat4 uProj;
uniform mat4 uInvView;
uniform float uRadius;
uniform vec3 uColor;
uniform float uAmbient;

#define MAX_LIGHTS 5
uniform int uLightCount;
uniform vec3 uLightPos[MAX_LIGHTS];
uniform vec3 uLightColor[MAX_LIGHTS];

// Forward sphere impostor lit by the scene point lights.
void main()
{
    vec2 xy = vec2(gl_PointCoord.x * 2.0 - 1.0, 1.0 - gl_PointCoord.y * 2.0);
    float r2 = dot(xy, xy);
    if (r2 > 1.0)
        discard;

    vec3 n = vec3(xy, sqrt(1.0 - r2));
    vec3 viewPos = vViewCenter + n * uRadius;
    vec4 clip = uProj * vec4(viewPos, 1.0);
    gl_FragDepth = clip.z / clip.w * 0.5 + 0.5;

    vec3 worldPos = (uInvView * vec4(viewPos, 1.0)).xyz;
    vec3 N = normalize(mat3(uInvView) * n);
    vec3 V = normalize(uInvView[3].xyz - worldPos);

    vec3 color = uAmbient * uColor;
    int count = clamp(uLightCount, 0, MAX_LIGHTS);
    for (int i = 0; i < count; ++i)
    {
        vec3 toL = uLightPos[i] - worldPos;
        vec3 L = toL / max(length(toL), 1e-6);
        color += EvalBRDF(L, V, N, uColor, vec3(0.02), 0.15) * uLightColor[i];
    }
    FragColor = vec4(color, 1.0);
}
