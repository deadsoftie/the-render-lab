#version 330 core
#include "brdf.glsl"

in vec3 vWorldPos;
in vec3 vWorldNrm;

out vec4 FragColor;

uniform vec3 uCamPos;

#define MAX_LIGHTS 5
uniform int  uLightCount;
uniform vec3 uLightPos[MAX_LIGHTS];
uniform vec3 uLightColor[MAX_LIGHTS];

uniform vec3  uAlbedo;
uniform vec3  uKs;
uniform float uAmbient;
uniform float uAlpha;

void main()
{
    vec3 N = normalize(vWorldNrm);
    vec3 V = normalize(uCamPos - vWorldPos);

    vec3 color = uAmbient * uAlbedo;

    int count = clamp(uLightCount, 0, MAX_LIGHTS);
    for (int i = 0; i < count; ++i)
    {
        vec3  toL = uLightPos[i] - vWorldPos;
        float d   = length(toL);
        vec3  L   = toL / max(d, 1e-6);

        vec3 brdfVal = EvalBRDF(L, V, N, uAlbedo, uKs, uAlpha);
        color += brdfVal * uLightColor[i];
    }

    FragColor = vec4(color, 1.0);
}
