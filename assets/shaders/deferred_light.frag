#version 330 core

in vec2 vUV;
out vec4 FragColor;

uniform sampler2D uWorldPosTex;
uniform sampler2D uNormalTex;
uniform sampler2D uKdTex;
uniform sampler2D uKsAlphaTex;

uniform vec3 uCamPos;

#define MAX_LIGHTS 64
uniform int  uLightCount;
uniform vec3 uLightPos[MAX_LIGHTS];
uniform vec3 uLightColor[MAX_LIGHTS];

uniform float uAmbient = 0.02;

// 0 Final, 1 WorldPos, 2 Normal, 3 Kd, 4 KsAlpha
uniform int uDebugView = 0;

vec3 TonemapVec3(vec3 x)
{
    // quick and dirty for visualizing HDR-ish buffers
    return x / (x + vec3(1.0));
}

void main()
{
    vec4 wp = texture(uWorldPosTex, vUV);
    if (wp.w < 0.5)
    {
        // no geometry here
        FragColor = vec4(0.0, 0.0, 0.0, 1.0);
        return;
    }
    vec3 worldPos = wp.xyz;
    vec3 N = normalize(texture(uNormalTex, vUV).xyz);
    vec3 Kd = texture(uKdTex, vUV).rgb;

    vec4 ksA = texture(uKsAlphaTex, vUV);
    vec3 Ks = ksA.rgb;
    float alpha = max(ksA.a, 1.0); // avoid alpha=0 edge cases


    if (uDebugView == 1)
    {
        // visualize world position (tonemapped)
        FragColor = vec4(TonemapVec3(abs(worldPos)), 1.0);
        return;
    }
    if (uDebugView == 2)
    {
        // normals in [0,1]
        FragColor = vec4(N * 0.5 + 0.5, 1.0);
        return;
    }
    if (uDebugView == 3)
    {
        FragColor = vec4(Kd, 1.0);
        return;
    }
    if (uDebugView == 4)
    {
        // show Ks in RGB and alpha remapped in A (not visible, but ok)
        FragColor = vec4(Ks, 1.0);
        return;
    }

    vec3 V = normalize(uCamPos - worldPos);
    vec3 color = uAmbient * Kd;

    int count = clamp(uLightCount, 0, MAX_LIGHTS);
    for (int i = 0; i < count; ++i)
    {
        vec3 L = normalize(uLightPos[i] - worldPos);

        float ndotl = max(dot(N, L), 0.0);
        vec3 diffuse = Kd * ndotl;

        vec3 H = normalize(L + V);
        float specPow = pow(max(dot(N, H), 0.0), alpha);
        vec3 spec = specPow * uLightColor[i];
        
        color += diffuse * uLightColor[i] + spec;
    }

    FragColor = vec4(color, 1.0);
}
