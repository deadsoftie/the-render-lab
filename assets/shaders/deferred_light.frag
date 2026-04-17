#version 330 core
#include "brdf.glsl"
#include "shadows.glsl"

in vec2 vUV;
out vec4 FragColor;

uniform sampler2D uWorldPosTex;
uniform sampler2D uNormalTex;
uniform sampler2D uKdTex;
uniform sampler2D uKsAlphaTex;

uniform vec3 uCamPos;

#define MAX_LIGHTS 5
uniform int   uLightCount;
uniform vec3  uLightPos[MAX_LIGHTS];
uniform vec3  uLightColor[MAX_LIGHTS];
uniform float uLightRange[MAX_LIGHTS];

uniform float uAmbient  = 0.02;
uniform float uExposure = 1.0;

uniform sampler2D uAOTex;
uniform int       uAOEnabled  = 0;
uniform float     uAOStrength = 1.0;

uniform int uToonEnabled = 0;
uniform int uToonBands   = 3;

// Shadow maps — one cube map per light (texture units 4..8)
uniform samplerCube uShadowMaps[5];
uniform int         uShadowsEnabled;
uniform float       uShadowFarPlane[5];
uniform float       uShadowBias      = 0.04;
uniform float       uShadowPcfRadius = 0.05;

// Moment shadow maps (texture units 9..13)
uniform samplerCube uMSMaps[5];
uniform int         uUseMSM;
uniform float       uMSMAlpha    = 0.001;
uniform float       uMSMFarPlane[5];

float ShadowPCF(int lightIdx, vec3 worldPos, vec3 lightPos, float farPlane)
{
    vec3  dir         = worldPos - lightPos;
    float currentDist = length(dir);
    float shadow      = 0.0;
    for (int s = 0; s < 12; ++s)
    {
        float closest = texture(uShadowMaps[lightIdx],
                                dir + kPcfDirs[s] * uShadowPcfRadius).r * farPlane;
        shadow += (currentDist - uShadowBias > closest) ? 1.0 : 0.0;
    }
    return shadow / 12.0;
}

uniform int   uDebugView       = 0;
uniform int   uDebugLightIndex = 0;
uniform float uGlobeRadius     = 0.06;

vec3 TonemapVec3(vec3 x) { return x / (x + vec3(1.0)); }

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

    if (uDebugView == 1) { FragColor = vec4(TonemapVec3(abs(worldPos)), 1.0); return; }
    if (uDebugView == 2) { FragColor = vec4(N * 0.5 + 0.5, 1.0); return; }
    if (uDebugView == 3) { FragColor = vec4(Kd, 1.0); return; }
    if (uDebugView == 4) { FragColor = vec4(Ks, 1.0); return; }

    float ao = (uAOEnabled != 0) ? mix(1.0, texture(uAOTex, vUV).r, uAOStrength) : 1.0;

    vec3 V = normalize(uCamPos - worldPos);

    if (uDebugView == 5) { FragColor = vec4(V * 0.5 + 0.5, 1.0); return;}

    int count = clamp(uLightCount, 0, MAX_LIGHTS);
    int li = clamp(uDebugLightIndex, 0, max(count - 1, 0));

    if (uDebugView == 6)
    {
        vec3 outCol = vec3(0.0);
        float r = max(uGlobeRadius, 1e-6);

        for (int i = 0; i < count; ++i)
        {
            float d = length(worldPos - uLightPos[i]);

            // soft sphere edge for nicer look
            float s = 1.0 - smoothstep(r * 0.85, r, d);

            outCol += uLightColor[i] * s;
        }

        FragColor = vec4(clamp(outCol, 0.0, 1.0), 1.0);
        return;
    }

    if (uDebugView == 12 || uDebugView == 13 || uDebugView == 14)
    {
        float aoVis = texture(uAOTex, vUV).r;
        FragColor = vec4(aoVis, aoVis, aoVis, 1.0);
        return;
    }

    if (uDebugView == 7)
    {
        vec3 toL = uLightPos[li] - worldPos;
        float d  = length(toL);
        float r  = max(uLightRange[li], 1e-6);

        float att = Attenuation(d, r);

        // visualize as grayscale (tonemap helps if values get big)
        vec3 vis = TonemapVec3(vec3(att * 2.0)); // *2 just to brighten the debug
        FragColor = vec4(vis, 1.0);
        return;
    }

    if (uDebugView == 8)
    {
        // Show blurred MSM mean depth (moment b1 = z) for the selected light.
        // Direction is from light toward the fragment (same convention as the cubemap).
        vec3  dir     = worldPos - uLightPos[li];
        float zf      = length(dir) / max(uMSMFarPlane[li], 1e-6);
        vec4  moments = texture(uMSMaps[li], dir);
        // Display: mean depth as grayscale, tinted red where fragment is in shadow
        float meanZ   = moments.r;
        float inShadow = step(meanZ + 0.01, zf);  // rough binary shadow indicator
        FragColor = vec4(meanZ, meanZ * (1.0 - inShadow * 0.5), meanZ * (1.0 - inShadow), 1.0);
        return;
    }

    // Small ambient term keeps unlit surfaces from going fully black.
    // (IBL path uses deferred_ibl.frag instead.)
    vec3 color = uAmbient * Kd * ao;

    for (int i = 0; i < count; ++i)
    {
        vec3  toL = uLightPos[i] - worldPos;
        float d   = length(toL);
        vec3  L   = toL / max(d, 1e-6);

        float att = Attenuation(d, uLightRange[i]);
        if (att <= 0.0) continue;

        float shadowFactor = 0.0;
        if (uShadowsEnabled != 0)
        {
            if (uUseMSM != 0)
            {
                vec3 dir = worldPos - uLightPos[i];
                float zf = length(dir) / uMSMFarPlane[i];
                vec4 moments = texture(uMSMaps[i], dir);
                shadowFactor = MSMShadow(moments, zf, uMSMAlpha);
            }
            else
            {
                shadowFactor = ShadowPCF(i, worldPos, uLightPos[i], uShadowFarPlane[i]);
            }
        }

        vec3 brdfVal;
        if (uToonEnabled != 0)
        {
            float NdotL_raw  = max(dot(N, L), 0.0);
            float NdotL_toon = floor(NdotL_raw * float(uToonBands)) / float(uToonBands);
            vec3  H          = normalize(L + V);
            float NdotH      = max(dot(N, H), 0.0);
            float spec       = pow(NdotH, alpha);
            brdfVal = Kd * NdotL_toon + Ks * step(0.5, spec);
        }
        else
        {
            brdfVal = EvalBRDF(L, V, N, Kd, Ks, alpha);
        }
        color += brdfVal * uLightColor[i] * att * (1.0 - shadowFactor);
    }

    // Tone mapping: Reinhard + exposure + gamma to sRGB
    vec3 mapped = (uExposure * color) / (uExposure * color + vec3(1.0));
    FragColor = vec4(pow(max(mapped, vec3(0.0)), vec3(1.0 / 2.2)), 1.0);
}
