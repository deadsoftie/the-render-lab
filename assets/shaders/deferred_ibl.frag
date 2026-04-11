#version 330 core
#include "brdf.glsl"
#include "shadows.glsl"

in vec2 vUV;
out vec4 FragColor;

// ---- GBuffer ----------------------------------------------------------------
uniform sampler2D uWorldPosTex;
uniform sampler2D uNormalTex;
uniform sampler2D uKdTex;
uniform sampler2D uKsAlphaTex;

// ---- Camera -----------------------------------------------------------------
uniform vec3 uCamPos;
uniform mat4 uInvViewProj;

// ---- Direct lights ----------------------------------------------------------
#define MAX_LIGHTS 5
uniform int   uLightCount;
uniform vec3  uLightPos[MAX_LIGHTS];
uniform vec3  uLightColor[MAX_LIGHTS];
uniform float uLightRange[MAX_LIGHTS];

// ---- Shadow maps (PCF) — texture units 4..8 ---------------------------------
uniform samplerCube uShadowMaps[5];
uniform int         uShadowsEnabled;
uniform float       uShadowFarPlane[5];
uniform float       uShadowBias      = 0.04;
uniform float       uShadowPcfRadius = 0.05;

// ---- Moment shadow maps — texture units 9..13 -------------------------------
uniform samplerCube uMSMaps[5];
uniform int         uUseMSM;
uniform float       uMSMAlpha    = 0.001;
uniform float       uMSMFarPlane[5];

// ---- IBL textures -----------------------------------------------------------
uniform sampler2D uHDRITex;       // equirectangular environment map  (unit 14)
uniform sampler2D uIrradianceTex; // pre-baked irradiance map         (unit 15)
uniform int       uHDRIWidth;
uniform int       uHDRIHeight;

// ---- Importance sampling ----------------------------------------------------
uniform int  uIBLSamples;
uniform vec2 uHammersley[100];

// ---- Ambient occlusion ------------------------------------------------------
uniform sampler2D uAOTex;
uniform int       uAOEnabled = 0;

// ---- Tone mapping -----------------------------------------------------------
uniform float uExposure     = 1.0;
uniform float uHDRIRotation = 0.0;

// ---- Spherical harmonics irradiance (Part B) --------------------------------
// 9 pre-multiplied SH coefficients (bands 0-2), already convolved with the
// cosine lobe.  std140 pads vec3 → vec4; use .rgb to read.
layout(std140) uniform SHBlock {
    vec4 uSHCoeffs[9];
};
uniform int uUseSHIrradiance = 0;

// Reconstruct diffuse irradiance from 9 SH coefficients.
// N must be in world space (Y-up).  Coefficients are pre-multiplied by the
// Ramamoorthi & Hanrahan cosine-lobe convolution factors.
vec3 EvalSH(vec3 N)
{
    vec3 c = vec3(0.0);
    c += uSHCoeffs[0].rgb * 0.282095;
    c += uSHCoeffs[1].rgb * 0.488603 * N.z;
    c += uSHCoeffs[2].rgb * 0.488603 * N.y;
    c += uSHCoeffs[3].rgb * 0.488603 * N.x;
    c += uSHCoeffs[4].rgb * 1.092548 * N.x * N.z;
    c += uSHCoeffs[5].rgb * 1.092548 * N.y * N.z;
    c += uSHCoeffs[6].rgb * 0.315392 * (3.0 * N.z * N.z - 1.0);
    c += uSHCoeffs[7].rgb * 1.092548 * N.x * N.y;
    c += uSHCoeffs[8].rgb * 0.546274 * (N.x * N.x - N.y * N.y);
    return max(c, vec3(0.0));
}

// ---- Debug ------------------------------------------------------------------
uniform int   uDebugView       = 0;
uniform int   uDebugLightIndex = 0;
uniform float uGlobeRadius     = 0.06;

// =============================================================================
// PCF shadow (deferred multi-light variant — uses uShadowMaps array)
// =============================================================================
float ShadowPCF(int lightIdx, vec3 worldPos, vec3 lightPos, float farPlane)
{
    vec3  dir         = worldPos - lightPos;
    float currentDist = length(dir);
    float shadow      = 0.0;
    for (int s = 0; s < 20; ++s)
    {
        float closest = texture(uShadowMaps[lightIdx],
                                dir + kPcfDirs[s] * uShadowPcfRadius).r * farPlane;
        shadow += (currentDist - uShadowBias > closest) ? 1.0 : 0.0;
    }
    return shadow / 20.0;
}

// =============================================================================
// Sphere map helpers  —  Y-up convention
// =============================================================================

// Rotate direction around the Y axis by yaw radians.
vec3 RotateY(vec3 d, float yaw)
{
    float c = cos(yaw), s = sin(yaw);
    return vec3(d.x * c + d.z * s,
                d.y,
               -d.x * s + d.z * c);
}

// Direction → equirectangular UV  (Y is the vertical / polar axis)
//   u: longitude, wraps around Y  →  atan2(z, x) in XZ plane
//   v: latitude from top           →  acos(y)
vec2 uvOf(vec3 w)
{
    return vec2(0.5 - atan(w.z, w.x) / (2.0 * PI),
                acos(clamp(w.y, -1.0, 1.0)) / PI);
}

// Equirectangular UV → direction  (inverse of uvOf, Y-up)
vec3 vectorOf(float u, float v)
{
    float phi   = 2.0 * PI * (0.5 - u);
    float theta = PI * v;
    return vec3(cos(phi) * sin(theta),   // X
                cos(theta),              // Y  ← up axis
                sin(phi) * sin(theta));  // Z
}

// =============================================================================
// Tone mapping
// =============================================================================

vec3 ToneMap(vec3 c)
{
    vec3 mapped = (uExposure * c) / (uExposure * c + vec3(1.0));
    return pow(max(mapped, vec3(0.0)), vec3(1.0 / 2.2));
}

// Simple tonemap for debug GBuffer visualisation only.
vec3 TonemapVec3(vec3 x) { return x / (x + vec3(1.0)); }

// =============================================================================
// main
// =============================================================================

void main()
{
    vec4 wp = texture(uWorldPosTex, vUV);

    // -------------------------------------------------------------------------
    // Background / skydome — no geometry in this pixel
    // -------------------------------------------------------------------------
    if (wp.w < 0.5)
    {
        // Reconstruct world-space ray direction from NDC position
        vec4 clip   = vec4(vUV * 2.0 - 1.0, 1.0, 1.0);
        vec4 worldH = uInvViewProj * clip;
        vec3 skyDir = normalize(worldH.xyz / worldH.w - uCamPos);

        vec3 skyColor;
        if (uDebugView == 9)
            skyColor = texture(uIrradianceTex, uvOf(RotateY(skyDir, uHDRIRotation))).rgb;
        else
            skyColor = texture(uHDRITex, uvOf(RotateY(skyDir, uHDRIRotation))).rgb;

        FragColor = vec4(ToneMap(skyColor), 1.0);
        return;
    }

    // -------------------------------------------------------------------------
    // Geometry pixel — read GBuffer
    // -------------------------------------------------------------------------
    vec3  worldPos = wp.xyz;
    vec3  N        = normalize(texture(uNormalTex, vUV).xyz);
    vec3  Kd       = texture(uKdTex, vUV).rgb;
    vec4  ksA      = texture(uKsAlphaTex, vUV);
    vec3  F0       = ksA.rgb;
    float alpha    = max(ksA.a, 1.0);
    vec3  V        = normalize(uCamPos - worldPos);
    float NdotV    = max(dot(N, V), 1e-4);
    float ao       = (uAOEnabled != 0) ? texture(uAOTex, vUV).r : 1.0;

    // ---- Debug views (same indices as PBS shader) ---------------------------
    if (uDebugView == 1) { FragColor = vec4(TonemapVec3(abs(worldPos)), 1.0); return; }
    if (uDebugView == 2) { FragColor = vec4(N * 0.5 + 0.5, 1.0);             return; }
    if (uDebugView == 3) { FragColor = vec4(Kd, 1.0);                         return; }
    if (uDebugView == 4) { FragColor = vec4(F0, 1.0);                         return; }
    if (uDebugView == 5) { FragColor = vec4(V * 0.5 + 0.5, 1.0);             return; }

    int count = clamp(uLightCount, 0, MAX_LIGHTS);
    int li    = clamp(uDebugLightIndex, 0, max(count - 1, 0));

    if (uDebugView == 6)
    {
        vec3  outCol = vec3(0.0);
        float r      = max(uGlobeRadius, 1e-6);
        for (int i = 0; i < count; ++i)
        {
            float d = length(worldPos - uLightPos[i]);
            outCol += uLightColor[i] * (1.0 - smoothstep(r * 0.85, r, d));
        }
        FragColor = vec4(clamp(outCol, 0.0, 1.0), 1.0);
        return;
    }
    if (uDebugView == 7)
    {
        float d   = length(uLightPos[li] - worldPos);
        float att = Attenuation(d, max(uLightRange[li], 1e-6));
        FragColor = vec4(TonemapVec3(vec3(att * 2.0)), 1.0);
        return;
    }
    if (uDebugView == 8)
    {
        vec3  dir      = worldPos - uLightPos[li];
        float zf       = length(dir) / max(uMSMFarPlane[li], 1e-6);
        vec4  moments  = texture(uMSMaps[li], dir);
        float meanZ    = moments.r;
        float inShadow = step(meanZ + 0.01, zf);
        FragColor = vec4(meanZ, meanZ * (1.0 - inShadow * 0.5), meanZ * (1.0 - inShadow), 1.0);
        return;
    }
    if (uDebugView == 9)
    {
        vec3 Nrot9 = RotateY(N, uHDRIRotation);
        vec3 irr   = (uUseSHIrradiance != 0)
                   ? EvalSH(Nrot9)
                   : texture(uIrradianceTex, uvOf(Nrot9)).rgb;
        FragColor = vec4(ToneMap(irr), 1.0);
        return;
    }

    // =========================================================================
    // IBL Diffuse — irradiance map lookup OR spherical harmonics reconstruction
    // =========================================================================
    vec3 Nrot       = RotateY(N, uHDRIRotation);
    vec3 irradiance = (uUseSHIrradiance != 0)
                    ? EvalSH(Nrot)
                    : texture(uIrradianceTex, uvOf(Nrot)).rgb;
    vec3 diffuseIBL = (Kd / PI) * irradiance * ao;

    // =========================================================================
    // IBL Specular — GGX importance sampling with correct H→L derivation
    //
    // We importance-sample the GGX NDF to get a half-vector H, then derive
    // the incoming light direction L = reflect(-V, H).  After the PDF and
    // the 1/(4*NdotL*NdotV) BRDF denominator cancel with the Jacobian of
    // the H→L change of variables, each sample contributes:
    //
    //   Li(L) * G(NdotL, NdotV) * F(LdotH) * LdotH
    //   ─────────────────────────────────────────────
    //                 NdotV * NdotH
    // =========================================================================
    float roughness = PhongToRoughness(alpha);
    float a         = roughness * roughness;   // GGX α  (perceptual roughness²)
    float a2        = a * a;                   // GGX α² — must match D_GGX / G_Smith convention

    // Tangent frame around the surface normal N for half-vector sampling
    vec3 upN  = abs(N.y) < 0.999 ? vec3(0.0, 1.0, 0.0) : vec3(1.0, 0.0, 0.0);
    vec3 TanN = normalize(cross(upN, N));
    vec3 BitN = cross(N, TanN);

    int  N_samples   = clamp(uIBLSamples, 1, 100);
    vec3 specularSum = vec3(0.0);

    for (int k = 0; k < N_samples; ++k)
    {
        vec2 xi = uHammersley[k];

        // GGX NDF importance-sample: draw a half-vector H in tangent space
        float cosTheta_h = sqrt((1.0 - xi.y) / max(1.0 + (a2 - 1.0) * xi.y, 1e-5));
        float sinTheta_h = sqrt(max(1.0 - cosTheta_h * cosTheta_h, 0.0));
        float phi_h      = 2.0 * PI * xi.x;

        // Transform H to world space
        vec3 H = normalize(sinTheta_h * cos(phi_h) * TanN
                         + sinTheta_h * sin(phi_h) * BitN
                         + cosTheta_h             * N);

        // Derive light direction by reflecting V about H
        vec3  L     = normalize(2.0 * dot(V, H) * H - V);
        float NdotL = dot(N, L);
        if (NdotL <= 0.0) continue;

        float NdotH = max(dot(N, H), 0.0);
        float LdotH = max(dot(L, H), 0.0);

        // MIP level: derived from the GGX PDF and the texel solid angle
        float D_H     = D_GGX(NdotH, roughness);
        float pdf     = max(D_H * NdotH / (4.0 * max(LdotH, 1e-5)), 1e-5);
        float saTexel = 4.0 * PI / float(uHDRIWidth * uHDRIHeight);
        float mip     = max(0.5 * log2(1.0 / (float(N_samples) * pdf * saTexel)), 0.0);

        vec3 Li = textureLod(uHDRITex, uvOf(RotateY(L, uHDRIRotation)), mip).rgb;

        // Estimator after D and PDF cancellation: G * F * LdotH / (NdotV * NdotH)
        float G = G_Smith(NdotL, NdotV, roughness);
        vec3  F = F_Schlick(F0, LdotH);
        specularSum += Li * G * F * LdotH / max(NdotV * NdotH, 1e-5);
    }
    vec3 specularIBL = specularSum / float(N_samples);

    // =========================================================================
    // Direct lights (PBS BRDF, same as deferred_light.frag)
    // =========================================================================
    vec3 directLight = vec3(0.0);
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
                vec3  dir     = worldPos - uLightPos[i];
                float zf      = length(dir) / uMSMFarPlane[i];
                vec4  moments = texture(uMSMaps[i], dir);
                shadowFactor  = MSMShadow(moments, zf, uMSMAlpha);
            }
            else
            {
                shadowFactor = ShadowPCF(i, worldPos, uLightPos[i], uShadowFarPlane[i]);
            }
        }

        vec3 brdfVal = EvalBRDF(L, V, N, Kd, F0, alpha);
        directLight += brdfVal * uLightColor[i] * att * (1.0 - shadowFactor);
    }

    // =========================================================================
    // Isolation debug views (need IBL results computed above)
    // =========================================================================
    if (uDebugView == 10) { FragColor = vec4(ToneMap(diffuseIBL),  1.0); return; }
    if (uDebugView == 11) { FragColor = vec4(ToneMap(specularIBL), 1.0); return; }

    if (uDebugView == 12 || uDebugView == 13 || uDebugView == 14)
    {
        float aoVis = texture(uAOTex, vUV).r;
        FragColor = vec4(aoVis, aoVis, aoVis, 1.0);
        return;
    }

    // =========================================================================
    // Combine and tone map
    // =========================================================================
    vec3 color = diffuseIBL + specularIBL + directLight;
    FragColor  = vec4(ToneMap(color), 1.0);
}
