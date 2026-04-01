#version 330 core
#include "brdf.glsl"

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
#define MAX_LIGHTS 64
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

// ---- Tone mapping -----------------------------------------------------------
uniform float uExposure     = 1.0;
uniform float uHDRIRotation = 0.0; // yaw in radians, rotates the environment around Y

// ---- Debug ------------------------------------------------------------------
uniform int   uDebugView       = 0;
uniform int   uDebugLightIndex = 0;
uniform float uGlobeRadius     = 0.06;

// =============================================================================
// Shadow helpers (identical to deferred_light.frag)
// =============================================================================

const vec3 kPcfDirs[20] = vec3[](
    vec3( 1, 1, 1), vec3( 1,-1, 1), vec3(-1,-1, 1), vec3(-1, 1, 1),
    vec3( 1, 1,-1), vec3( 1,-1,-1), vec3(-1,-1,-1), vec3(-1, 1,-1),
    vec3( 1, 1, 0), vec3( 1,-1, 0), vec3(-1,-1, 0), vec3(-1, 1, 0),
    vec3( 1, 0, 1), vec3(-1, 0, 1), vec3( 1, 0,-1), vec3(-1, 0,-1),
    vec3( 0, 1, 1), vec3( 0,-1, 1), vec3( 0,-1,-1), vec3( 0, 1,-1)
);

float MSMShadow(vec4 b, float zf, float alpha)
{
    vec4 bp = mix(b, vec4(0.5, 0.25, 0.125, 0.0625), alpha);
    float bv = bp.x, c = bp.y;
    float d = sqrt(max(bp.y - bp.x*bp.x, 0.0));
    if (d < 1e-4) d = 1e-4;
    float e = (bp.z - bp.x*bp.y) / d;
    float f = sqrt(max(bp.w - bp.y*bp.y - e*e, 0.0));
    if (f < 1e-4) f = 1e-4;
    float ch2 = (zf - bv) / d;
    float ch3 = (zf*zf - c - e*ch2) / f;
    float c3 = ch3 / f;
    float c2 = (ch2 - e*c3) / d;
    float c1 = 1.0 - bv*c2 - c*c3;
    float disc   = max(c2*c2 - 4.0*c3*c1, 0.0);
    float sq     = sqrt(disc);
    float inv2c3 = 1.0 / (2.0*c3 + 1e-6);
    float z2 = (-c2 - sq) * inv2c3;
    float z3 = (-c2 + sq) * inv2c3;
    if (z2 > z3) { float t = z2; z2 = z3; z3 = t; }
    if (zf <= z2) return 0.0;
    if (zf <= z3)
        return clamp((zf*z3 - bp.x*(zf+z3) + bp.y) / ((z3-z2)*(zf-z2) + 1e-6), 0.0, 1.0);
    return clamp(1.0 - (z2*z3 - bp.x*(z2+z3) + bp.y) / ((zf-z2)*(zf-z3) + 1e-6), 0.0, 1.0);
}

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

float Attenuation(float d, float r)
{
    if (d >= r) return 0.0;
    float invd2 = 1.0 / max(d * d, 1e-6);
    float invr2 = 1.0 / max(r * r, 1e-6);
    float att   = max(invd2 - invr2, 0.0);
    float t     = clamp(1.0 - d / max(r, 1e-6), 0.0, 1.0);
    return att * t * t;
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
        vec4 clip     = vec4(vUV * 2.0 - 1.0, 1.0, 1.0);
        vec4 worldH   = uInvViewProj * clip;
        vec3 skyDir   = normalize(worldH.xyz / worldH.w - uCamPos);
        vec3 skyColor = texture(uHDRITex, uvOf(RotateY(skyDir, uHDRIRotation))).rgb;
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

    // =========================================================================
    // IBL Diffuse — irradiance map lookup by normal
    // =========================================================================
    vec3 irradiance = texture(uIrradianceTex, uvOf(RotateY(N, uHDRIRotation))).rgb;
    vec3 diffuseIBL = (Kd / PI) * irradiance;

    // =========================================================================
    // IBL Specular — GGX importance sampling (Monte Carlo, eq. 3 from spec)
    // =========================================================================
    float roughness = PhongToRoughness(alpha);

    // Reflection frame centred at R (spec steps 3-c)
    vec3 R = 2.0 * dot(N, V) * N - V;
    vec3 upVec = abs(R.z) < 0.999 ? vec3(0.0, 0.0, 1.0) : vec3(1.0, 0.0, 0.0);
    vec3 A = normalize(cross(upVec, R));
    vec3 B = cross(R, A);

    int  N_samples = clamp(uIBLSamples, 1, 100);
    vec3 specularSum = vec3(0.0);

    for (int k = 0; k < N_samples; ++k)
    {
        vec2 xi = uHammersley[k];

        // GGX importance sampling: theta of the half-vector
        float theta  = atan(roughness * sqrt(xi.y) / sqrt(max(1.0 - xi.y, 1e-5)));
        vec3  D      = vectorOf(xi.x, theta / PI);

        // Rotate D from Z-aligned frame to reflection frame → light direction omega_k
        vec3  L      = normalize(D.x * A + D.y * B + D.z * R);
        float NdotL  = max(dot(N, L), 0.0);
        if (NdotL <= 0.0) continue;

        // Half-vector for BRDF evaluation
        vec3  H      = normalize(L + V);
        float NdotH  = max(dot(N, H), 0.0);
        float LdotH  = max(dot(L, H), 0.0);

        // MIP level (spec equation, adjusted second term per debugging hint)
        float D_H  = D_GGX(NdotH, roughness);
        float mip  = 0.5 * log2(float(uHDRIWidth * uHDRIHeight) / float(N_samples))
                   - 0.5 * log2(max(D_H / 4.0, 1e-5)) - 1.0;
        mip = max(mip, 0.0);

        vec3 Li = textureLod(uHDRITex, uvOf(RotateY(L, uHDRIRotation)), mip).rgb;

        // Monte Carlo estimator eq (3): Li * NdotL * G * F / (4 * NdotL * NdotV)
        // NdotL cancels → Li * G * F / (4 * NdotV)
        float G  = G_Smith(NdotL, NdotV, roughness);
        vec3  F  = F_Schlick(F0, LdotH);
        specularSum += Li * G * F / max(4.0 * NdotV, 1e-5);
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
    // Combine and tone map
    // =========================================================================
    vec3 color = diffuseIBL + specularIBL + directLight;
    FragColor  = vec4(ToneMap(color), 1.0);
}
