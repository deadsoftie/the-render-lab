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
uniform float uLightRange[MAX_LIGHTS];

uniform float uAmbient = 0.02;

// Shadow maps — one cube map per light (texture units 4..8)
uniform samplerCube uShadowMaps[5];
uniform int         uShadowsEnabled;
uniform float       uShadowFarPlane[5];
uniform float       uShadowBias      = 0.04;
uniform float       uShadowPcfRadius = 0.05;

// Moment shadow maps (texture units 9..13)
uniform samplerCube uMSMaps[5];
uniform int         uUseMSM;
uniform float       uMSMAlpha = 0.001;
uniform float       uMSMFarPlane[5];

// 20-sample offset kernel for PCF
const vec3 kPcfDirs[20] = vec3[](
    vec3( 1, 1, 1), vec3( 1,-1, 1), vec3(-1,-1, 1), vec3(-1, 1, 1),
    vec3( 1, 1,-1), vec3( 1,-1,-1), vec3(-1,-1,-1), vec3(-1, 1,-1),
    vec3( 1, 1, 0), vec3( 1,-1, 0), vec3(-1,-1, 0), vec3(-1, 1, 0),
    vec3( 1, 0, 1), vec3(-1, 0, 1), vec3( 1, 0,-1), vec3(-1, 0,-1),
    vec3( 0, 1, 1), vec3( 0,-1, 1), vec3( 0,-1,-1), vec3( 0, 1,-1)
);

// Hamburger 4-Moment Shadow Mapping
// b: moments (z, z^2, z^3, z^4), zf: normalized fragment depth, alpha: light-leak bias
float MSMShadow(vec4 b, float zf, float alpha)
{
    // Bias moments toward uniform distribution to suppress light leaking
    vec4 bp = mix(b, vec4(0.5), alpha);

    // Cholesky decomposition of 3x3 Hankel moment matrix
    float bv = bp.x, c = bp.y;
    float d = sqrt(max(bp.y - bp.x*bp.x, 0.0));
    if (d < 1e-4) d = 1e-4;
    float e = (bp.z - bp.x*bp.y) / d;
    float f = sqrt(max(bp.w - bp.y*bp.y - e*e, 0.0));
    if (f < 1e-4) f = 1e-4;

    // Forward substitution: solve L * y = [1, zf, zf^2]
    float ch2 = (zf - bv) / d;
    float ch3 = (zf*zf - c - e*ch2) / f;

    // Back substitution: solve L^T * c_vec = y
    float c3 = ch3 / f;
    float c2 = (ch2 - e*c3) / d;
    float c1 = 1.0 - bv*c2 - c*c3;

    // Roots of quadratic c1 + c2*z + c3*z^2 = 0
    float disc   = max(c2*c2 - 4.0*c3*c1, 0.0);
    float sq     = sqrt(disc);
    float inv2c3 = 1.0 / (2.0*c3 + 1e-6);
    float z2 = (-c2 - sq) * inv2c3;
    float z3 = (-c2 + sq) * inv2c3;
    if (z2 > z3) { float t = z2; z2 = z3; z3 = t; }

    // Shadow factor from moment statistics
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

// 0 Final, 1 WorldPos, 2 Normal, 3 Kd, 4 KsAlpha, 5 EyeVec, 6 LightGlobes, 7 Brightness
uniform int uDebugView = 0;

// which light to visualize in some debug views
uniform int   uDebugLightIndex = 0;

// how big the globe sphere is in world units
uniform float uGlobeRadius = 0.06;

vec3 TonemapVec3(vec3 x)
{
    // quick and dirty for visualizing HDR-ish buffers
    return x / (x + vec3(1.0));
}

float Attenuation(float d, float r)
{
    if (d >= r) return 0.0;

    float invd2 = 1.0 / max(d*d, 1e-6);
    float invr2 = 1.0 / max(r*r, 1e-6);
    float att   = max(invd2 - invr2, 0.0);

    // smooth-edges to cutoff artefacts
    float t = clamp(1.0 - d / max(r, 1e-6), 0.0, 1.0);
    att *= t * t;

    return att;
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

    if (uDebugView == 1) { FragColor = vec4(TonemapVec3(abs(worldPos)), 1.0); return; }
    if (uDebugView == 2) { FragColor = vec4(N * 0.5 + 0.5, 1.0); return; }
    if (uDebugView == 3) { FragColor = vec4(Kd, 1.0); return; }
    if (uDebugView == 4) { FragColor = vec4(Ks, 1.0); return; }

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

    vec3 color = uAmbient * Kd;

    for (int i = 0; i < count; ++i)
    {
        vec3 L = normalize(uLightPos[i] - worldPos);

        float ndotl = max(dot(N, L), 0.0);
        vec3 diffuse = Kd * ndotl;

        vec3 H = normalize(L + V);
        float specPow = pow(max(dot(N, H), 0.0), alpha);
        vec3 spec = specPow * uLightColor[i];

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

        color += (diffuse * uLightColor[i] + spec) * (1.0 - shadowFactor);
    }

    FragColor = vec4(color, 1.0);
}
