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

    vec3 color = uAmbient * Kd;

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
