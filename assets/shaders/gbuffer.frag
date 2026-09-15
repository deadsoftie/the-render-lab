#version 330 core
#include "brdf.glsl"

in vec3 vWorldPos;
in vec3 vWorldNrm;
in vec2 vUV;
in vec3 vWorldTangent;

layout(location=0) out vec4 gWorldPos;   // xyz
layout(location=1) out vec4 gNormal;     // xyz
layout(location=2) out vec4 gKd;         // rgb = albedo, a = metallic
layout(location=3) out vec4 gKsAlpha;    // rgb = Ks (F0), a = roughness

uniform vec3  uKd;
uniform vec3  uKs;
uniform float uAlpha; // shininess, used when no roughness texture is bound
uniform float uMetallic;

uniform sampler2D uAlbedoTex;
uniform bool uHasAlbedoTex;
uniform sampler2D uSpecularTex;
uniform bool uHasSpecularTex;
uniform sampler2D uRoughnessTex;
uniform bool uHasRoughnessTex;
uniform sampler2D uMetallicTex;
uniform bool uHasMetallicTex;
uniform sampler2D uNormalTex;
uniform bool uHasNormalTex;

// Blend factor (0..1) between the scalar fallback and the texture sample, per channel.
// Only affects anything when the matching uHasXTex is true.
uniform float uAlbedoStrength    = 1.0;
uniform float uSpecularStrength  = 1.0;
uniform float uRoughnessStrength = 1.0;
uniform float uMetallicStrength  = 1.0;
uniform float uNormalStrength    = 1.0;

void main()
{
    vec3  albedo = uHasAlbedoTex
                 ? mix(uKd, texture(uAlbedoTex, vUV).rgb, uAlbedoStrength)
                 : uKd;
    vec3  Ks = uHasSpecularTex
             ? mix(uKs, texture(uSpecularTex, vUV).rgb, uSpecularStrength)
             : uKs;
    float roughnessFallback = PhongToRoughness(uAlpha);
    float roughness = uHasRoughnessTex
                     ? mix(roughnessFallback, texture(uRoughnessTex, vUV).r, uRoughnessStrength)
                     : roughnessFallback;
    float metallic = uHasMetallicTex
                    ? mix(uMetallic, texture(uMetallicTex, vUV).r, uMetallicStrength)
                    : uMetallic;

    vec3 N = normalize(vWorldNrm);
    if (uHasNormalTex)
    {
        vec3 T = normalize(vWorldTangent - N * dot(N, vWorldTangent));
        vec3 B = cross(N, T);
        vec3 nSample = texture(uNormalTex, vUV).rgb * 2.0 - 1.0;
        nSample = normalize(mix(vec3(0.0, 0.0, 1.0), nSample, uNormalStrength));
        N = normalize(mat3(T, B, N) * nSample);
    }

    gWorldPos = vec4(vWorldPos, 1.0);
    gNormal   = vec4(N, 1.0);
    gKd       = vec4(albedo, metallic);
    gKsAlpha  = vec4(Ks, max(roughness, 0.03));
}
