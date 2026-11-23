#version 330 core
#include "brdf.glsl"

in vec2 vUV;
out vec4 FragColor;

uniform sampler2D uSceneColor;
uniform sampler2D uEyeDepth;
uniform sampler2D uNormals;
uniform sampler2D uThickness;
uniform sampler2D uSceneDepth;
uniform sampler2D uHDRITex;

uniform vec2 uInvProjScale;
uniform mat4 uView;
uniform mat4 uInvView;
uniform mat4 uProj;
uniform vec3 uCamPos;

uniform int uHasEnv;
uniform float uHDRIRotation;
uniform float uExposure;

uniform vec3 uColor;
uniform float uAbsorption;
uniform float uScatter;
uniform float uRefraction;
uniform float uRoughness;
uniform vec3 uLightDir;
uniform vec3 uLightColor;

const float kF0 = 0.02;

vec3 RotateY(vec3 d, float yaw)
{
    float c = cos(yaw), s = sin(yaw);
    return vec3(d.x * c + d.z * s, d.y, -d.x * s + d.z * c);
}

vec2 uvOf(vec3 w)
{
    return vec2(0.5 - atan(w.z, w.x) / (2.0 * PI), acos(clamp(w.y, -1.0, 1.0)) / PI);
}

vec3 Reinhard(vec3 c)
{
    vec3 e = uExposure * max(c, vec3(0.0));
    return e / (e + vec3(1.0));
}

float WindowDepth(float eyeDepth)
{
    vec4 clip = uProj * vec4(0.0, 0.0, -eyeDepth, 1.0);
    return clip.z / clip.w * 0.5 + 0.5;
}

vec3 Environment(vec3 dir)
{
    if (uHasEnv != 0)
        return texture(uHDRITex, uvOf(RotateY(dir, uHDRIRotation))).rgb;
    return mix(vec3(0.35, 0.38, 0.42), vec3(0.55, 0.7, 0.95), clamp(dir.y * 0.5 + 0.5, 0.0, 1.0));
}

void main()
{
    float z = texture(uEyeDepth, vUV).r;
    if (z <= 0.0)
        discard;

    vec4 normalTex = texture(uNormals, vUV);
    vec3 N = normalTex.w > 0.0 ? normalize(normalTex.xyz) : vec3(0.0, 1.0, 0.0);
    vec2 ndc = vUV * 2.0 - 1.0;
    vec3 viewPos = vec3(ndc * uInvProjScale * z, -z);
    vec3 world = (uInvView * vec4(viewPos, 1.0)).xyz;
    vec3 V = normalize(uCamPos - world);
    float thickness = texture(uThickness, vUV).r;

    vec3 viewN = mat3(uView) * N;
    vec2 uv = clamp(vUV + viewN.xy * thickness * uRefraction, vec2(0.001), vec2(0.999));
    if (texture(uSceneDepth, uv).r < WindowDepth(z))
        uv = vUV;
    vec3 scene = pow(texture(uSceneColor, uv).rgb, vec3(2.2));
    vec3 transmittance = exp(-uAbsorption * thickness * (vec3(1.0) - uColor));
    vec3 refracted = scene * transmittance + uColor * uScatter * (vec3(1.0) - transmittance);

    float NdotV = max(dot(N, V), 1e-4);
    float fresnel = kF0 + (1.0 - kF0) * pow(1.0 - NdotV, 5.0);
    vec3 reflected = Reinhard(Environment(reflect(-V, N)));

    vec3 specular = vec3(0.0);
    float NdotL = dot(N, uLightDir);
    if (NdotL > 0.0)
    {
        vec3 H = normalize(uLightDir + V);
        float D = D_GGX(max(dot(N, H), 0.0), uRoughness);
        float G = G_Smith(NdotL, NdotV, uRoughness);
        vec3 F = F_Schlick(vec3(kF0), max(dot(uLightDir, H), 0.0));
        specular = Reinhard(D * G * F / max(4.0 * NdotL * NdotV, 1e-5) * NdotL * uLightColor);
    }

    vec3 color = mix(refracted, reflected, fresnel) + specular;
    FragColor = vec4(pow(clamp(color, 0.0, 1.0), vec3(1.0 / 2.2)), 1.0);
}
