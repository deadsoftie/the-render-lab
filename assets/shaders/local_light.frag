#version 330 core

out vec4 FragColor;

uniform sampler2D uWorldPosTex;
uniform sampler2D uNormalTex;
uniform sampler2D uKdTex;
uniform sampler2D uKsAlphaTex;

uniform vec3 uCamPos;

uniform vec3 uLightPos;
uniform vec3 uLightColor;
uniform float uLightRange;

uniform vec2 uInvResolution; // 1/width, 1/height

// Shadow
uniform samplerCube uShadowMap;
uniform int         uShadowsActive;
uniform float       uShadowFarPlane;
uniform float       uShadowBias      = 0.04;
uniform float       uShadowPcfRadius = 0.05;

const vec3 kPcfDirs[20] = vec3[](
    vec3( 1, 1, 1), vec3( 1,-1, 1), vec3(-1,-1, 1), vec3(-1, 1, 1),
    vec3( 1, 1,-1), vec3( 1,-1,-1), vec3(-1,-1,-1), vec3(-1, 1,-1),
    vec3( 1, 1, 0), vec3( 1,-1, 0), vec3(-1,-1, 0), vec3(-1, 1, 0),
    vec3( 1, 0, 1), vec3(-1, 0, 1), vec3( 1, 0,-1), vec3(-1, 0,-1),
    vec3( 0, 1, 1), vec3( 0,-1, 1), vec3( 0,-1,-1), vec3( 0, 1,-1)
);

float ShadowPCF(vec3 worldPos, vec3 lightPos, float farPlane)
{
    vec3  dir         = worldPos - lightPos;
    float currentDist = length(dir);
    float shadow      = 0.0;
    for (int s = 0; s < 20; ++s)
    {
        float closest = texture(uShadowMap,
                                dir + kPcfDirs[s] * uShadowPcfRadius).r * farPlane;
        shadow += (currentDist - uShadowBias > closest) ? 1.0 : 0.0;
    }
    return shadow / 20.0;
}

void main()
{
    vec2 uv = gl_FragCoord.xy * uInvResolution;

    vec4 wp = texture(uWorldPosTex, uv);
    if (wp.w < 0.5)
    {
        FragColor = vec4(0.0);
        return;
    }
    vec3 worldPos = wp.xyz;
    vec3 N = normalize(texture(uNormalTex, uv).xyz);
    vec3 Kd = texture(uKdTex, uv).rgb;

    vec4 ksA = texture(uKsAlphaTex, uv);
    vec3 Ks = ksA.rgb;
    float alpha = max(ksA.a, 1.0);

    vec3 toL = uLightPos - worldPos;
    float d = length(toL);
    float r = uLightRange;

    if (d >= r)
    {
        FragColor = vec4(0.0);
        return;
    }

    float invd2 = 1.0 / max(d*d, 1e-6);
    float invr2 = 1.0 / max(r*r, 1e-6);
    float att   = max(invd2 - invr2, 0.0);

    vec3 L = toL / max(d, 1e-6);
    vec3 V = normalize(uCamPos - worldPos);

    float ndotl = max(dot(N, L), 0.0);
    vec3 diffuse = Kd * ndotl;

    vec3 H = normalize(L + V);
    float specPow = pow(max(dot(N, H), 0.0), alpha);
    vec3 spec = specPow * uLightColor;

    vec3 outCol = (diffuse * uLightColor + spec) * att;

    float shadowFactor = 0.0;
    if (uShadowsActive != 0)
        shadowFactor = ShadowPCF(worldPos, uLightPos, uShadowFarPlane);

    FragColor = vec4(outCol * (1.0 - shadowFactor), 1.0);
}
