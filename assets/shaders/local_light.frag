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

// Moment shadow map (texture unit 5)
uniform samplerCube uMSMMap;
uniform int         uUseMSM;
uniform float       uMSMAlpha = 0.001;

// Hamburger 4-Moment Shadow Mapping
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
    {
        if (uUseMSM != 0)
        {
            vec3 dir = worldPos - uLightPos;
            float zf = length(dir) / uShadowFarPlane;
            vec4 moments = texture(uMSMMap, dir);
            shadowFactor = MSMShadow(moments, zf, uMSMAlpha);
        }
        else
        {
            shadowFactor = ShadowPCF(worldPos, uLightPos, uShadowFarPlane);
        }
    }

    FragColor = vec4(outCol * (1.0 - shadowFactor), 1.0);
}
