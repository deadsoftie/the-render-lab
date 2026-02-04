#version 330 core

in vec3 vWorldPos;
in vec3 vWorldNrm;

out vec4 FragColor;

uniform vec3 uCamPos;

// Multi-light support
#define MAX_LIGHTS 4
uniform int  uLightCount;
uniform vec3 uLightPos[MAX_LIGHTS];
uniform vec3 uLightColor[MAX_LIGHTS];

uniform vec3  uAlbedo;
uniform float uAmbient;
uniform float uShininess;

void main()
{
    vec3 N = normalize(vWorldNrm);
    vec3 V = normalize(uCamPos - vWorldPos);

    vec3 ambient = uAmbient * uAlbedo;

    vec3 lit = vec3(0.0);
    int count = clamp(uLightCount, 0, MAX_LIGHTS);

    for (int i = 0; i < count; ++i)
    {
        vec3 L = normalize(uLightPos[i] - vWorldPos);

        // Lambert
        float ndotl = max(dot(N, L), 0.0);
        vec3 diffuse = uAlbedo * ndotl;

        // Blinn-Phong specular
        vec3 H = normalize(L + V);
        float specPow = pow(max(dot(N, H), 0.0), uShininess);
        vec3 spec = specPow * uLightColor[i];

        lit += diffuse * uLightColor[i] + spec;
    }

    vec3 color = ambient + lit;
    FragColor = vec4(color, 1.0);
}
