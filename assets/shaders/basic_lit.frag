#version 330 core

in vec3 vWorldPos;
in vec3 vWorldNrm;

out vec4 FragColor;

uniform vec3 uCamPos;
uniform vec3 uLightPos;
uniform vec3 uLightColor;

uniform vec3 uAlbedo;
uniform float uAmbient;
uniform float uShininess;

void main()
{
    vec3 N = normalize(vWorldNrm);
    vec3 L = normalize(uLightPos - vWorldPos);
    vec3 V = normalize(uCamPos - vWorldPos);

    // Lambert
    float ndotl = max(dot(N, L), 0.0);
    vec3 diffuse = uAlbedo * ndotl;

    // Blinn-Phong specular
    vec3 H = normalize(L + V);
    float specPow = pow(max(dot(N, H), 0.0), uShininess);
    vec3 spec = specPow * uLightColor;

    vec3 ambient = uAmbient * uAlbedo;

    vec3 color = ambient + (diffuse + spec) * uLightColor;
    FragColor = vec4(color, 1.0);
}
