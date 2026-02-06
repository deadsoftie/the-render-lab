#version 330 core

in vec3 vWorldPos;
in vec3 vWorldNrm;

layout(location=0) out vec4 gWorldPos;   // xyz
layout(location=1) out vec4 gNormal;     // xyz
layout(location=2) out vec4 gKd;         // rgb
layout(location=3) out vec4 gKsAlpha;    // rgb = Ks, a = shininess (alpha)

uniform vec3  uKd;
uniform vec3  uKs;
uniform float uAlpha; // shininess

void main()
{
    gWorldPos = vec4(vWorldPos, 1.0);
    gNormal   = vec4(normalize(vWorldNrm), 1.0);
    gKd       = vec4(uKd, 1.0);
    gKsAlpha  = vec4(uKs, uAlpha);
}