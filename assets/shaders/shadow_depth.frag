#version 330 core

in vec3 vWorldPos;

uniform vec3  uLightPos;
uniform float uFarPlane;

void main()
{
    // Store linear distance normalised to [0,1] so PCF comparisons are stable
    gl_FragDepth = length(vWorldPos - uLightPos) / uFarPlane;
}
