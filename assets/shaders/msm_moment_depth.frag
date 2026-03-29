#version 330 core

in vec3 vWorldPos;

uniform vec3  uLightPos;
uniform float uFarPlane;

out vec4 FragMoments;

void main()
{
    float z = length(vWorldPos - uLightPos) / uFarPlane;
    FragMoments = vec4(z, z*z, z*z*z, z*z*z*z);
}
