#version 330 core

layout(location=0) in vec3 aPos;
layout(location=1) in vec3 aNrm;

uniform mat4 uModel;
uniform mat4 uView;
uniform mat4 uProj;

out vec3 vWorldPos;
out vec3 vWorldNrm;

void main()
{
    vec4 world = uModel * vec4(aPos, 1.0);
    vWorldPos = world.xyz;

    // correct normal transform for non-uniform scale
    vWorldNrm = mat3(transpose(inverse(uModel))) * aNrm;

    gl_Position = uProj * uView * world;
}
