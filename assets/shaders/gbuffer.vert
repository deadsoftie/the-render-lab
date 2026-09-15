#version 330 core

layout(location=0) in vec3 aPos;
layout(location=1) in vec3 aNrm;
layout(location=2) in vec2 aUV;
layout(location=5) in vec3 aTangent;

uniform mat4 uModel;
uniform mat3 uNormalMatrix; // transpose(inverse(uModel)), computed on CPU
uniform mat4 uView;
uniform mat4 uProj;

out vec3 vWorldPos;
out vec3 vWorldNrm;
out vec2 vUV;
out vec3 vWorldTangent;

void main()
{
    vec4 world = uModel * vec4(aPos, 1.0);
    vWorldPos     = world.xyz;
    vWorldNrm     = uNormalMatrix * aNrm;
    vWorldTangent = uNormalMatrix * aTangent;
    vUV           = aUV;

    gl_Position = uProj * uView * world;
}
