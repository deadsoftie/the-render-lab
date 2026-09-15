#version 330 core

layout(location = 0) in vec3 aPos;
layout(location = 1) in vec3 aNrm;  // unused — present to match mesh vertex layout

uniform mat4 uModel;
uniform mat4 uLightVP;

void main()
{
    gl_Position = uLightVP * uModel * vec4(aPos, 1.0);
}
