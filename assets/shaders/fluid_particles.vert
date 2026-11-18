#version 330 core

layout(location=0) in vec4 aPos;

uniform mat4 uView;
uniform mat4 uProj;
uniform float uRadius;
uniform float uViewportHeight;

out vec3 vViewCenter;

void main()
{
    vec4 view = uView * vec4(aPos.xyz, 1.0);
    vViewCenter = view.xyz;
    gl_Position = uProj * view;
    gl_PointSize = max(1.0, uRadius * uProj[1][1] * uViewportHeight / gl_Position.w);
}
