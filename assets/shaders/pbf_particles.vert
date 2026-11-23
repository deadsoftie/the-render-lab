#version 430 core

layout(location = 0) in vec4 aPos;

uniform mat4 uView;
uniform mat4 uProj;
uniform float uRadius;
uniform float uViewportHeight;

out vec3 vViewCenter;

// Point sprite sized to the projected sphere.
void main()
{
    vec4 view = uView * vec4(aPos.xyz, 1.0);
    vViewCenter = view.xyz;
    gl_Position = uProj * view;
    // Projected diameter in pixels: viewport height * proj[1][1] * r / w.
    gl_PointSize = max(1.0, uViewportHeight * uProj[1][1] * uRadius / max(gl_Position.w, 1e-4));
}
