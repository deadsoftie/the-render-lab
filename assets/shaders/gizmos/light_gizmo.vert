#version 330 core
layout (location = 0) in vec3 aPos;
layout (location = 1) in vec2 aUV;

uniform mat4 uView;
uniform mat4 uProj;
uniform vec3 uWorldPos;
uniform float uSize;

out vec2 vUV;

void main()
{
    // Billboard: extract camera right/up vectors from view matrix
    vec3 right = vec3(uView[0][0], uView[1][0], uView[2][0]);
    vec3 up    = vec3(uView[0][1], uView[1][1], uView[2][1]);

    vec3 world =
        uWorldPos +
        right * aPos.x * uSize +
        up    * aPos.y * uSize;

    gl_Position = uProj * uView * vec4(world, 1.0);
    vUV = aUV;
}
