#version 330 core

in vec2 vUV;

uniform sampler2D uEyeDepth;
uniform mat4 uProj;

void main()
{
    float z = texture(uEyeDepth, vUV).r;
    if (z <= 0.0)
        discard;
    vec4 clip = uProj * vec4(0.0, 0.0, -z, 1.0);
    gl_FragDepth = clip.z / clip.w * 0.5 + 0.5;
}
