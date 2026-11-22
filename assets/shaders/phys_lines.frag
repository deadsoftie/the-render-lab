#version 430 core

in vec3 vColor;
out vec4 FragColor;

uniform sampler2D uSceneDepth;
uniform vec2 uViewport;
uniform int uDepthTest;
uniform float uDepthBias;

void main()
{
    if (uDepthTest == 1)
    {
        float sceneDepth = texture(uSceneDepth, gl_FragCoord.xy / uViewport).r;
        if (gl_FragCoord.z > sceneDepth + uDepthBias)
            discard;
    }
    FragColor = vec4(vColor, 1.0);
}
