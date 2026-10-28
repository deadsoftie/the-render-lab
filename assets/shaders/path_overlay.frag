#version 330 core

in vec3 vWorldPos;
out vec4 FragColor;

uniform vec3 uColor;
uniform vec3 uCamPos;
uniform sampler2D uGWorldPos;
uniform float uOccludedDim;

// The default framebuffer has no scene depth in the deferred path, so occlusion is tested against the GBuffer world position instead.
void main()
{
    vec4 scene = texelFetch(uGWorldPos, ivec2(gl_FragCoord.xy), 0);
    float sceneDist = (scene.w > 0.5) ? distance(uCamPos, scene.xyz) : 1e30;
    bool occluded = sceneDist + 0.05 < distance(uCamPos, vWorldPos);
    FragColor = vec4(occluded ? uColor * uOccludedDim : uColor, 1.0);
}
