#version 330 core

in vec3 vWorldPos;
flat in int vLevel;
out vec4 FragColor;

uniform vec3 uCamPos;
uniform sampler2D uGWorldPos;
uniform float uOccludedDim;

const vec3 kLevelColors[4] = vec3[](vec3(0.25, 0.55, 1.0),
                                    vec3(0.3, 1.0, 0.45),
                                    vec3(1.0, 0.8, 0.2),
                                    vec3(1.0, 0.35, 0.35));

void main()
{
    // Drawn without depth testing; lines behind GBuffer geometry are dimmed instead of hidden.
    vec4 scene = texelFetch(uGWorldPos, ivec2(gl_FragCoord.xy), 0);
    float sceneDist = (scene.w > 0.5) ? distance(uCamPos, scene.xyz) : 1e30;
    bool occluded = sceneDist + 0.02 < distance(uCamPos, vWorldPos);
    vec3 color = kLevelColors[min(vLevel, 3)];
    FragColor = vec4(occluded ? color * uOccludedDim : color, 1.0);
}
