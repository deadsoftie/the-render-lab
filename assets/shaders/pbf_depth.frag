#version 430 core

in vec3 vViewCenter;

layout(location = 0) out float outEyeDepth;

uniform mat4 uProj;
uniform float uRadius;
uniform sampler2D uSceneDepth;
uniform vec2 uViewport;

// SSFR: nearest sphere eye depth, occluded by the scene depth.
void main()
{
    vec2 xy = vec2(gl_PointCoord.x * 2.0 - 1.0, 1.0 - gl_PointCoord.y * 2.0);
    float r2 = dot(xy, xy);
    if (r2 > 1.0)
        discard;

    vec3 viewPos = vViewCenter + vec3(xy, sqrt(1.0 - r2)) * uRadius;
    vec4 clip = uProj * vec4(viewPos, 1.0);
    float depth = clip.z / clip.w * 0.5 + 0.5;
    float sceneDepth = texture(uSceneDepth, gl_FragCoord.xy / uViewport).r;
    if (depth >= sceneDepth)
        discard;

    gl_FragDepth = depth;
    outEyeDepth = -viewPos.z;
}
