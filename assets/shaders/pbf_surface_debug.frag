#version 330 core

in vec2 vUV;
out vec4 FragColor;

uniform sampler2D uEyeDepth;
uniform sampler2D uThickness;
uniform sampler2D uNormals;
uniform int uMode;
uniform vec2 uDepthRange;
uniform float uAbsorption;
uniform vec3 uColor;

// Debug views: 1 depth, 3 normals, else thickness.
void main()
{
    if (uMode == 1)
    {
        float eyeDepth = texture(uEyeDepth, vUV).r;
        if (eyeDepth <= 0.0)
            discard;
        float t = clamp((eyeDepth - uDepthRange.x) / max(uDepthRange.y - uDepthRange.x, 1e-4), 0.0, 1.0);
        FragColor = vec4(vec3(1.0 - 0.85 * t), 1.0);
        return;
    }

    if (uMode == 3)
    {
        vec4 n = texture(uNormals, vUV);
        if (n.w <= 0.0)
            discard;
        FragColor = vec4(n.xyz * 0.5 + 0.5, 1.0);
        return;
    }

    float thickness = texture(uThickness, vUV).r;
    if (thickness <= 0.0)
        discard;
    float transmittance = exp(-uAbsorption * thickness);
    vec3 tint = mix(vec3(1.0), uColor, 1.0 - transmittance);
    FragColor = vec4(tint * (0.25 + 0.75 * transmittance), 1.0);
}
