#version 330 core

in  vec2 vUV;
out vec4 FragColor;

uniform sampler2D uWorldPosTex;
uniform sampler2D uNormalTex;

uniform mat4  uView;
uniform vec2  uTexelSize;       // 1 / resolution
uniform float uThickness   = 1.0;
uniform float uDepthThreshold  = 0.05;
uniform float uNormalThreshold = 0.3;
uniform vec3  uOutlineColor    = vec3(0.0);
uniform int   uDebugOutline    = 0;

// Returns view-space depth for a world-space position sample.
// Returns 0 for background pixels (wp.w < 0.5).
float SampleDepth(vec2 uv)
{
    vec4 wp = texture(uWorldPosTex, uv);
    if (wp.w < 0.5) return 0.0;
    return -(uView * vec4(wp.xyz, 1.0)).z;
}

// Returns the world-space normal at uv, or vec3(0) for background.
vec3 SampleNormal(vec2 uv)
{
    vec4 wp = texture(uWorldPosTex, uv);
    if (wp.w < 0.5) return vec3(0.0);
    return texture(uNormalTex, uv).xyz;
}

void main()
{
    // Background pixel — no outline on void
    vec4 wp = texture(uWorldPosTex, vUV);
    if (wp.w < 0.5)
    {
        FragColor = vec4(0.0);
        return;
    }

    vec2 step = uTexelSize * uThickness;

    // 3x3 neighbourhood offsets
    vec2 tl = vUV + vec2(-step.x,  step.y);
    vec2 t  = vUV + vec2( 0.0,     step.y);
    vec2 tr = vUV + vec2( step.x,  step.y);
    vec2 ml = vUV + vec2(-step.x,  0.0   );
    vec2 mr = vUV + vec2( step.x,  0.0   );
    vec2 bl = vUV + vec2(-step.x, -step.y);
    vec2 b  = vUV + vec2( 0.0,    -step.y);
    vec2 br = vUV + vec2( step.x, -step.y);

    // -------------------------------------------------------------------------
    // Depth Sobel (paper §2.1.1, Appendix A eq. 5-6)
    // -------------------------------------------------------------------------
    float dTL = SampleDepth(tl); float dT  = SampleDepth(t);  float dTR = SampleDepth(tr);
    float dML = SampleDepth(ml);                               float dMR = SampleDepth(mr);
    float dBL = SampleDepth(bl); float dB  = SampleDepth(b);  float dBR = SampleDepth(br);

    float dSx = -dTL - 2.0*dML - dBL + dTR + 2.0*dMR + dBR;
    float dSy = -dTL - 2.0*dT  - dTR + dBL + 2.0*dB  + dBR;
    float depthMag = sqrt(dSx*dSx + dSy*dSy);

    // -------------------------------------------------------------------------
    // Normal Sobel (paper §2.1.2) — applied per channel, magnitude combined
    // -------------------------------------------------------------------------
    vec3 nTL = SampleNormal(tl); vec3 nT  = SampleNormal(t);  vec3 nTR = SampleNormal(tr);
    vec3 nML = SampleNormal(ml);                               vec3 nMR = SampleNormal(mr);
    vec3 nBL = SampleNormal(bl); vec3 nB  = SampleNormal(b);  vec3 nBR = SampleNormal(br);

    vec3 nSx = -nTL - 2.0*nML - nBL + nTR + 2.0*nMR + nBR;
    vec3 nSy = -nTL - 2.0*nT  - nTR + nBL + 2.0*nB  + nBR;
    float normalMag = length(nSx) + length(nSy);

    // -------------------------------------------------------------------------
    // Combine (paper §2.1.2, Fig. 1(e))
    // -------------------------------------------------------------------------
    float edge = clamp(step(uDepthThreshold,  depthMag)
                     + step(uNormalThreshold, normalMag), 0.0, 1.0);

    if (uDebugOutline != 0)
    {
        // Show raw edge mask as greyscale (debug view 15)
        FragColor = vec4(edge, edge, edge, 1.0);
        return;
    }

    // Alpha-composite outline over existing framebuffer content
    FragColor = vec4(uOutlineColor, edge);
}
