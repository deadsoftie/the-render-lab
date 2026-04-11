#version 330 core

// Alchemy Ambient Obscurance
// Reference: McGuire et al. — equations from CS562 Project 4 spec
//
// Samples n points in a screen-space spiral around each pixel, weighted by
// the Alchemy formula, to produce an occlusion factor in [0, 1].

in vec2 vUV;

layout(location = 0) out float FragAO;

uniform sampler2D uWorldPosTex;  // GBuffer attachment 0 — world pos (w=1 if geometry)
uniform sampler2D uNormalTex;    // GBuffer attachment 1 — world normal

uniform mat4  uView;      // world → camera space
uniform float uR;         // range of influence (world units, ~1 m)
uniform int   uN;         // sample count (10–20)
uniform float uC;         // falloff constant (~0.1 * R)
uniform float uDelta;     // depth bias (~0.001)
uniform float uScaleS;    // AO intensity scale s
uniform float uContrastK; // AO contrast exponent k

const float PI = 3.14159265358979;

void main()
{
    // -------------------------------------------------------------------------
    // Background — no geometry at this pixel
    // -------------------------------------------------------------------------
    vec4 wp = texture(uWorldPosTex, vUV);
    if (wp.w < 0.5)
    {
        FragAO = 1.0;
        return;
    }

    vec3  worldPos = wp.xyz;
    vec3  N        = normalize(texture(uNormalTex, vUV).xyz);

    // Camera-space depth of the centre pixel (positive value, distance from eye).
    float d = -(uView * vec4(worldPos, 1.0)).z;
    d = max(d, 1e-5);

    // -------------------------------------------------------------------------
    // Per-pixel pseudo-random rotation angle (hash on integer pixel coords).
    // φ = ((30*x') XOR y') + 10*x'*y'
    // -------------------------------------------------------------------------
    ivec2 px  = ivec2(gl_FragCoord.xy);
    float phi = float((30 * px.x ^ px.y) + 10 * px.x * px.y);
    phi = mod(phi, 2.0 * PI);

    // -------------------------------------------------------------------------
    // Alchemy AO spiral accumulation
    // -------------------------------------------------------------------------
    float S  = 0.0;
    float c2 = uC * uC;
    int   n  = max(uN, 1);

    for (int i = 0; i < n; ++i)
    {
        // Spiral parameter α in (0, 1)
        float alpha = (float(i) + 0.5) / float(n);

        // Screen-space spiral radius (UV units) — R projected to screen at depth d
        float h = alpha * uR / d;

        // Spiral angle: 7 full turns per 9 samples, offset by the pixel hash
        float theta = 2.0 * PI * alpha * (7.0 * float(n) / 9.0) + phi;

        vec2 sUV = vUV + h * vec2(cos(theta), sin(theta));

        // Skip samples outside the viewport
        if (any(lessThan(sUV, vec2(0.0))) || any(greaterThan(sUV, vec2(1.0))))
            continue;

        vec4 sWP = texture(uWorldPosTex, sUV);
        if (sWP.w < 0.5)
            continue; // background sample

        // ω_i = P_i − P
        vec3  omega    = sWP.xyz - worldPos;
        float omegaLen = length(omega);

        // Heaviside H(R − ||ω_i||): skip samples beyond range
        if (omegaLen >= uR)
            continue;

        // Camera-space depth of the sample point (for the depth-bias term)
        float di = -(uView * vec4(sWP.xyz, 1.0)).z;

        // Alchemy numerator and denominator
        float num = max(0.0, dot(N, omega) - uDelta * di);
        float den = max(c2, dot(omega, omega));

        S += num / den;
    }

    // Apply scale factor: S = (2πc / n) * Σ(...)
    S *= (2.0 * PI * uC) / float(n);

    // Final occlusion factor — clamped and contrast-adjusted
    FragAO = pow(clamp(1.0 - uScaleS * S, 0.0, 1.0), uContrastK);
}
