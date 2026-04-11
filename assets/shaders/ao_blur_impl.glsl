// Bilateral blur implementation shared by ao_blur_h.frag and ao_blur_v.frag.
// The caller sets uTexelDir to (1/W, 0) for the horizontal pass and
// (0, 1/H) for the vertical pass.

in vec2 vUV;

layout(location = 0) out float FragAO;

uniform sampler2D uAOTex;       // input AO texture for this pass
uniform sampler2D uWorldPosTex; // GBuffer world pos — needed for camera-space depth
uniform sampler2D uNormalTex;   // GBuffer normal  — needed for range kernel

uniform mat4  uView;
uniform vec2  uTexelDir;    // one texel step along the blur axis
uniform float uDepthSigma;  // depth Gaussian variance s (default 0.01)
uniform int   uBlurRadius;  // half-width in pixels       (default 8)

// Camera-space depth: positive distance from the eye along -Z.
float CameraDepth(vec3 worldPos)
{
    return -(uView * vec4(worldPos, 1.0)).z;
}

// Unnormalised Gaussian — normalisation cancels in the weighted average.
float Gaussian(float x, float sigma)
{
    return exp(-0.5 * x * x / (sigma * sigma));
}

void main()
{
    // -------------------------------------------------------------------------
    // No geometry — pass through 1.0 (fully unoccluded).
    // -------------------------------------------------------------------------
    vec4 wp = texture(uWorldPosTex, vUV);
    if (wp.w < 0.5)
    {
        FragAO = 1.0;
        return;
    }

    vec3  N = normalize(texture(uNormalTex, vUV).xyz);
    float d = CameraDepth(wp.xyz);

    // Spatial sigma: kernel falls to ~0.01 at the edge (radius = 3 sigma).
    float sigmaSpatial = max(float(uBlurRadius) / 3.0, 1e-5);

    float sum  = 0.0;
    float wsum = 0.0;

    for (int j = -uBlurRadius; j <= uBlurRadius; ++j)
    {
        vec2 sUV = vUV + float(j) * uTexelDir;

        // Skip samples outside the viewport.
        if (any(lessThan(sUV, vec2(0.0))) || any(greaterThan(sUV, vec2(1.0))))
            continue;

        vec4 sWP = texture(uWorldPosTex, sUV);
        if (sWP.w < 0.5)
            continue; // background — don't let it bleed into geometry AO

        vec3  Ni = normalize(texture(uNormalTex, sUV).xyz);
        float di = CameraDepth(sWP.xyz);
        float Ii = texture(uAOTex, sUV).r;

        // Spatial kernel S(xi, x): Gaussian on pixel distance.
        float S_w = Gaussian(float(j), sigmaSpatial);

        // Range kernel R(xi, x): normal alignment × depth Gaussian.
        // max(0, Ni·N) goes to 0 across a surface discontinuity.
        float R_w = max(0.0, dot(Ni, N)) * Gaussian(di - d, uDepthSigma);

        float w = S_w * R_w;
        sum  += w * Ii;
        wsum += w;
    }

    FragAO = sum / max(wsum, 1e-6);
}
