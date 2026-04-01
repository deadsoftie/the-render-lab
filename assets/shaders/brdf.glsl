// brdf.glsl — GGX micro-facet BRDF
// Include after #version directive.  All functions are in linear colour space.

const float PI = 3.14159265359;

// Map Phong shininess (1..256) to GGX roughness (0..1).
// Uses the energy-equivalent relation from Walter et al. 2007.
float PhongToRoughness(float alpha)
{
    return sqrt(2.0 / (alpha + 2.0));
}

// GGX / Trowbridge-Reitz Normal Distribution Function.
float D_GGX(float NdotH, float roughness)
{
    float a  = roughness * roughness;
    float a2 = a * a;
    float d  = NdotH * NdotH * (a2 - 1.0) + 1.0;
    return a2 / (PI * d * d);
}

// Schlick-GGX single-term geometry helper.
float G1_SchlickGGX(float NdotX, float roughness)
{
    float r = roughness + 1.0;
    float k = (r * r) / 8.0;
    return NdotX / (NdotX * (1.0 - k) + k);
}

// Smith height-correlated geometry / shadowing-masking term.
float G_Smith(float NdotL, float NdotV, float roughness)
{
    return G1_SchlickGGX(NdotL, roughness) * G1_SchlickGGX(NdotV, roughness);
}

// Schlick Fresnel approximation.  F0 is specular reflectance at normal incidence.
vec3 F_Schlick(vec3 F0, float LdotH)
{
    return F0 + (1.0 - F0) * pow(1.0 - LdotH, 5.0);
}

// Full micro-facet BRDF evaluated for a single light direction.
// Returns the combined diffuse + specular contribution already multiplied by NdotL,
// ready to be scaled by light colour and attenuation.
//
// Parameters:
//   L       — normalised direction from surface to light
//   V       — normalised direction from surface to camera
//   N       — normalised surface normal
//   Kd      — diffuse albedo (linear)
//   F0      — specular colour at normal incidence (a.k.a. Ks)
//   alpha   — Phong shininess exponent (1..256); converted to GGX roughness internally
vec3 EvalBRDF(vec3 L, vec3 V, vec3 N, vec3 Kd, vec3 F0, float alpha)
{
    float roughness = PhongToRoughness(alpha);

    vec3  H     = normalize(L + V);
    float NdotL = max(dot(N, L), 0.0);
    float NdotV = max(dot(N, V), 1e-4);
    float NdotH = max(dot(N, H), 0.0);
    float LdotH = max(dot(L, H), 0.0);

    // Lambertian diffuse
    vec3 diffuse = Kd / PI;

    // Specular: Cook-Torrance micro-facet
    float D   = D_GGX(NdotH, roughness);
    float G   = G_Smith(NdotL, NdotV, roughness);
    vec3  F   = F_Schlick(F0, LdotH);
    vec3  spec = (D * G * F) / max(4.0 * NdotL * NdotV, 1e-5);

    return (diffuse + spec) * NdotL;
}
