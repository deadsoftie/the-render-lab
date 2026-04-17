// shadows.glsl — shared shadow helpers included by deferred light shaders.
// Requires: PI (from brdf.glsl, included before this file).

// 12-sample offset kernel for PCF (cube corners + edge midpoints, good coverage)
const vec3 kPcfDirs[12] = vec3[](
    vec3( 1, 1, 1), vec3( 1,-1, 1), vec3(-1,-1, 1), vec3(-1, 1, 1),
    vec3( 1, 1,-1), vec3( 1,-1,-1), vec3(-1,-1,-1), vec3(-1, 1,-1),
    vec3( 1, 1, 0), vec3( 1,-1, 0), vec3(-1,-1, 0), vec3(-1, 1, 0)
);

// Hamburger 4-Moment Shadow Mapping
// b: moments (z, z^2, z^3, z^4), zf: normalised fragment depth, alpha: light-leak bias
float MSMShadow(vec4 b, float zf, float alpha)
{
    // Bias moments toward Dirac at 0.5: (0.5^1, 0.5^2, 0.5^3, 0.5^4)
    vec4 bp = mix(b, vec4(0.5, 0.25, 0.125, 0.0625), alpha);

    // Cholesky decomposition of 3x3 Hankel moment matrix
    float bv = bp.x, c = bp.y;
    float d = sqrt(max(bp.y - bp.x*bp.x, 0.0));
    if (d < 1e-4) d = 1e-4;
    float e = (bp.z - bp.x*bp.y) / d;
    float f = sqrt(max(bp.w - bp.y*bp.y - e*e, 0.0));
    if (f < 1e-4) f = 1e-4;

    // Forward substitution: solve L * y = [1, zf, zf^2]
    float ch2 = (zf - bv) / d;
    float ch3 = (zf*zf - c - e*ch2) / f;

    // Back substitution: solve L^T * c_vec = y
    float c3 = ch3 / f;
    float c2 = (ch2 - e*c3) / d;
    float c1 = 1.0 - bv*c2 - c*c3;

    // Roots of quadratic c1 + c2*z + c3*z^2 = 0
    float disc   = max(c2*c2 - 4.0*c3*c1, 0.0);
    float sq     = sqrt(disc);
    float inv2c3 = 1.0 / (2.0*c3 + 1e-6);
    float z2 = (-c2 - sq) * inv2c3;
    float z3 = (-c2 + sq) * inv2c3;
    if (z2 > z3) { float t = z2; z2 = z3; z3 = t; }

    // Shadow factor from moment statistics
    if (zf <= z2) return 0.0;
    if (zf <= z3)
        return clamp((zf*z3 - bp.x*(zf+z3) + bp.y) / ((z3-z2)*(zf-z2) + 1e-6), 0.0, 1.0);
    return clamp(1.0 - (z2*z3 - bp.x*(z2+z3) + bp.y) / ((zf-z2)*(zf-z3) + 1e-6), 0.0, 1.0);
}

// Windowed inverse-square attenuation with smooth cutoff at range r.
float Attenuation(float d, float r)
{
    if (d >= r) return 0.0;
    float invd2 = 1.0 / max(d * d, 1e-6);
    float invr2 = 1.0 / max(r * r, 1e-6);
    float att   = max(invd2 - invr2, 0.0);
    float t     = clamp(1.0 - d / max(r, 1e-6), 0.0, 1.0);
    return att * t * t;
}
