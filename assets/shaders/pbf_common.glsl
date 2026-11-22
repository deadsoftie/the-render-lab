layout(std140, binding = 0) uniform PbfParams
{
    vec4 domainMin;
    vec4 domainMax;
    vec4 gravity;
    ivec4 gridDims;
    float rho0;
    float invRho0;
    float epsilon;
    float sCorrK;
    float sCorrDq;
    float sCorrN;
    float xsphC;
    float vorticityEps;
    float poly6;
    float spikyGrad;
    float wDq;
    float mass;
    int particleCount;
    int iterations;
    int pad0;
    int pad1;
};

layout(std430, binding = 6) buffer CellCounts { uint cellCount[]; };
layout(std430, binding = 7) buffer CellStarts { uint cellStart[]; };
layout(std430, binding = 8) buffer ParticleCells { uint particleCell[]; };
layout(std430, binding = 9) buffer ParticleRanks { uint particleRank[]; };

float SmoothingRadius()
{
    return domainMin.w;
}

float ParticleRadius()
{
    return domainMax.w;
}

float TimeStep()
{
    return gravity.w;
}

ivec3 CellCoord(vec3 p)
{
    ivec3 c = ivec3(floor((p - domainMin.xyz) / SmoothingRadius()));
    return clamp(c, ivec3(0), gridDims.xyz - ivec3(1));
}

int CellIndex(ivec3 c)
{
    return (c.z * gridDims.y + c.y) * gridDims.x + c.x;
}

float Poly6(float r2)
{
    float h = SmoothingRadius();
    float d = h * h - r2;
    return d > 0.0 ? poly6 * d * d * d : 0.0;
}

vec3 SpikyGrad(vec3 r, float len)
{
    float h = SmoothingRadius();
    if (len >= h || len < 1e-6)
        return vec3(0.0);
    float d = h - len;
    return (spikyGrad * d * d / len) * r;
}

#define PBF_NEIGHBOR_BEGIN(P) { ivec3 nc_ = CellCoord(P); for (int z_ = max(nc_.z - 1, 0); z_ <= min(nc_.z + 1, gridDims.z - 1); ++z_) for (int y_ = max(nc_.y - 1, 0); y_ <= min(nc_.y + 1, gridDims.y - 1); ++y_) for (int x_ = max(nc_.x - 1, 0); x_ <= min(nc_.x + 1, gridDims.x - 1); ++x_) { int cell_ = CellIndex(ivec3(x_, y_, z_)); uint start_ = cellStart[cell_]; uint end_ = start_ + cellCount[cell_]; for (uint j = start_; j < end_; ++j) {
#define PBF_NEIGHBOR_END } } }
