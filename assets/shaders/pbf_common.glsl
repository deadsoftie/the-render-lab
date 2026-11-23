#ifndef PBF_GROUP_SIZE
#define PBF_GROUP_SIZE 256
#endif
#define PBF_MAX_NEIGHBORS 64

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
    float lambdaScale;
    int pad1;
};

layout(std140, binding = 1) uniform PbfShapes
{
    ivec4 shapeCount;
    vec4 shapeA[8];
    vec4 shapeB[8];
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

vec3 PushOutOfShape(vec3 p, int s, float r)
{
    vec4 a = shapeA[s];
    vec4 b = shapeB[s];
    vec3 lo = domainMin.xyz + vec3(r);
    vec3 hi = domainMax.xyz - vec3(r);
    if (a.w < 0.5)
    {
        vec3 center = 0.5 * (a.xyz + b.xyz);
        vec3 halfExt = 0.5 * (b.xyz - a.xyz);
        vec3 q = abs(p - center) - halfExt;
        if (max(q.x, max(q.y, q.z)) < 0.0)
        {
            vec3 local = p - center;
            float best = 1e30;
            int bestAxis = -1;
            float bestTarget = 0.0;
            for (int axis = 0; axis < 3; ++axis)
            {
                for (int side = -1; side <= 1; side += 2)
                {
                    float target = center[axis] + float(side) * (halfExt[axis] + r);
                    if (target < lo[axis] - 1e-6 || target > hi[axis] + 1e-6)
                        continue;
                    float depth = halfExt[axis] - float(side) * local[axis];
                    if (depth < best)
                    {
                        best = depth;
                        bestAxis = axis;
                        bestTarget = target;
                    }
                }
            }
            if (bestAxis >= 0)
                p[bestAxis] = bestTarget;
            return p;
        }
        vec3 closest = clamp(p, a.xyz, b.xyz);
        vec3 d = p - closest;
        float dist = length(d);
        if (dist < r && dist > 1e-7)
            p = closest + d * (r / dist);
        return p;
    }
    vec3 d = p - a.xyz;
    float dist = length(d);
    float reach = b.x + r;
    if (dist >= reach || dist <= 1e-7)
        return p;
    vec3 target = a.xyz + d * (reach / dist);
    for (int axis = 0; axis < 3; ++axis)
    {
        if (target[axis] >= lo[axis] - 1e-6 && target[axis] <= hi[axis] + 1e-6)
            continue;
        vec3 side = d;
        side[axis] = 0.0;
        float sideLength = length(side);
        if (sideLength < 1e-7)
        {
            side = axis == 0 ? vec3(0.0, 0.0, 1.0) : vec3(1.0, 0.0, 0.0);
            sideLength = 1.0;
        }
        float along = clamp(d[axis], -reach, reach);
        float radial = sqrt(max(reach * reach - along * along, 0.0));
        target = a.xyz + side * (radial / sideLength);
        target[axis] = a.xyz[axis] + along;
        break;
    }
    return target;
}

vec3 Collide(vec3 p)
{
    float r = ParticleRadius();
    for (int s = 0; s < shapeCount.x; ++s)
        p = PushOutOfShape(p, s, r);
    return clamp(p, domainMin.xyz + vec3(r), domainMax.xyz - vec3(r));
}

#ifdef PBF_NEIGHBOR_LIST
layout(std430, binding = 11) readonly buffer NeighborList { uint neighborList[]; };
#define PBF_NEIGHBOR_BEGIN(P) { uint nbBase_ = i * uint(PBF_MAX_NEIGHBORS); uint nbCount_ = neighborList[nbBase_]; for (uint nb_ = 1u; nb_ <= nbCount_; ++nb_) { uint j = neighborList[nbBase_ + nb_];
#define PBF_NEIGHBOR_END } }
#else
#define PBF_NEIGHBOR_BEGIN(P) { ivec3 nc_ = CellCoord(P); for (int z_ = max(nc_.z - 1, 0); z_ <= min(nc_.z + 1, gridDims.z - 1); ++z_) for (int y_ = max(nc_.y - 1, 0); y_ <= min(nc_.y + 1, gridDims.y - 1); ++y_) for (int x_ = max(nc_.x - 1, 0); x_ <= min(nc_.x + 1, gridDims.x - 1); ++x_) { int cell_ = CellIndex(ivec3(x_, y_, z_)); uint start_ = cellStart[cell_]; uint end_ = start_ + cellCount[cell_]; for (uint j = start_; j < end_; ++j) {
#define PBF_NEIGHBOR_END } } }
#endif
