#version 430 core

// Raymarches the simulation's sparse level set after the lighting pass and writes shaded water
// over the lit frame. The tree (node pool and child lists) and the atlas level set are CUDA
// interop copies; the constants below mirror SparseTypes.h in the fluid CUDA sources.

in vec2 vUV;
out vec4 FragColor;

layout(std430, binding = 0) readonly buffer NodeBuffer
{
    int uNodes[];
};

layout(std430, binding = 1) readonly buffer ChildListBuffer
{
    int uChildLists[];
};

uniform sampler2D uSceneColor;
uniform sampler2D uWorldPosTex;
uniform sampler3D uLevelSet;
uniform sampler2D uHDRITex;
uniform int uHasHDRI;

uniform mat4 uInvViewProj;
uniform mat4 uView;
uniform vec3 uCamPos;
uniform vec3 uOrigin;
uniform float uVoxelSize;
uniform int uCoordOffset;
uniform vec3 uAtlasDims;
uniform vec3 uDomainMin;
uniform vec3 uDomainMax;
uniform vec3 uWaterColor;
uniform float uAbsorption;
uniform float uRefraction;
uniform float uExposure;
uniform float uHDRIRotation;

const int kNodeStride = 7;
const int kLog2Dim = 3;
const int kBrickDim = 8;
const int kChildCount = 512;
const int kBrickTexels = 10;
const int kAtlasBricksXY = 16;
const int kMaxSteps = 512;
const int kBisectionSteps = 6;
const int kThicknessSteps = 64;
const float kPi = 3.14159265359;
const float kWaterF0 = 0.02;
const vec3 kSkyColor = vec3(0.55, 0.65, 0.8);

int NodeLevel(int n)
{
    return uNodes[n * kNodeStride];
}

ivec3 NodeCoord(int n)
{
    return ivec3(uNodes[n * kNodeStride + 1], uNodes[n * kNodeStride + 2], uNodes[n * kNodeStride + 3]);
}

int NodeChildList(int n)
{
    return uNodes[n * kNodeStride + 5];
}

int NodeBrick(int n)
{
    return uNodes[n * kNodeStride + 6];
}

ivec3 VoxelOf(vec3 p)
{
    return ivec3(floor((p - uOrigin) / uVoxelSize)) + uCoordOffset;
}

// Leaf node containing the voxel, or -1 with emptyLevel set to the level of the missing child, so
// the caller can skip that whole empty cell.
int FindLeaf(ivec3 voxel, out int emptyLevel)
{
    int rootLevel = NodeLevel(0);
    emptyLevel = rootLevel;
    if (any(notEqual(voxel >> (kLog2Dim * (rootLevel + 1)), NodeCoord(0))))
        return -1;

    int current = 0;
    for (int level = rootLevel - 1; level >= 0; --level)
    {
        ivec3 c = (voxel >> (kLog2Dim * (level + 1))) & (kBrickDim - 1);
        int child = uChildLists[NodeChildList(current) * kChildCount + c.x + kBrickDim * (c.y + kBrickDim * c.z)];
        if (child < 0)
        {
            emptyLevel = level;
            return -1;
        }
        current = child;
    }
    emptyLevel = -1;
    return current;
}

// The level set is stored at voxel centers, so the brick-local position maps straight onto the
// texture coordinate (texel centers sit half a texel in). Taps stay inside the brick and its apron.
float SampleLeaf(int leaf, vec3 p)
{
    int brick = NodeBrick(leaf);
    vec3 local = (p - uOrigin) / uVoxelSize - vec3((NodeCoord(leaf) << kLog2Dim) - uCoordOffset);
    vec3 base = vec3((brick % kAtlasBricksXY) * kBrickTexels + 1,
                     ((brick / kAtlasBricksXY) % kAtlasBricksXY) * kBrickTexels + 1,
                     (brick / (kAtlasBricksXY * kAtlasBricksXY)) * kBrickTexels + 1);
    return texture(uLevelSet, (base + local) / uAtlasDims).r;
}

float Phi(vec3 p)
{
    int emptyLevel;
    int leaf = FindLeaf(VoxelOf(p), emptyLevel);
    return leaf < 0 ? uVoxelSize : SampleLeaf(leaf, p);
}

vec2 RayBox(vec3 ro, vec3 invDir, vec3 lo, vec3 hi)
{
    vec3 t0 = (lo - ro) * invDir;
    vec3 t1 = (hi - ro) * invDir;
    vec3 tMin = min(t0, t1);
    vec3 tMax = max(t0, t1);
    return vec2(max(max(tMin.x, tMin.y), tMin.z), min(min(tMax.x, tMax.y), tMax.z));
}

float CellExit(ivec3 voxel, int level, vec3 ro, vec3 invDir)
{
    int shift = kLog2Dim * (level + 1);
    vec3 lo = uOrigin + vec3(((voxel >> shift) << shift) - uCoordOffset) * uVoxelSize;
    vec3 hi = lo + float(1 << shift) * uVoxelSize;
    return RayBox(ro, invDir, lo, hi).y;
}

vec3 RotateY(vec3 d, float yaw)
{
    float c = cos(yaw), s = sin(yaw);
    return vec3(d.x * c + d.z * s, d.y, -d.x * s + d.z * c);
}

vec2 EquirectUV(vec3 w)
{
    return vec2(0.5 - atan(w.z, w.x) / (2.0 * kPi), acos(clamp(w.y, -1.0, 1.0)) / kPi);
}

vec3 ToneMap(vec3 c)
{
    vec3 mapped = (uExposure * c) / (uExposure * c + vec3(1.0));
    return pow(max(mapped, vec3(0.0)), vec3(1.0 / 2.2));
}

void main()
{
    vec4 clip = vec4(vUV * 2.0 - 1.0, 1.0, 1.0);
    vec4 farPoint = uInvViewProj * clip;
    vec3 ro = uCamPos;
    vec3 rd = normalize(farPoint.xyz / farPoint.w - ro);
    vec3 safeDir = mix(rd, vec3(1e-8), lessThan(abs(rd), vec3(1e-8)));
    vec3 invDir = 1.0 / safeDir;

    vec4 worldPos = texture(uWorldPosTex, vUV);
    float sceneT = worldPos.w > 0.5 ? length(worldPos.xyz - ro) : 1e30;

    int rootShift = kLog2Dim * (NodeLevel(0) + 1);
    vec3 rootLo = uOrigin + vec3((NodeCoord(0) << rootShift) - uCoordOffset) * uVoxelSize;
    vec3 rootHi = rootLo + float(1 << rootShift) * uVoxelSize;
    vec2 range = RayBox(ro, invDir, max(rootLo, uDomainMin), min(rootHi, uDomainMax));
    float t = max(range.x, 0.0);
    float tEnd = min(range.y, sceneT);
    if (t >= tEnd)
        discard;

    float skip = 1e-3 * uVoxelSize;
    float prevT = t;
    float prevPhi = uVoxelSize;
    float tHit = -1.0;
    for (int i = 0; i < kMaxSteps && t < tEnd; ++i)
    {
        vec3 p = ro + rd * t;
        ivec3 voxel = VoxelOf(p);
        int emptyLevel;
        int leaf = FindLeaf(voxel, emptyLevel);
        if (leaf < 0)
        {
            t = max(CellExit(voxel, emptyLevel, ro, invDir), t) + skip;
            prevT = t;
            prevPhi = uVoxelSize;
            continue;
        }

        float phi = SampleLeaf(leaf, p);
        if (phi < 0.0)
        {
            float a = prevT;
            float b = t;
            if (prevPhi > 0.0)
            {
                for (int k = 0; k < kBisectionSteps; ++k)
                {
                    float m = 0.5 * (a + b);
                    if (Phi(ro + rd * m) < 0.0)
                        b = m;
                    else
                        a = m;
                }
            }
            tHit = b;
            break;
        }
        prevT = t;
        prevPhi = phi;
        // The Zhu-Bridson level set is not a true distance, so steps are capped at half a voxel
        // to avoid jumping over thin sheets.
        t += clamp(0.8 * phi, 0.1 * uVoxelSize, 0.5 * uVoxelSize);
    }
    if (tHit < 0.0)
        discard;

    vec3 hit = ro + rd * tHit;
    float e = 0.5 * uVoxelSize;
    vec3 gradient = vec3(Phi(hit + vec3(e, 0.0, 0.0)) - Phi(hit - vec3(e, 0.0, 0.0)),
                         Phi(hit + vec3(0.0, e, 0.0)) - Phi(hit - vec3(0.0, e, 0.0)),
                         Phi(hit + vec3(0.0, 0.0, e)) - Phi(hit - vec3(0.0, 0.0, e)));
    vec3 n = dot(gradient, gradient) > 1e-12 ? normalize(gradient) : -rd;

    // Thickness for Beer absorption: march on until the ray leaves the water or hits the scene.
    float exitT = tHit;
    for (int i = 0; i < kThicknessSteps; ++i)
    {
        exitT += 0.75 * uVoxelSize;
        if (exitT >= sceneT)
        {
            exitT = sceneT;
            break;
        }
        if (Phi(ro + rd * exitT) > 0.0)
            break;
    }
    float thickness = exitT - tHit;

    vec3 r = reflect(rd, n);
    vec3 reflection = uHasHDRI != 0 ? ToneMap(texture(uHDRITex, EquirectUV(RotateY(r, uHDRIRotation))).rgb)
                                    : kSkyColor;

    // Screen-space refraction: sample the lit frame copy, offset by the view-space normal.
    vec3 viewNormal = mat3(uView) * n;
    vec2 refractUV = clamp(vUV - viewNormal.xy * uRefraction, vec2(0.001), vec2(0.999));
    vec3 behind = texture(uSceneColor, refractUV).rgb;
    vec3 transmittance = exp(-(vec3(1.0) - uWaterColor) * uAbsorption * thickness);
    vec3 refraction = mix(uWaterColor * 0.6, behind, transmittance);

    float cosTheta = clamp(dot(n, -rd), 0.0, 1.0);
    float fresnel = kWaterF0 + (1.0 - kWaterF0) * pow(1.0 - cosTheta, 5.0);
    FragColor = vec4(mix(refraction, reflection, fresnel), 1.0);
}
