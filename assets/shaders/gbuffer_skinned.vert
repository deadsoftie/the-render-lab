#version 330 core

layout(location=0) in vec3 aPos;
layout(location=1) in vec3 aNrm;
layout(location=2) in vec2 aUV;
layout(location=3) in vec4 aBoneIDs;
layout(location=4) in vec4 aBoneWeights;

uniform mat4 uModel;
uniform mat3 uNormalMatrix; // transpose(inverse(uModel)), computed on CPU
uniform mat4 uView;
uniform mat4 uProj;

uniform vec4 uBoneDQReal[64];
uniform vec4 uBoneDQDual[64];
uniform float uBoneScales[64];

out vec3 vWorldPos;
out vec3 vWorldNrm;
out vec2 vUV;

vec4 quatMul(vec4 a, vec4 b)
{
    return vec4(
        a.w*b.x + a.x*b.w + a.y*b.z - a.z*b.y,
        a.w*b.y - a.x*b.z + a.y*b.w + a.z*b.x,
        a.w*b.z + a.x*b.y - a.y*b.x + a.z*b.w,
        a.w*b.w - a.x*b.x - a.y*b.y - a.z*b.z
    );
}

vec4 quatConj(vec4 q)
{
    return vec4(-q.xyz, q.w);
}

vec3 quatRotate(vec4 q, vec3 v)
{
    vec3 t = 2.0 * cross(q.xyz, v);
    return v + q.w * t + cross(q.xyz, t);
}

void main()
{
    // Dual quaternion skinning: weighted-matrix-sum collapses volume at joints with sharply divergent bone orientations, so blend unit quaternions instead, sign-aligned against the first influence since q and -q are the same rotation but cancel out unaligned.
    int id0 = int(aBoneIDs.x);
    int id1 = int(aBoneIDs.y);
    int id2 = int(aBoneIDs.z);
    int id3 = int(aBoneIDs.w);

    vec4 r0 = uBoneDQReal[id0];
    vec4 r1 = uBoneDQReal[id1];
    vec4 r2 = uBoneDQReal[id2];
    vec4 r3 = uBoneDQReal[id3];

    float sign1 = dot(r1, r0) < 0.0 ? -1.0 : 1.0;
    float sign2 = dot(r2, r0) < 0.0 ? -1.0 : 1.0;
    float sign3 = dot(r3, r0) < 0.0 ? -1.0 : 1.0;

    vec4 real = r0 * aBoneWeights.x +
                r1 * (aBoneWeights.y * sign1) +
                r2 * (aBoneWeights.z * sign2) +
                r3 * (aBoneWeights.w * sign3);

    vec4 dual = uBoneDQDual[id0] * aBoneWeights.x +
                uBoneDQDual[id1] * (aBoneWeights.y * sign1) +
                uBoneDQDual[id2] * (aBoneWeights.z * sign2) +
                uBoneDQDual[id3] * (aBoneWeights.w * sign3);

    float scale = uBoneScales[id0] * aBoneWeights.x +
                  uBoneScales[id1] * aBoneWeights.y +
                  uBoneScales[id2] * aBoneWeights.z +
                  uBoneScales[id3] * aBoneWeights.w;

    float len = length(real);
    real /= len;
    dual /= len;

    vec3 translation = 2.0 * quatMul(dual, quatConj(real)).xyz;

    vec3 skinnedPos = quatRotate(real, aPos) * scale + translation;
    vec3 skinnedNrm = quatRotate(real, aNrm);

    vec4 world = uModel * vec4(skinnedPos, 1.0);
    vWorldPos  = world.xyz;
    vWorldNrm  = uNormalMatrix * skinnedNrm;
    vUV        = aUV;

    gl_Position = uProj * uView * world;
}
