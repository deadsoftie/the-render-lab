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

uniform mat4 uBoneMatrices[64];

out vec3 vWorldPos;
out vec3 vWorldNrm;
out vec2 vUV;

void main()
{
    mat4 skinMatrix = uBoneMatrices[int(aBoneIDs.x)] * aBoneWeights.x +
                       uBoneMatrices[int(aBoneIDs.y)] * aBoneWeights.y +
                       uBoneMatrices[int(aBoneIDs.z)] * aBoneWeights.z +
                       uBoneMatrices[int(aBoneIDs.w)] * aBoneWeights.w;

    vec4 skinnedPos = skinMatrix * vec4(aPos, 1.0);
    // Bone matrices only ever carry uniform scale (VQS), so the skin matrix's
    // 3x3 block is angle-preserving and safe to use directly for normals.
    vec3 skinnedNrm = mat3(skinMatrix) * aNrm;

    vec4 world = uModel * skinnedPos;
    vWorldPos  = world.xyz;
    vWorldNrm  = uNormalMatrix * skinnedNrm;
    vUV        = aUV;

    gl_Position = uProj * uView * world;
}
