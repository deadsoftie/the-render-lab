#version 330 core

layout(location = 0) in vec3 aPos;
layout(location = 1) in vec3 aNrm;  // unused — present to match mesh vertex layout
layout(location = 2) in vec2 aUV;   // unused
layout(location = 3) in vec4 aBoneIDs;
layout(location = 4) in vec4 aBoneWeights;
layout(location = 5) in vec3 aTangent;  // unused

uniform mat4 uModel;
uniform mat4 uLightVP;

uniform mat4 uBoneMatrices[64];

out vec3 vWorldPos;

void main()
{
    mat4 skinMatrix = uBoneMatrices[int(aBoneIDs.x)] * aBoneWeights.x +
                       uBoneMatrices[int(aBoneIDs.y)] * aBoneWeights.y +
                       uBoneMatrices[int(aBoneIDs.z)] * aBoneWeights.z +
                       uBoneMatrices[int(aBoneIDs.w)] * aBoneWeights.w;

    vec4 world = uModel * skinMatrix * vec4(aPos, 1.0);
    vWorldPos = world.xyz;
    gl_Position = uLightVP * world;
}
