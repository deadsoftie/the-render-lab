#version 330 core

layout(location=0) in vec3 aCorner;
layout(location=1) in int aLevel;
layout(location=2) in ivec3 aCoord;

uniform mat4 uView;
uniform mat4 uProj;
uniform vec3 uOrigin;
uniform float uVoxelSize;
uniform int uCoordOffset;
uniform int uLog2Dim;
uniform int uShowLeaves;
uniform int uShowInterior;

out vec3 vWorldPos;
flat out int vLevel;

void main()
{
    bool visible = aLevel >= 0 && (aLevel == 0 ? uShowLeaves != 0 : uShowInterior != 0);
    int level = max(aLevel, 0);
    float size = float(1 << (uLog2Dim * (level + 1)));
    vec3 voxelMin = vec3(aCoord) * size - float(uCoordOffset);
    vWorldPos = uOrigin + (voxelMin + aCorner * size) * uVoxelSize;
    vLevel = level;
    gl_Position = visible ? uProj * uView * vec4(vWorldPos, 1.0) : vec4(2.0, 2.0, 2.0, 1.0);
}
