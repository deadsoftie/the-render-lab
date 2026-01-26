#pragma once
#include <vector>

namespace Geometry
{
    struct MeshData
    {
        std::vector<float> vertices;        // interleaved: pos(3) + nrm(3)
        std::vector<unsigned int> indices;  // triangles
    };

    // Creates a cube centered at origin with correct face normals.
    // Layout per vertex: position.xyz, normal.xyz
    MeshData MakeCube(float halfExtent = 0.5f);
}  // namespace Geometry
