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

    struct SubmeshRange
    {
        unsigned int indexStart = 0;  // start index (in indices)
        unsigned int indexCount = 0;  // number of indices
        glm::vec3 albedo{1, 1, 1};
    };

    // 5-wall Cornell box (no front wall), normals face inward.
    // parts order: 0=floor, 1=ceiling, 2=back, 3=left(red), 4=right(green)
    struct CornellMesh
    {
        MeshData mesh;
        SubmeshRange parts[5];
    };

    CornellMesh MakeCornellBox(glm::vec3 halfExtents = {1.0f, 1.0f, 1.0f});

    // Large plane under everything
    MeshData MakeGroundPlane(float halfSize = 10.0f, float y = -1.25f);

}  // namespace Geometry
