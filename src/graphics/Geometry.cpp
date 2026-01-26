#include "pch.h"
#include "graphics/Geometry.h"

namespace Geometry
{
    // 24 unique vertices (4 per face) so normals are correct per face.
    // Indices: 36 (12 triangles).
    MeshData MakeCube(float h)
    {
        MeshData out;
        out.vertices.reserve(24 * 6);
        out.indices.reserve(36);

        auto pushVertex = [&](float px, float py, float pz, float nx, float ny, float nz)
        {
            out.vertices.push_back(px);
            out.vertices.push_back(py);
            out.vertices.push_back(pz);
            out.vertices.push_back(nx);
            out.vertices.push_back(ny);
            out.vertices.push_back(nz);
        };

        // Face order: +Z, -Z, +X, -X, +Y, -Y
        // Each face: 4 verts (CCW) with constant normal

        // +Z (front)
        pushVertex(-h, -h, +h, 0, 0, 1);
        pushVertex(+h, -h, +h, 0, 0, 1);
        pushVertex(+h, +h, +h, 0, 0, 1);
        pushVertex(-h, +h, +h, 0, 0, 1);

        // -Z (back)
        pushVertex(+h, -h, -h, 0, 0, -1);
        pushVertex(-h, -h, -h, 0, 0, -1);
        pushVertex(-h, +h, -h, 0, 0, -1);
        pushVertex(+h, +h, -h, 0, 0, -1);

        // +X (right)
        pushVertex(+h, -h, +h, 1, 0, 0);
        pushVertex(+h, -h, -h, 1, 0, 0);
        pushVertex(+h, +h, -h, 1, 0, 0);
        pushVertex(+h, +h, +h, 1, 0, 0);

        // -X (left)
        pushVertex(-h, -h, -h, -1, 0, 0);
        pushVertex(-h, -h, +h, -1, 0, 0);
        pushVertex(-h, +h, +h, -1, 0, 0);
        pushVertex(-h, +h, -h, -1, 0, 0);

        // +Y (top)
        pushVertex(-h, +h, +h, 0, 1, 0);
        pushVertex(+h, +h, +h, 0, 1, 0);
        pushVertex(+h, +h, -h, 0, 1, 0);
        pushVertex(-h, +h, -h, 0, 1, 0);

        // -Y (bottom)
        pushVertex(-h, -h, -h, 0, -1, 0);
        pushVertex(+h, -h, -h, 0, -1, 0);
        pushVertex(+h, -h, +h, 0, -1, 0);
        pushVertex(-h, -h, +h, 0, -1, 0);

        // Indices for each face (two triangles): (0,1,2) (0,2,3)
        for (unsigned int face = 0; face < 6; ++face)
        {
            unsigned int base = face * 4;
            out.indices.push_back(base + 0);
            out.indices.push_back(base + 1);
            out.indices.push_back(base + 2);

            out.indices.push_back(base + 0);
            out.indices.push_back(base + 2);
            out.indices.push_back(base + 3);
        }

        return out;
    }
}  // namespace Geometry
