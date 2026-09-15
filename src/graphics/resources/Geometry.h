#pragma once
#include <string>
#include <vector>

namespace Geometry
{
    struct MeshData
    {
        std::vector<float> vertices;        // interleaved: pos(3) + nrm(3) + uv(2) + tan(3)
        std::vector<unsigned int> indices;  // triangles
    };

    // Appends a tangent vector (UV-gradient method) to the end of each vertex in an
    // interleaved pos(3)+nrm(3)+uv(2)+[...] buffer, growing its stride by 3 floats.
    // Position/normal/uv are always at offsets 0/3/6 regardless of what follows (bone
    // data, etc.), so this works for both MeshData (strideFloats=8) and
    // SkinnedMeshData (strideFloats=16).
    void AppendTangents(std::vector<float>& vertices, const std::vector<unsigned int>& indices,
                        size_t strideFloats);

    // Creates a cube centered at origin with correct face normals.
    // Layout per vertex: position.xyz, normal.xyz, uv.xy
    MeshData MakeCube(float halfExtent = 0.5f);

    struct SubmeshRange
    {
        unsigned int indexStart = 0;  // start index (in indices)
        unsigned int indexCount = 0;  // number of indices
        glm::vec3 albedo{1, 1, 1};
        std::string albedoTexture;    // empty = no texture, use albedo color
        std::string specularTexture;  // empty = no texture, use material ks
        std::string roughnessTexture; // empty = no texture, use material alpha
        std::string metallicTexture;  // empty = no texture, use material metallic
        std::string normalTexture;    // empty = no texture, use geometric normal
    };

    struct SkinnedMeshData
    {
        // interleaved: pos(3) + nrm(3) + uv(2) + boneIDs(4, float-encoded) + boneWeights(4) + tan(3)
        std::vector<float> vertices;
        std::vector<unsigned int> indices;
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

    // UV sphere centered at origin.
    // Layout per vertex: position.xyz, normal.xyz, uv.xy
    MeshData MakeSphere(float radius = 0.5f, int slices = 32, int stacks = 16);
}  // namespace Geometry
