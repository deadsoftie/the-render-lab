#pragma once
#include <vector>

class SkinnedMesh
{
   public:
    ~SkinnedMesh();

    SkinnedMesh() = default;
    SkinnedMesh(const SkinnedMesh&) = delete;
    SkinnedMesh& operator=(const SkinnedMesh&) = delete;

    // Interleaved layout: pos(3) + nrm(3) + uv(2) + boneIDs(4, float-encoded) + boneWeights(4) + tan(3).
    void Create(const std::vector<float>& verts, const std::vector<unsigned int>& indices);
    void Draw() const;
    void DrawRange(unsigned int indexStart, unsigned int indexCount) const;

   private:
    unsigned int m_vao = 0;
    unsigned int m_vbo = 0;
    unsigned int m_ebo = 0;
    int m_indexCount = 0;
};
