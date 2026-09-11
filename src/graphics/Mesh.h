#pragma once
#include <vector>

class Mesh
{
   public:
    ~Mesh();

    Mesh() = default;
    Mesh(const Mesh&) = delete;
    Mesh& operator=(const Mesh&) = delete;

    void Create(const std::vector<float>& verts, const std::vector<unsigned int>& indices);
    void Draw() const;
    void DrawRange(unsigned int indexStart, unsigned int indexCount) const;

   private:
    unsigned int m_vao = 0;
    unsigned int m_vbo = 0;
    unsigned int m_ebo = 0;
    int m_indexCount = 0;
};
