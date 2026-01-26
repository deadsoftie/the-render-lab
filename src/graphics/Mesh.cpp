#include "pch.h"
#include "graphics/Mesh.h"
#include <glad/glad.h>

Mesh::~Mesh()
{
    if (m_ebo)
        glDeleteBuffers(1, &m_ebo);
    if (m_vbo)
        glDeleteBuffers(1, &m_vbo);
    if (m_vao)
        glDeleteVertexArrays(1, &m_vao);
}

void Mesh::Create(const std::vector<float>& verts, const std::vector<unsigned int>& indices)
{
    m_indexCount = static_cast<int>(indices.size());

    glGenVertexArrays(1, &m_vao);
    glBindVertexArray(m_vao);

    glGenBuffers(1, &m_vbo);
    glBindBuffer(GL_ARRAY_BUFFER, m_vbo);
    glBufferData(GL_ARRAY_BUFFER,
                 static_cast<GLsizeiptr>(verts.size() * sizeof(float)),
                 verts.data(),
                 GL_STATIC_DRAW);

    glGenBuffers(1, &m_ebo);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, m_ebo);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER,
                 static_cast<GLsizeiptr>(indices.size() * sizeof(unsigned int)),
                 indices.data(),
                 GL_STATIC_DRAW);

    // layout: pos(3), nrm(3)
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float), static_cast<void*>(0));

    glEnableVertexAttribArray(1);
    glVertexAttribPointer(1,
                          3,
                          GL_FLOAT,
                          GL_FALSE,
                          6 * sizeof(float),
                          reinterpret_cast<void*>(3 * sizeof(float)));

    glBindVertexArray(0);
}

void Mesh::Draw() const
{
    glBindVertexArray(m_vao);
    glDrawElements(GL_TRIANGLES, m_indexCount, GL_UNSIGNED_INT, 0);
    glBindVertexArray(0);
}
