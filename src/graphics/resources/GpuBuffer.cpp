#include "pch.h"
#include "graphics/resources/GpuBuffer.h"

#include <glad/glad.h>

GpuBuffer::~GpuBuffer()
{
    Destroy();
}

bool GpuBuffer::Resize(size_t bytes, bool preserve)
{
    if (bytes <= m_capacity && m_id != 0)
        return true;

    GLuint id = 0;
    glGenBuffers(1, &id);
    if (id == 0)
        return false;
    glBindBuffer(GL_COPY_WRITE_BUFFER, id);
    glBufferData(GL_COPY_WRITE_BUFFER, static_cast<GLsizeiptr>(bytes), nullptr, GL_DYNAMIC_DRAW);

    if (preserve && m_id != 0 && m_capacity > 0)
    {
        glBindBuffer(GL_COPY_READ_BUFFER, m_id);
        glCopyBufferSubData(GL_COPY_READ_BUFFER, GL_COPY_WRITE_BUFFER, 0, 0, static_cast<GLsizeiptr>(m_capacity));
        glBindBuffer(GL_COPY_READ_BUFFER, 0);
    }
    glBindBuffer(GL_COPY_WRITE_BUFFER, 0);

    Destroy();
    m_id = id;
    m_capacity = bytes;
    return true;
}

void GpuBuffer::Upload(const void* data, size_t bytes, size_t offset)
{
    if (m_id == 0 || bytes == 0 || offset + bytes > m_capacity)
        return;
    glBindBuffer(GL_COPY_WRITE_BUFFER, m_id);
    glBufferSubData(GL_COPY_WRITE_BUFFER, static_cast<GLintptr>(offset), static_cast<GLsizeiptr>(bytes), data);
    glBindBuffer(GL_COPY_WRITE_BUFFER, 0);
}

void GpuBuffer::Download(void* data, size_t bytes, size_t offset) const
{
    if (m_id == 0 || bytes == 0 || offset + bytes > m_capacity)
        return;
    glMemoryBarrier(GL_BUFFER_UPDATE_BARRIER_BIT);
    glBindBuffer(GL_COPY_READ_BUFFER, m_id);
    glGetBufferSubData(GL_COPY_READ_BUFFER, static_cast<GLintptr>(offset), static_cast<GLsizeiptr>(bytes), data);
    glBindBuffer(GL_COPY_READ_BUFFER, 0);
}

void GpuBuffer::Clear()
{
    if (m_id == 0)
        return;
    const GLuint zero = 0;
    glBindBuffer(GL_COPY_WRITE_BUFFER, m_id);
    glClearBufferData(GL_COPY_WRITE_BUFFER, GL_R32UI, GL_RED_INTEGER, GL_UNSIGNED_INT, &zero);
    glBindBuffer(GL_COPY_WRITE_BUFFER, 0);
}

void GpuBuffer::BindBase(unsigned int target, unsigned int binding) const
{
    glBindBufferBase(target, binding, m_id);
}

void GpuBuffer::Destroy()
{
    if (m_id != 0)
    {
        glDeleteBuffers(1, &m_id);
        m_id = 0;
    }
    m_capacity = 0;
}
