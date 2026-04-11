#include "pch.h"
#include "graphics/AOBuffer.h"

#include <algorithm>

bool AOBuffer::Create(int w, int h)
{
    Destroy();

    m_w = std::max(1, w);
    m_h = std::max(1, h);

    glGenFramebuffers(1, &m_fbo);
    glBindFramebuffer(GL_FRAMEBUFFER, m_fbo);

    glGenTextures(1, &m_tex);
    glBindTexture(GL_TEXTURE_2D, m_tex);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_R16F, m_w, m_h, 0, GL_RED, GL_FLOAT, nullptr);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
    glBindTexture(GL_TEXTURE_2D, 0);

    glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, m_tex, 0);

    constexpr GLenum buf = GL_COLOR_ATTACHMENT0;
    glDrawBuffers(1, &buf);

    GLenum status = glCheckFramebufferStatus(GL_FRAMEBUFFER);
    glBindFramebuffer(GL_FRAMEBUFFER, 0);

    if (status != GL_FRAMEBUFFER_COMPLETE)
    {
        std::cerr << "[AOBuffer] FBO incomplete: 0x" << std::hex << status << std::dec << "\n";
        Destroy();
        return false;
    }

    return true;
}

void AOBuffer::Destroy()
{
    if (m_tex)
    {
        glDeleteTextures(1, &m_tex);
        m_tex = 0;
    }
    if (m_fbo)
    {
        glDeleteFramebuffers(1, &m_fbo);
        m_fbo = 0;
    }
    m_w = m_h = 0;
}

void AOBuffer::Resize(int w, int h)
{
    w = std::max(1, w);
    h = std::max(1, h);
    if (w == m_w && h == m_h && m_fbo != 0)
        return;

    Create(w, h);
}

void AOBuffer::BindForWriting() const
{
    glBindFramebuffer(GL_FRAMEBUFFER, m_fbo);
}

void AOBuffer::UnbindWriting()
{
    glBindFramebuffer(GL_FRAMEBUFFER, 0);
}
