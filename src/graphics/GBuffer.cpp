#include "pch.h"
#include "graphics/GBuffer.h"

#include <algorithm>

static GLuint MakeColorTex16F(int w, int h, GLenum internalFmt, GLenum fmt, GLenum type)
{
    GLuint t = 0;
    glGenTextures(1, &t);
    glBindTexture(GL_TEXTURE_2D, t);
    glTexImage2D(GL_TEXTURE_2D, 0, internalFmt, w, h, 0, fmt, type, nullptr);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
    glBindTexture(GL_TEXTURE_2D, 0);
    return t;
}

static GLuint MakeColorTex8(int w, int h, GLenum internalFmt, GLenum fmt, GLenum type)
{
    GLuint t = 0;
    glGenTextures(1, &t);
    glBindTexture(GL_TEXTURE_2D, t);
    glTexImage2D(GL_TEXTURE_2D, 0, internalFmt, w, h, 0, fmt, type, nullptr);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
    glBindTexture(GL_TEXTURE_2D, 0);
    return t;
}

bool GBuffer::Create(int w, int h)
{
    Destroy();

    m_w = std::max(1, w);
    m_h = std::max(1, h);

    glGenFramebuffers(1, &m_fbo);
    glBindFramebuffer(GL_FRAMEBUFFER, m_fbo);

    CreateAttachments();

    GLenum status = glCheckFramebufferStatus(GL_FRAMEBUFFER);
    glBindFramebuffer(GL_FRAMEBUFFER, 0);

    if (status != GL_FRAMEBUFFER_COMPLETE)
    {
        std::cerr << "[GBuffer] FBO incomplete: 0x" << std::hex << status << std::dec << "\n";
        Destroy();
        return false;
    }

    return true;
}

void GBuffer::Destroy()
{
    if (m_depthRbo)
    {
        glDeleteRenderbuffers(1, &m_depthRbo);
        m_depthRbo = 0;
    }
    if (m_texWorldPos)
    {
        glDeleteTextures(1, &m_texWorldPos);
        m_texWorldPos = 0;
    }
    if (m_texNormal)
    {
        glDeleteTextures(1, &m_texNormal);
        m_texNormal = 0;
    }
    if (m_texKd)
    {
        glDeleteTextures(1, &m_texKd);
        m_texKd = 0;
    }
    if (m_texKsAlpha)
    {
        glDeleteTextures(1, &m_texKsAlpha);
        m_texKsAlpha = 0;
    }
    if (m_fbo)
    {
        glDeleteFramebuffers(1, &m_fbo);
        m_fbo = 0;
    }
    m_w = m_h = 0;
}

void GBuffer::Resize(int w, int h)
{
    w = std::max(1, w);
    h = std::max(1, h);
    if (w == m_w && h == m_h && m_fbo != 0)
        return;

    Create(w, h);
}

void GBuffer::BindForWriting() const
{
    glBindFramebuffer(GL_FRAMEBUFFER, m_fbo);
}

void GBuffer::UnbindWriting()
{
    glBindFramebuffer(GL_FRAMEBUFFER, 0);
}

void GBuffer::CreateAttachments()
{
    m_texWorldPos = MakeColorTex16F(m_w, m_h, GL_RGBA16F, GL_RGBA, GL_FLOAT);
    m_texNormal = MakeColorTex16F(m_w, m_h, GL_RGBA16F, GL_RGBA, GL_FLOAT);
    m_texKd = MakeColorTex8(m_w, m_h, GL_RGBA8, GL_RGBA, GL_UNSIGNED_BYTE);
    m_texKsAlpha = MakeColorTex16F(m_w, m_h, GL_RGBA16F, GL_RGBA, GL_FLOAT);

    glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, m_texWorldPos, 0);
    glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT1, GL_TEXTURE_2D, m_texNormal, 0);
    glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT2, GL_TEXTURE_2D, m_texKd, 0);
    glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT3, GL_TEXTURE_2D, m_texKsAlpha, 0);

    glGenRenderbuffers(1, &m_depthRbo);
    glBindRenderbuffer(GL_RENDERBUFFER, m_depthRbo);
    glRenderbufferStorage(GL_RENDERBUFFER, GL_DEPTH_COMPONENT24, m_w, m_h);
    glFramebufferRenderbuffer(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, GL_RENDERBUFFER, m_depthRbo);
    glBindRenderbuffer(GL_RENDERBUFFER, 0);

    constexpr GLenum bufs[] = {GL_COLOR_ATTACHMENT0,
                               GL_COLOR_ATTACHMENT1,
                               GL_COLOR_ATTACHMENT2,
                               GL_COLOR_ATTACHMENT3};
    glDrawBuffers(4, bufs);
}