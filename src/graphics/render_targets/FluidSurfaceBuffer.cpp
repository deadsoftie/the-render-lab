#include "pch.h"
#include "graphics/render_targets/FluidSurfaceBuffer.h"

#include <algorithm>

namespace
{
    GLuint MakeTexture(GLenum internalFormat, GLenum format, GLenum type, int w, int h, GLenum filter)
    {
        GLuint tex = 0;
        glGenTextures(1, &tex);
        glBindTexture(GL_TEXTURE_2D, tex);
        glTexImage2D(GL_TEXTURE_2D, 0, internalFormat, w, h, 0, format, type, nullptr);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, filter);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, filter);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
        glBindTexture(GL_TEXTURE_2D, 0);
        return tex;
    }

    bool Complete(const char* name)
    {
        GLenum status = glCheckFramebufferStatus(GL_FRAMEBUFFER);
        if (status == GL_FRAMEBUFFER_COMPLETE)
            return true;
        std::cerr << "[FluidSurfaceBuffer] " << name << " FBO incomplete: 0x" << std::hex << status << std::dec
                  << "\n";
        return false;
    }
}

bool FluidSurfaceBuffer::Create(int w, int h)
{
    Destroy();

    m_w = std::max(1, w);
    m_h = std::max(1, h);
    m_thicknessW = std::max(1, m_w / 2);
    m_thicknessH = std::max(1, m_h / 2);

    for (GLuint& tex : m_texEyeDepth)
        tex = MakeTexture(GL_R32F, GL_RED, GL_FLOAT, m_w, m_h, GL_NEAREST);
    m_texDepth = MakeTexture(GL_DEPTH_COMPONENT32F, GL_DEPTH_COMPONENT, GL_FLOAT, m_w, m_h, GL_NEAREST);
    m_texNormal = MakeTexture(GL_RGBA16F, GL_RGBA, GL_FLOAT, m_w, m_h, GL_NEAREST);
    m_texThickness = MakeTexture(GL_R16F, GL_RED, GL_FLOAT, m_thicknessW, m_thicknessH, GL_LINEAR);
    m_texThicknessTemp = MakeTexture(GL_R16F, GL_RED, GL_FLOAT, m_thicknessW, m_thicknessH, GL_LINEAR);

    glGenFramebuffers(1, &m_depthFbo);
    glBindFramebuffer(GL_FRAMEBUFFER, m_depthFbo);
    glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, m_texEyeDepth[0], 0);
    glFramebufferTexture2D(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, GL_TEXTURE_2D, m_texDepth, 0);
    constexpr GLenum buf = GL_COLOR_ATTACHMENT0;
    glDrawBuffers(1, &buf);
    bool ok = Complete("depth");

    glGenFramebuffers(1, &m_thicknessFbo);
    glBindFramebuffer(GL_FRAMEBUFFER, m_thicknessFbo);
    glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, m_texThickness, 0);
    glDrawBuffers(1, &buf);
    ok = Complete("thickness") && ok;

    glBindFramebuffer(GL_FRAMEBUFFER, 0);
    if (!ok)
    {
        Destroy();
        return false;
    }
    return true;
}

void FluidSurfaceBuffer::Destroy()
{
    GLuint textures[] = {m_texEyeDepth[0], m_texEyeDepth[1], m_texDepth, m_texNormal, m_texThickness, m_texThicknessTemp};
    for (GLuint tex : textures)
        if (tex)
            glDeleteTextures(1, &tex);
    m_texEyeDepth[0] = m_texEyeDepth[1] = 0;
    m_texDepth = m_texNormal = m_texThickness = m_texThicknessTemp = 0;
    if (m_depthFbo)
        glDeleteFramebuffers(1, &m_depthFbo);
    if (m_thicknessFbo)
        glDeleteFramebuffers(1, &m_thicknessFbo);
    m_depthFbo = m_thicknessFbo = 0;
    m_w = m_h = m_thicknessW = m_thicknessH = 0;
}

bool FluidSurfaceBuffer::Resize(int w, int h)
{
    w = std::max(1, w);
    h = std::max(1, h);
    if (w == m_w && h == m_h && m_depthFbo != 0)
        return true;
    return Create(w, h);
}

void FluidSurfaceBuffer::BindDepthForWriting() const
{
    glBindFramebuffer(GL_FRAMEBUFFER, m_depthFbo);
    glViewport(0, 0, m_w, m_h);
}

void FluidSurfaceBuffer::BindThicknessForWriting() const
{
    glBindFramebuffer(GL_FRAMEBUFFER, m_thicknessFbo);
    glViewport(0, 0, m_thicknessW, m_thicknessH);
}

void FluidSurfaceBuffer::UnbindWriting()
{
    glBindFramebuffer(GL_FRAMEBUFFER, 0);
}
