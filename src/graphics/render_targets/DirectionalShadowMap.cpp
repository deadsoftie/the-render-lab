#include "pch.h"
#include <glad/glad.h>
#include "graphics/render_targets/DirectionalShadowMap.h"

bool DirectionalShadowMap::Create(int resolution)
{
    m_resolution = resolution;

    glGenTextures(1, &m_tex);
    glBindTexture(GL_TEXTURE_2D, m_tex);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_DEPTH_COMPONENT24,
                resolution, resolution,
                0, GL_DEPTH_COMPONENT, GL_FLOAT, nullptr);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_BORDER);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_BORDER);
    // Border depth = 1.0 (far) so sampling outside the ortho frustum reads as unshadowed,
    // no manual bounds check needed at the call site.
    const float borderColor[4] = {1.0f, 1.0f, 1.0f, 1.0f};
    glTexParameterfv(GL_TEXTURE_2D, GL_TEXTURE_BORDER_COLOR, borderColor);
    glBindTexture(GL_TEXTURE_2D, 0);

    glGenFramebuffers(1, &m_fbo);
    glBindFramebuffer(GL_FRAMEBUFFER, m_fbo);
    glFramebufferTexture2D(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, GL_TEXTURE_2D, m_tex, 0);
    glDrawBuffer(GL_NONE);
    glReadBuffer(GL_NONE);
    glBindFramebuffer(GL_FRAMEBUFFER, 0);

    return true;
}

void DirectionalShadowMap::Destroy()
{
    if (m_tex) { glDeleteTextures(1, &m_tex);      m_tex = 0; }
    if (m_fbo) { glDeleteFramebuffers(1, &m_fbo);  m_fbo = 0; }
    m_resolution = 0;
}

void DirectionalShadowMap::BindForWriting()
{
    glBindFramebuffer(GL_FRAMEBUFFER, m_fbo);
}

void DirectionalShadowMap::Unbind()
{
    glBindFramebuffer(GL_FRAMEBUFFER, 0);
}
