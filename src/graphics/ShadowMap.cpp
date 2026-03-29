#include "pch.h"
#include <glad/glad.h>
#include "graphics/ShadowMap.h"

bool ShadowMap::Create(int resolution)
{
    m_resolution = resolution;

    // Depth cubemap texture
    glGenTextures(1, &m_texCube);
    glBindTexture(GL_TEXTURE_CUBE_MAP, m_texCube);
    for (int face = 0; face < 6; ++face)
    {
        glTexImage2D(GL_TEXTURE_CUBE_MAP_POSITIVE_X + face,
                     0, GL_DEPTH_COMPONENT24,
                     resolution, resolution,
                     0, GL_DEPTH_COMPONENT, GL_FLOAT, nullptr);
    }
    glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
    glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
    glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_WRAP_R, GL_CLAMP_TO_EDGE);
    glBindTexture(GL_TEXTURE_CUBE_MAP, 0);

    // FBO — depth-only, no colour output
    glGenFramebuffers(1, &m_fbo);
    glBindFramebuffer(GL_FRAMEBUFFER, m_fbo);
    glDrawBuffer(GL_NONE);
    glReadBuffer(GL_NONE);
    glBindFramebuffer(GL_FRAMEBUFFER, 0);

    return true;
}

void ShadowMap::Destroy()
{
    if (m_texCube) { glDeleteTextures(1, &m_texCube);      m_texCube  = 0; }
    if (m_fbo)     { glDeleteFramebuffers(1, &m_fbo);       m_fbo      = 0; }
    m_resolution = 0;
}

void ShadowMap::BindForFace(int face)
{
    glBindFramebuffer(GL_FRAMEBUFFER, m_fbo);
    glFramebufferTexture2D(GL_FRAMEBUFFER,
                           GL_DEPTH_ATTACHMENT,
                           GL_TEXTURE_CUBE_MAP_POSITIVE_X + face,
                           m_texCube,
                           0);
}

void ShadowMap::Unbind()
{
    glBindFramebuffer(GL_FRAMEBUFFER, 0);
}
