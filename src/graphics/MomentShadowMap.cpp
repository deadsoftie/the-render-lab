#include "pch.h"
#include <glad/glad.h>
#include "graphics/MomentShadowMap.h"
#include "graphics/Shader.h"

bool MomentShadowMap::Create(int resolution)
{
    m_resolution = resolution;

    // Raw moment cubemap (RGBA32F)
    glGenTextures(1, &m_momentCube);
    glBindTexture(GL_TEXTURE_CUBE_MAP, m_momentCube);
    for (int face = 0; face < 6; ++face)
        glTexImage2D(GL_TEXTURE_CUBE_MAP_POSITIVE_X + face, 0, GL_RGBA32F,
                     resolution, resolution, 0, GL_RGBA, GL_FLOAT, nullptr);
    glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_WRAP_R, GL_CLAMP_TO_EDGE);
    glBindTexture(GL_TEXTURE_CUBE_MAP, 0);

    // Blurred moment cubemap (RGBA32F)
    glGenTextures(1, &m_blurredCube);
    glBindTexture(GL_TEXTURE_CUBE_MAP, m_blurredCube);
    for (int face = 0; face < 6; ++face)
        glTexImage2D(GL_TEXTURE_CUBE_MAP_POSITIVE_X + face, 0, GL_RGBA32F,
                     resolution, resolution, 0, GL_RGBA, GL_FLOAT, nullptr);
    glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_WRAP_R, GL_CLAMP_TO_EDGE);
    glBindTexture(GL_TEXTURE_CUBE_MAP, 0);

    // Ping-pong 2D texture for horizontal blur intermediate
    glGenTextures(1, &m_pingPong2D);
    glBindTexture(GL_TEXTURE_2D, m_pingPong2D);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA32F,
                 resolution, resolution, 0, GL_RGBA, GL_FLOAT, nullptr);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
    glBindTexture(GL_TEXTURE_2D, 0);

    // Depth renderbuffer for capture pass
    glGenRenderbuffers(1, &m_depthRBO);
    glBindRenderbuffer(GL_RENDERBUFFER, m_depthRBO);
    glRenderbufferStorage(GL_RENDERBUFFER, GL_DEPTH_COMPONENT24, resolution, resolution);
    glBindRenderbuffer(GL_RENDERBUFFER, 0);

    // Capture FBO (color attachment set per-face in BindForCapture)
    glGenFramebuffers(1, &m_captureFBO);
    glBindFramebuffer(GL_FRAMEBUFFER, m_captureFBO);
    glFramebufferRenderbuffer(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, GL_RENDERBUFFER, m_depthRBO);
    glBindFramebuffer(GL_FRAMEBUFFER, 0);

    return true;
}

void MomentShadowMap::Destroy()
{
    if (m_momentCube)  { glDeleteTextures(1, &m_momentCube);           m_momentCube  = 0; }
    if (m_blurredCube) { glDeleteTextures(1, &m_blurredCube);          m_blurredCube = 0; }
    if (m_pingPong2D)  { glDeleteTextures(1, &m_pingPong2D);           m_pingPong2D  = 0; }
    if (m_depthRBO)    { glDeleteRenderbuffers(1, &m_depthRBO);        m_depthRBO    = 0; }
    if (m_captureFBO)  { glDeleteFramebuffers(1, &m_captureFBO);       m_captureFBO  = 0; }
    m_resolution = 0;
}

void MomentShadowMap::BindForCapture(int face)
{
    glBindFramebuffer(GL_FRAMEBUFFER, m_captureFBO);
    glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0,
                           GL_TEXTURE_CUBE_MAP_POSITIVE_X + face,
                           m_momentCube, 0);
    static const GLenum draw = GL_COLOR_ATTACHMENT0;
    glDrawBuffers(1, &draw);
}

void MomentShadowMap::Unbind()
{
    glBindFramebuffer(GL_FRAMEBUFFER, 0);
}

void MomentShadowMap::Blur(int face, Shader& hComp, Shader& vComp, float blurStep)
{
    const int groups = (m_resolution + 7) / 8;

    // H-pass: read from moment cubemap, write to ping-pong 2D
    hComp.Bind();
    glActiveTexture(GL_TEXTURE0);
    glBindTexture(GL_TEXTURE_CUBE_MAP, m_momentCube);
    hComp.SetInt("uMomentCube", 0);
    glBindImageTexture(0, m_pingPong2D, 0, GL_FALSE, 0, GL_WRITE_ONLY, GL_RGBA32F);
    hComp.SetInt("uFace", face);
    hComp.SetInt("uResolution", m_resolution);
    hComp.SetFloat("uBlurStep", blurStep);
    hComp.Dispatch(groups, groups, 1);
    glMemoryBarrier(GL_TEXTURE_FETCH_BARRIER_BIT);

    // V-pass: read from ping-pong 2D, write to blurred cubemap face
    vComp.Bind();
    glActiveTexture(GL_TEXTURE0);
    glBindTexture(GL_TEXTURE_2D, m_pingPong2D);
    vComp.SetInt("uPingPong", 0);
    glBindImageTexture(0, m_blurredCube, 0, GL_TRUE, face, GL_WRITE_ONLY, GL_RGBA32F);
    vComp.SetInt("uFace", face);
    vComp.SetInt("uResolution", m_resolution);
    vComp.SetFloat("uBlurStep", blurStep);
    vComp.Dispatch(groups, groups, 1);
}
