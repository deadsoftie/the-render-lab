#pragma once
#include <glad/glad.h>

// MRT G-Buffer: attachment 0 world pos, 1 world normal (RGBA16F), 2 albedo+metallic (RGBA8), 3 Ks/F0+roughness (RGBA16F).
class GBuffer
{
   public:
    bool Create(int w, int h);
    void Destroy();

    bool Resize(int w, int h);

    void BindForWriting() const;
    static void UnbindWriting();

    int Width() const { return m_w; }
    int Height() const { return m_h; }

    GLuint TexWorldPos() const { return m_texWorldPos; }
    GLuint TexNormal() const { return m_texNormal; }
    GLuint TexKd() const { return m_texKd; }
    GLuint TexKsAlpha() const { return m_texKsAlpha; }

   private:
    void CreateAttachments();

    GLuint m_fbo = 0;
    GLuint m_texWorldPos = 0;
    GLuint m_texNormal = 0;
    GLuint m_texKd = 0;
    GLuint m_texKsAlpha = 0;
    GLuint m_depthRbo = 0;

    int m_w = 0;
    int m_h = 0;
};