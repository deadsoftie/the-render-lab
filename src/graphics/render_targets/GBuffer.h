#pragma once
#include <glad/glad.h>

// Simple MRT G-Buffer for deferred shading.
// Layout (by attachment):
//  0: World Position (RGBA16F)
//  1: World Normal   (RGBA16F)
//  2: Kd (diffuse/albedo) (RGBA8)
//  3: Ks (F0) + alpha (roughness exponent) (RGBA16F)  (rgb = Ks/F0, a = Phong alpha 1..256)

class GBuffer
{
   public:
    bool Create(int w, int h);
    void Destroy();

    void Resize(int w, int h);

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