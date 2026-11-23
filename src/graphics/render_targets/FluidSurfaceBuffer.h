#pragma once
#include <glad/glad.h>

class FluidSurfaceBuffer
{
   public:
    bool Create(int w, int h);
    void Destroy();

    bool Resize(int w, int h);

    void BindDepthForWriting() const;
    void BindThicknessForWriting() const;
    static void UnbindWriting();

    int Width() const { return m_w; }
    int Height() const { return m_h; }
    int ThicknessWidth() const { return m_thicknessW; }
    int ThicknessHeight() const { return m_thicknessH; }

    GLuint TexEyeDepth(int index = 0) const { return m_texEyeDepth[index]; }
    GLuint TexThickness() const { return m_texThickness; }
    GLuint TexThicknessTemp() const { return m_texThicknessTemp; }
    GLuint TexNormal() const { return m_texNormal; }
    GLuint TexDepthStencil() const { return m_texDepth; }

   private:
    GLuint m_depthFbo = 0;
    GLuint m_thicknessFbo = 0;
    GLuint m_texEyeDepth[2] = {0, 0};
    GLuint m_texDepth = 0;
    GLuint m_texThickness = 0;
    GLuint m_texThicknessTemp = 0;
    GLuint m_texNormal = 0;

    int m_w = 0;
    int m_h = 0;
    int m_thicknessW = 0;
    int m_thicknessH = 0;
};
