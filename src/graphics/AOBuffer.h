#pragma once
#include <glad/glad.h>

// Single-channel R16F framebuffer used for the AO pass and both bilateral blur passes.
// Three instances are kept in the Renderer:
//   m_aoRawBuffer    — output of the Alchemy AO pass
//   m_aoBlurHBuffer  — output of the horizontal bilateral blur
//   m_aoBlurVBuffer  — output of the vertical bilateral blur (final AO)

class AOBuffer
{
   public:
    bool Create(int w, int h);
    void Destroy();

    void Resize(int w, int h);

    void BindForWriting() const;
    static void UnbindWriting();

    int Width()  const { return m_w; }
    int Height() const { return m_h; }

    GLuint TexAO() const { return m_tex; }

   private:
    GLuint m_fbo = 0;
    GLuint m_tex = 0;

    int m_w = 0;
    int m_h = 0;
};
