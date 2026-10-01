#pragma once
#include <glad/glad.h>

// Single-channel R16F framebuffer; the Renderer keeps 3 instances for the Alchemy AO pass and its horizontal/vertical bilateral blur passes.
class AOBuffer
{
   public:
    bool Create(int w, int h);
    void Destroy();

    bool Resize(int w, int h);

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
