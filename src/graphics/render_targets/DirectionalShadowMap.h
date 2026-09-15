#pragma once

class DirectionalShadowMap
{
public:
    bool Create(int resolution);
    void Destroy();
    void BindForWriting();
    static void Unbind();
    unsigned int Tex() const { return m_tex; }
    int Resolution() const { return m_resolution; }

private:
    unsigned int m_fbo        = 0;
    unsigned int m_tex        = 0;
    int          m_resolution = 0;
};
