#pragma once

class ShadowMap
{
public:
    bool Create(int resolution);
    void Destroy();
    void BindForFace(int face);  // face 0-5 → GL_TEXTURE_CUBE_MAP_POSITIVE_X + face
    static void Unbind();
    unsigned int TexCube() const { return m_texCube; }
    int Resolution() const { return m_resolution; }

private:
    unsigned int m_fbo      = 0;
    unsigned int m_texCube  = 0;
    int          m_resolution = 0;
};
