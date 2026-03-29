#pragma once

class Shader;

class MomentShadowMap
{
public:
    bool Create(int resolution);
    void Destroy();
    void BindForCapture(int face);  // binds capture FBO with cube face as color attachment
    static void Unbind();
    void Blur(int face, Shader& hComp, Shader& vComp, float blurStep);
    unsigned int TexBlurred() const { return m_blurredCube; }
    int Resolution() const { return m_resolution; }

private:
    unsigned int m_momentCube  = 0;  // RGBA32F cubemap for raw moments
    unsigned int m_blurredCube = 0;  // RGBA32F cubemap for blurred moments
    unsigned int m_pingPong2D  = 0;  // RGBA32F 2D texture for H-blur intermediate
    unsigned int m_depthRBO    = 0;  // depth renderbuffer for capture pass
    unsigned int m_captureFBO  = 0;  // FBO used during moment capture
    int          m_resolution  = 0;
};
