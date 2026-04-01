#pragma once
#include <string>
#include <glad/glad.h>

class Texture
{
   public:
    Texture() = default;
    ~Texture();

    bool LoadFromFile(const std::string& path, bool srgb = false, bool flipY = true);
    bool LoadHDR(const std::string& path);

    void Bind(unsigned slot = 0) const;
    static void Unbind(unsigned slot = 0);

    GLuint ID() const { return m_id; }
    int Width() const { return m_width; }
    int Height() const { return m_height; }

    void Destroy();

   private:
    GLuint m_id = 0;
    int m_width = 0;
    int m_height = 0;
};
