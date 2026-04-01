#include "pch.h"
#include "graphics/Texture.h"

#define STB_IMAGE_IMPLEMENTATION
#include <stb_image.h>

#include <iostream>

Texture::~Texture()
{
    Destroy();
}

bool Texture::LoadFromFile(const std::string& path, bool srgb, bool flipY)
{
    Destroy();

    stbi_set_flip_vertically_on_load(flipY);

    int channels = 0;
    unsigned char* data = stbi_load(path.c_str(), &m_width, &m_height, &channels, 0);

    if (!data)
    {
        std::cerr << "[Texture] Failed to load: " << path << "\n";
        return false;
    }

    GLenum format = GL_RGB;
    GLenum internal = GL_RGB8;

    if (channels == 1)
    {
        format = GL_RED;
        internal = GL_R8;
    }
    else if (channels == 3)
    {
        format = GL_RGB;
        internal = srgb ? GL_SRGB8 : GL_RGB8;
    }
    else if (channels == 4)
    {
        format = GL_RGBA;
        internal = srgb ? GL_SRGB8_ALPHA8 : GL_RGBA8;
    }
    else
    {
        std::cerr << "[Texture] Unsupported channel count\n";
        stbi_image_free(data);
        return false;
    }

    glGenTextures(1, &m_id);
    glBindTexture(GL_TEXTURE_2D, m_id);

    glTexImage2D(GL_TEXTURE_2D, 0, internal, m_width, m_height, 0, format, GL_UNSIGNED_BYTE, data);

    glGenerateMipmap(GL_TEXTURE_2D);

    // Reasonable defaults
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);

    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);

    glBindTexture(GL_TEXTURE_2D, 0);
    stbi_image_free(data);

    return true;
}

bool Texture::LoadHDR(const std::string& path)
{
    Destroy();

    // Equirectangular HDR maps must NOT be flipped: our uvOf maps Y=+1 (up)
    // to v=0, which must correspond to the top row of the file (sky).
    stbi_set_flip_vertically_on_load(false);

    int channels = 0;
    float* data = stbi_loadf(path.c_str(), &m_width, &m_height, &channels, 3);

    if (!data)
    {
        std::cerr << "[Texture] Failed to load HDR: " << path << "\n";
        return false;
    }

    glGenTextures(1, &m_id);
    glBindTexture(GL_TEXTURE_2D, m_id);

    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB16F, m_width, m_height, 0, GL_RGB, GL_FLOAT, data);

    glGenerateMipmap(GL_TEXTURE_2D);

    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    // Repeat horizontally so the longitude wrap is seamless
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);

    glBindTexture(GL_TEXTURE_2D, 0);
    stbi_image_free(data);

    return true;
}

void Texture::Bind(unsigned slot) const
{
    glActiveTexture(GL_TEXTURE0 + slot);
    glBindTexture(GL_TEXTURE_2D, m_id);
}

void Texture::Unbind(unsigned slot)
{
    glActiveTexture(GL_TEXTURE0 + slot);
    glBindTexture(GL_TEXTURE_2D, 0);
}

void Texture::Destroy()
{
    if (m_id)
    {
        glDeleteTextures(1, &m_id);
        m_id = 0;
    }
    m_width = m_height = 0;
}
