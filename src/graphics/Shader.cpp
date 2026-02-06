#include "pch.h"
#include "graphics/Shader.h"

#include <fstream>
#include <sstream>
#include <iostream>

#include <glad/glad.h>
#include <glm/gtc/type_ptr.hpp>

Shader::~Shader()
{
    if (m_program)
        glDeleteProgram(m_program);
}

std::string Shader::ReadTextFile(const std::string& path)
{
    std::ifstream f(path, std::ios::binary);
    if (!f.is_open())
        return {};

    std::ostringstream ss;
    ss << f.rdbuf();
    return ss.str();
}

unsigned int Shader::Compile(unsigned int type, const std::string& src)
{
    unsigned int s = glCreateShader(type);
    const char* c = src.c_str();
    glShaderSource(s, 1, &c, nullptr);
    glCompileShader(s);

    int ok = 0;
    glGetShaderiv(s, GL_COMPILE_STATUS, &ok);

    if (!ok)
    {
        int len = 0;
        glGetShaderiv(s, GL_INFO_LOG_LENGTH, &len);
        std::string log(len, '\0');
        glGetShaderInfoLog(s, len, nullptr, log.data());
        std::cerr << "[Shader] Compile failed:\n" << log << "\n";
        glDeleteShader(s);
        return 0;
    }

    return s;
}

unsigned int Shader::Link(unsigned int vs, unsigned int fs)
{
    unsigned int p = glCreateProgram();
    glAttachShader(p, vs);
    glAttachShader(p, fs);
    glLinkProgram(p);

    int ok = 0;
    glGetProgramiv(p, GL_LINK_STATUS, &ok);

    if (!ok)
    {
        int len = 0;
        glGetProgramiv(p, GL_INFO_LOG_LENGTH, &len);
        std::string log(len, '\0');
        glGetProgramInfoLog(p, len, nullptr, log.data());
        std::cerr << "[Shader] Link failed:\n" << log << "\n";
        glDeleteProgram(p);
        return 0;
    }

    return p;
}

bool Shader::LoadFromFiles(const std::string& vsPath, const std::string& fsPath)
{
    const std::string vsSrc = ReadTextFile(vsPath);
    const std::string fsSrc = ReadTextFile(fsPath);
    if (vsSrc.empty() || fsSrc.empty())
    {
        std::cerr << "[Shader] Failed to read shader files:\n"
                  << "  VS: " << vsPath << "\n"
                  << "  FS: " << fsPath << "\n";
        return false;
    }

    unsigned int vs = Compile(GL_VERTEX_SHADER, vsSrc);
    unsigned int fs = Compile(GL_FRAGMENT_SHADER, fsSrc);
    if (!vs || !fs)
        return false;

    unsigned int prog = Link(vs, fs);
    glDeleteShader(vs);
    glDeleteShader(fs);

    if (!prog)
        return false;

    if (m_program)
        glDeleteProgram(m_program);
    m_program = prog;
    return true;
}

void Shader::Bind() const
{
    glUseProgram(m_program);
}
void Shader::Unbind() const
{
    glUseProgram(0);
}

int Shader::GetLocation(const char* name) const
{
    return glGetUniformLocation(m_program, name);
}

void Shader::SetMat4(const char* name, const glm::mat4& m) const
{
    glUniformMatrix4fv(GetLocation(name), 1, GL_FALSE, glm::value_ptr(m));
}

void Shader::SetVec2(const char* name, const glm::vec2& v) const
{
    glUniform2fv(GetLocation(name), 1, glm::value_ptr(v));
}

void Shader::SetVec3(const char* name, const glm::vec3& v) const
{
    glUniform3fv(GetLocation(name), 1, glm::value_ptr(v));
}

void Shader::SetVec3Array(const char* name, const glm::vec3* v, int count) const
{
    glUniform3fv(GetLocation(name), count, glm::value_ptr(v[0]));
}

void Shader::SetFloat(const char* name, float f) const
{
    glUniform1f(GetLocation(name), f);
}

void Shader::SetInt(const char* name, int v) const
{
    glUniform1i(GetLocation(name), v);
}