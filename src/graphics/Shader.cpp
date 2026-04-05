#include "pch.h"
#include "graphics/Shader.h"

#include <fstream>
#include <sstream>
#include <iostream>

#include <glad/glad.h>
#include <glm/gtc/type_ptr.hpp>

// ---------------------------------------------------------------------------
// Shader include preprocessor
// ---------------------------------------------------------------------------
static std::string ReadFile(const std::string& path)
{
    std::ifstream f(path, std::ios::binary);
    if (!f.is_open())
        return {};
    std::ostringstream ss;
    ss << f.rdbuf();
    return ss.str();
}

static std::string DirectoryOf(const std::string& path)
{
    size_t pos = path.find_last_of("/\\");
    return (pos != std::string::npos) ? path.substr(0, pos + 1) : "";
}

// Resolves  #include "filename"  lines relative to dir.
static std::string ResolveIncludes(const std::string& src, const std::string& dir)
{
    std::istringstream in(src);
    std::ostringstream out;
    std::string line;
    while (std::getline(in, line))
    {
        size_t inc = line.find("#include");
        if (inc != std::string::npos)
        {
            size_t q1 = line.find('"', inc);
            size_t q2 = (q1 != std::string::npos) ? line.find('"', q1 + 1) : std::string::npos;
            if (q1 != std::string::npos && q2 != std::string::npos)
            {
                std::string includedSrc = ReadFile(dir + line.substr(q1 + 1, q2 - q1 - 1));
                if (!includedSrc.empty())
                    out << includedSrc << "\n";
                else
                    std::cerr << "[Shader] #include not found: " << line << "\n";
                continue;
            }
        }
        out << line << "\n";
    }
    return out.str();
}

Shader::~Shader()
{
    if (m_program)
        glDeleteProgram(m_program);
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
    const std::string dir   = DirectoryOf(vsPath);
    const std::string vsSrc = ResolveIncludes(ReadFile(vsPath), dir);
    const std::string fsSrc = ResolveIncludes(ReadFile(fsPath), dir);
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

bool Shader::LoadComputeFromFile(const std::string& csPath)
{
    const std::string dir   = DirectoryOf(csPath);
    const std::string csSrc = ResolveIncludes(ReadFile(csPath), dir);
    if (csSrc.empty())
    {
        std::cerr << "[Shader] Failed to read compute shader: " << csPath << "\n";
        return false;
    }

    unsigned int cs = Compile(GL_COMPUTE_SHADER, csSrc);
    if (!cs)
        return false;

    unsigned int p = glCreateProgram();
    glAttachShader(p, cs);
    glLinkProgram(p);
    glDeleteShader(cs);

    int ok = 0;
    glGetProgramiv(p, GL_LINK_STATUS, &ok);
    if (!ok)
    {
        int len = 0;
        glGetProgramiv(p, GL_INFO_LOG_LENGTH, &len);
        std::string log(len, '\0');
        glGetProgramInfoLog(p, len, nullptr, log.data());
        std::cerr << "[Shader] Compute link failed:\n" << log << "\n";
        glDeleteProgram(p);
        return false;
    }

    if (m_program)
        glDeleteProgram(m_program);
    m_program = p;
    return true;
}

void Shader::Dispatch(int gx, int gy, int gz) const
{
    glDispatchCompute(static_cast<unsigned int>(gx),
                      static_cast<unsigned int>(gy),
                      static_cast<unsigned int>(gz));
    glMemoryBarrier(GL_SHADER_IMAGE_ACCESS_BARRIER_BIT | GL_TEXTURE_FETCH_BARRIER_BIT);
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

void Shader::SetMat3(const char* name, const glm::mat3& m) const
{
    glUniformMatrix3fv(GetLocation(name), 1, GL_FALSE, glm::value_ptr(m));
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

void Shader::SetVec2Array(const char* name, const glm::vec2* v, int count) const
{
    glUniform2fv(GetLocation(name), count, glm::value_ptr(v[0]));
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

void Shader::SetFloatArray(const char* name, const float* v, int count) const
{
    glUniform1fv(GetLocation(name), count, v);
}

void Shader::BindUniformBlock(const char* name, unsigned int bindingPoint) const
{
    GLuint idx = glGetUniformBlockIndex(m_program, name);
    if (idx != GL_INVALID_INDEX)
        glUniformBlockBinding(m_program, idx, bindingPoint);
}