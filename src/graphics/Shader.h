#pragma once

#include <string>
#include <glm/glm.hpp>

class Shader
{
   public:
    Shader() = default;
    ~Shader();

    bool LoadFromFiles(const std::string& vsPath, const std::string& fsPath);
    bool LoadComputeFromFile(const std::string& csPath);
    void Dispatch(int gx, int gy, int gz = 1) const;
    void Bind() const;
    void Unbind() const;

    // Uniforms
    void SetMat4(const char* name, const glm::mat4& m) const;
    void SetVec2(const char* name, const glm::vec2& v) const;
    void SetVec3(const char* name, const glm::vec3& v) const;
    void SetVec3Array(const char* name, const glm::vec3* v, int count) const;
    void SetFloat(const char* name, float f) const;
    void SetInt(const char* name, int v) const;
    void SetFloatArray(const char* name, float* v, int count) const;

   private:
    unsigned int m_program = 0;

    static std::string ReadTextFile(const std::string& path);
    static unsigned int Compile(unsigned int type, const std::string& src);
    static unsigned int Link(unsigned int vs, unsigned int fs);
    int GetLocation(const char* name) const;
};