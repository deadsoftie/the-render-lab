#include "pch.h"
#include <glad/glad.h>

#include "PbfGpuTests.h"
#include "TestFramework.h"

std::string ShaderPath(const char* file)
{
    return std::string(TRL_SHADER_DIR) + file;
}

int main()
{
    if (!glfwInit())
    {
        std::cerr << "GLFW init failed\n";
        return 1;
    }
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 4);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
    glfwWindowHint(GLFW_VISIBLE, GLFW_FALSE);
    GLFWwindow* window = glfwCreateWindow(64, 64, "pbf-tests", nullptr, nullptr);
    if (!window)
    {
        std::cerr << "No OpenGL 4.3 context available\n";
        glfwTerminate();
        return 1;
    }
    glfwMakeContextCurrent(window);
    if (!gladLoadGLLoader(reinterpret_cast<GLADloadproc>(glfwGetProcAddress)))
    {
        std::cerr << "glad failed to load OpenGL\n";
        glfwDestroyWindow(window);
        glfwTerminate();
        return 1;
    }
    std::cout << "OpenGL " << glGetString(GL_VERSION) << " on " << glGetString(GL_RENDERER) << "\n";

    RunGpuBufferTests();
    RunGpuQueryTimerTests();
    RunPbfShaderCompileTests();
    RunGridTests();
    RunSolverTests();
    RunVelocityTests();

    int result = Test::Summarize();
    glfwDestroyWindow(window);
    glfwTerminate();
    return result;
}
