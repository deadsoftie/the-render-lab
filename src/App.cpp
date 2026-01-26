#include "pch.h"
#include "App.h"

#include "graphics/Shader.h"
#include "graphics/Mesh.h"
#include "glm/gtc/matrix_transform.hpp"
#include <glad/glad.h>

void App::GlfwErrorCallback(int /*error*/, const char* msg)
{
    fputs(msg, stderr);
}

bool App::InitWindow()
{
    glfwSetErrorCallback(GlfwErrorCallback);

    if (!glfwInit())
        return false;

    glfwWindowHint(GLFW_RESIZABLE, GLFW_TRUE);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
    glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GLFW_FALSE);

    m_window = glfwCreateWindow(750, 750, "The Render Lab", nullptr, nullptr);
    if (!m_window)
        return false;

    glfwMakeContextCurrent(m_window);
    glfwSwapInterval(1);

    // ---- GLAD initialization (REQUIRED) ----
    if (!gladLoadGLLoader(reinterpret_cast<GLADloadproc>(glfwGetProcAddress)))
    {
        std::cerr << "Failed to initialize GLAD\n";
        return false;
    }

    // ---- OpenGL sanity check ----
    std::cout << "OpenGL Version: " << reinterpret_cast<const char*>(glGetString(GL_VERSION))
              << "\n";
    std::cout << "GLSL Version: "
              << reinterpret_cast<const char*>(glGetString(GL_SHADING_LANGUAGE_VERSION)) << "\n";
    std::cout << "Renderer: " << reinterpret_cast<const char*>(glGetString(GL_RENDERER)) << "\n";

    return true;
}

void App::ShutdownWindow()
{
    if (m_window)
    {
        glfwDestroyWindow(m_window);
        m_window = nullptr;
    }
    glfwTerminate();
}

bool App::InitImGui()
{
    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImGui::StyleColorsDark();

    if (!ImGui_ImplGlfw_InitForOpenGL(m_window, true))
        return false;

    // Explicit GLSL version is safer
    if (!ImGui_ImplOpenGL3_Init("#version 330"))
        return false;

    return true;
}

void App::ShutdownImGui()
{
    ImGui_ImplOpenGL3_Shutdown();
    ImGui_ImplGlfw_Shutdown();
    ImGui::DestroyContext();
}

void App::RenderTestUI()
{
    ImGui::Begin("Sandbox Status");
    ImGui::Text("GLFW: OK");
    ImGui::Text("GLAD: OK");
    ImGui::Text("ImGui: OK");
    ImGui::Separator();
    ImGui::Text("Window size: %d x %d", m_width, m_height);
    ImGui::End();
}

void App::RenderTestClear()
{
    glViewport(0, 0, m_width, m_height);
    glClearColor(0.1f, 0.12f, 0.15f, 1.0f);
    glClear(GL_COLOR_BUFFER_BIT);
}

int App::Run()
{
    if (!InitWindow())
        return -1;

    if (!InitImGui())
    {
        ShutdownWindow();
        return -1;
    }

    // --------- ONE-TIME RENDER SETUP ----------
    static Mesh gMesh;
    static Shader gShader;
    static bool gInit = false;

    if (!gInit)
    {
        gShader.LoadFromFiles("assets/shaders/basic_lit.vert", "assets/shaders/basic_lit.frag");

        std::vector<float> verts = {
            -0.5f,
            -0.5f,
            0.f,
            0.f,
            0.f,
            1.f,
            0.5f,
            -0.5f,
            0.f,
            0.f,
            0.f,
            1.f,
            0.0f,
            0.5f,
            0.f,
            0.f,
            0.f,
            1.f,
        };
        std::vector<unsigned int> idx = {0, 1, 2};
        gMesh.Create(verts, idx);

        gInit = true;
    }
    // ------------------------------------------

    while (!glfwWindowShouldClose(m_window))
    {
        glfwPollEvents();

        // ---- ImGui frame ----
        ImGui_ImplOpenGL3_NewFrame();
        ImGui_ImplGlfw_NewFrame();
        ImGui::NewFrame();

        RenderTestUI();

        ImGui::Render();

        // ---- OpenGL ----
        RenderTestClear();

        // --------- DRAW LIT GEOMETRY ----------
        glm::vec3 camPos(0.f, 0.f, 2.0f);
        glm::mat4 view = glm::lookAt(camPos, glm::vec3(0, 0, 0), glm::vec3(0, 1, 0));
        glm::mat4 proj =
            glm::perspective(glm::radians(60.0f),
                             static_cast<float>(m_width) / static_cast<float>(m_height),
                             0.1f,
                             100.f);
        glm::mat4 model(1.0f);

        glm::vec3 lightPos(1.2f, 1.0f, 2.0f);

        gShader.Bind();
        gShader.SetMat4("uModel", model);
        gShader.SetMat4("uView", view);
        gShader.SetMat4("uProj", proj);
        gShader.SetVec3("uCamPos", camPos);
        gShader.SetVec3("uLightPos", lightPos);
        gShader.SetVec3("uLightColor", glm::vec3(1.0f));
        gShader.SetVec3("uAlbedo", glm::vec3(0.8f, 0.3f, 0.2f));
        gShader.SetFloat("uAmbient", 0.08f);
        gShader.SetFloat("uShininess", 64.0f);

        gMesh.Draw();
        gShader.Unbind();
        // -------------------------------------

        // Draw ImGui
        ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
        glfwSwapBuffers(m_window);
    }

    ShutdownImGui();
    ShutdownWindow();
    return 0;
}
