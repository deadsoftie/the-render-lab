#include "pch.h"
#include "App.h"

#include "graphics/Shader.h"
#include "graphics/Renderer.h"
#include "scene/Camera.h"
#include "scene/CameraController.h"
#include "input/Input.h"
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
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 4);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
    glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GLFW_FALSE);

    m_window = glfwCreateWindow(1280, 720, "The Render Lab", nullptr, nullptr);
    if (!m_window)
        return false;

    Input::Init(m_window);

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

    Renderer renderer;
    Camera camera;
    CameraController cameraController;

    camera.SetPosition(glm::vec3(0.0f, 0.0f, 3.0f));
    camera.SetTarget(glm::vec3(0.0f, 0.0f, 0.0f));
    camera.SetUp(glm::vec3(0.0f, 1.0f, 0.0f));

    int fbW = 0, fbH = 0;
    glfwGetFramebufferSize(m_window, &fbW, &fbH);
    fbW = (fbW > 0) ? fbW : 1;
    fbH = (fbH > 0) ? fbH : 1;

    // Cache size update
    m_width = fbW;
    m_height = fbH;

    camera.SetViewport(m_width, m_height);
    camera.SetPerspective(glm::radians(60.0f), 0.1f, 100.0f);

    cameraController.InitializeFromCamera(camera);

    if (!renderer.Init())
    {
        std::cerr << "Renderer init failed\n";
        ShutdownImGui();
        ShutdownWindow();
        return -1;
    }
    renderer.SetViewport(m_width, m_height);

    while (!glfwWindowShouldClose(m_window))
    {
        glfwPollEvents();
        Input::BeginFrame();

        int fbW = 0, fbH = 0;
        glfwGetFramebufferSize(m_window, &fbW, &fbH);
        fbW = (fbW > 0) ? fbW : 1;
        fbH = (fbH > 0) ? fbH : 1;

        m_width = fbW;
        m_height = fbH;

        camera.SetViewport(fbW, fbH);
        renderer.SetViewport(fbW, fbH);

        ImGui_ImplOpenGL3_NewFrame();
        ImGui_ImplGlfw_NewFrame();
        ImGui::NewFrame();

        renderer.DrawDebugUI();

        cameraController.Update(camera, 0.0f, fbW, fbH);

        ImGui::Render();
        renderer.RenderFrame(camera);

        ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
        glfwSwapBuffers(m_window);

        Input::EndFrame();
    }

    ShutdownImGui();
    ShutdownWindow();
    return 0;
}
