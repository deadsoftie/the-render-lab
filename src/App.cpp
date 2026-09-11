#include "pch.h"
#include "App.h"

#include "graphics/Shader.h"
#include "graphics/Renderer.h"
#include "scene/Camera.h"
#include "scene/CameraController.h"
#include "input/Input.h"
#include "glm/gtc/matrix_transform.hpp"

#include <glad/glad.h>
#include <imgui_internal.h>
#include <ImGuizmo.h>

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
    glfwWindowHint(GLFW_MAXIMIZED, GLFW_TRUE);
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

    ImGuiIO& io = ImGui::GetIO();
    io.ConfigFlags |= ImGuiConfigFlags_DockingEnable;

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

int App::Run()
{
    if (!InitWindow())
        return -1;

    if (!InitImGui())
    {
        ShutdownWindow();
        return -1;
    }

    {
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
            renderer.Shutdown();
            ShutdownImGui();
            ShutdownWindow();
            return -1;
        }
        renderer.SetViewport(m_width, m_height);

        while (!glfwWindowShouldClose(m_window))
        {
            glfwPollEvents();
            Input::BeginFrame();

            if (Input::KeyPressed(GLFW_KEY_ESCAPE))
                glfwSetWindowShouldClose(m_window, GLFW_TRUE);

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
            ImGuizmo::BeginFrame();

            // Fullscreen dockspace
            {
                ImGuiViewport* vp = ImGui::GetMainViewport();
                ImGui::SetNextWindowPos(vp->Pos);
                ImGui::SetNextWindowSize(vp->Size);
                ImGui::SetNextWindowViewport(vp->ID);
                ImGui::PushStyleVar(ImGuiStyleVar_WindowRounding, 0.0f);
                ImGui::PushStyleVar(ImGuiStyleVar_WindowBorderSize, 0.0f);
                ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2(0.0f, 0.0f));
                const ImGuiWindowFlags kDockHostFlags =
                    ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_NoCollapse |
                    ImGuiWindowFlags_NoResize   | ImGuiWindowFlags_NoMove     |
                    ImGuiWindowFlags_NoBringToFrontOnFocus | ImGuiWindowFlags_NoNavFocus |
                    ImGuiWindowFlags_NoBackground | ImGuiWindowFlags_NoDocking;
                ImGui::Begin("##DockHost", nullptr, kDockHostFlags);
                ImGui::PopStyleVar(3);

                ImGuiID dockId = ImGui::GetID("MainDockSpace");
                ImGui::DockSpace(dockId, ImVec2(0.0f, 0.0f), ImGuiDockNodeFlags_PassthruCentralNode);

                static bool s_layoutBuilt = false;
                if (!s_layoutBuilt)
                {
                    s_layoutBuilt = true;
                    ImGui::DockBuilderRemoveNode(dockId);
                    ImGui::DockBuilderAddNode(dockId,
                        ImGuiDockNodeFlags_PassthruCentralNode | ImGuiDockNodeFlags_DockSpace);
                    ImGui::DockBuilderSetNodeSize(dockId, vp->Size);

                    ImGuiID leftId;
                    ImGui::DockBuilderSplitNode(dockId, ImGuiDir_Left, 0.18f, &leftId, nullptr);
                    ImGui::DockBuilderDockWindow("Renderer", leftId);
                    ImGui::DockBuilderFinish(dockId);
                }

                ImGui::End();
            }

            renderer.DrawDebugUI();

            cameraController.Update(camera, 0.0f, fbW, fbH);

            ImGui::Render();
            renderer.RenderFrame(camera);

            ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
            glfwSwapBuffers(m_window);

            Input::EndFrame();
        }

        renderer.Shutdown();
    }

    ShutdownImGui();
    ShutdownWindow();
    return 0;
}
