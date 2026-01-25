#pragma once

struct GLFWwindow;

class App
{
   public:
    int Run();

   private:
    static void GlfwErrorCallback(int error, const char* msg);

    bool InitWindow();
    void ShutdownWindow();

    bool InitImGui();
    void ShutdownImGui();

    void RenderTestUI();
    void RenderTestClear();

   private:
    GLFWwindow* m_window = nullptr;

    int m_width = 1280;
    int m_height = 720;
};