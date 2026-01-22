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

   private:
    GLFWwindow* m_window = nullptr;
};