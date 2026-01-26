#pragma once
#include <GLFW/glfw3.h>

namespace Input
{
    // Init AFTER window creation
    void Init(GLFWwindow* window);

    // Call once per frame
    void BeginFrame();
    void EndFrame();

    // Keyboard
    bool KeyDown(int glfwKey);
    bool KeyPressed(int glfwKey);
    bool KeyReleased(int glfwKey);

    // Mouse buttons
    bool MouseDown(int glfwButton);
    bool MousePressed(int glfwButton);
    bool MouseReleased(int glfwButton);

    // Mouse
    void GetMousePos(double& x, double& y);
    void GetMouseDelta(double& dx, double& dy);

    // Scroll (per-frame delta)
    void GetScrollDelta(double& sx, double& sy);

    // Cursor
    void SetCursorLocked(bool locked);
    bool IsCursorLocked();

    GLFWwindow* GetWindow();
}  // namespace Input
