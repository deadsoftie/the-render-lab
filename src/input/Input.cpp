#include "pch.h"
#include "Input/Input.h"

#include <array>
#include <cassert>

namespace Input
{
    static GLFWwindow* gWindow = nullptr;

    static std::array<unsigned char, GLFW_KEY_LAST + 1> gKeyNow{};
    static std::array<unsigned char, GLFW_KEY_LAST + 1> gKeyPrev{};

    static std::array<unsigned char, GLFW_MOUSE_BUTTON_LAST + 1> gMouseNow{};
    static std::array<unsigned char, GLFW_MOUSE_BUTTON_LAST + 1> gMousePrev{};

    static double gMouseX = 0.0, gMouseY = 0.0;
    static double gPrevMouseX = 0.0, gPrevMouseY = 0.0;
    static double gDeltaMouseX = 0.0, gDeltaMouseY = 0.0;

    static double gScrollX = 0.0, gScrollY = 0.0;
    static bool gCursorLocked = false;

    // ------------------------------------------------------------
    // GLFW CALLBACKS
    // ------------------------------------------------------------

    static void KeyCallback(GLFWwindow*, int key, int, int action, int)
    {
        if (key < 0 || key > GLFW_KEY_LAST)
            return;

        if (action == GLFW_PRESS || action == GLFW_REPEAT)
            gKeyNow[key] = 1;
        else if (action == GLFW_RELEASE)
            gKeyNow[key] = 0;
    }

    static void MouseButtonCallback(GLFWwindow*, int button, int action, int)
    {
        if (button < 0 || button > GLFW_MOUSE_BUTTON_LAST)
            return;

        if (action == GLFW_PRESS)
            gMouseNow[button] = 1;
        else if (action == GLFW_RELEASE)
            gMouseNow[button] = 0;
    }

    static void CursorPosCallback(GLFWwindow*, double x, double y)
    {
        gMouseX = x;
        gMouseY = y;
    }

    static void ScrollCallback(GLFWwindow*, double xoff, double yoff)
    {
        gScrollX += xoff;
        gScrollY += yoff;
    }

    // ------------------------------------------------------------
    // PUBLIC API
    // ------------------------------------------------------------

    void Init(GLFWwindow* window)
    {
        assert(window && "Input::Init requires a valid GLFWwindow");
        gWindow = window;

        gKeyNow.fill(0);
        gKeyPrev.fill(0);
        gMouseNow.fill(0);
        gMousePrev.fill(0);

        glfwGetCursorPos(gWindow, &gMouseX, &gMouseY);
        gPrevMouseX = gMouseX;
        gPrevMouseY = gMouseY;

        gDeltaMouseX = gDeltaMouseY = 0.0;
        gScrollX = gScrollY = 0.0;

        glfwSetKeyCallback(gWindow, KeyCallback);
        glfwSetMouseButtonCallback(gWindow, MouseButtonCallback);
        glfwSetCursorPosCallback(gWindow, CursorPosCallback);
        glfwSetScrollCallback(gWindow, ScrollCallback);
    }

    void BeginFrame()
    {
        assert(gWindow && "Input::BeginFrame called before Input::Init");

        // roll previous states
        gKeyPrev = gKeyNow;
        gMousePrev = gMouseNow;

        // reset per-frame scroll
        gScrollX = 0.0;
        gScrollY = 0.0;

        // mouse delta
        gDeltaMouseX = gMouseX - gPrevMouseX;
        gDeltaMouseY = gMouseY - gPrevMouseY;

        gPrevMouseX = gMouseX;
        gPrevMouseY = gMouseY;
    }

    void EndFrame()
    {
    }

    // ------------------------------------------------------------
    // QUERIES
    // ------------------------------------------------------------

    bool KeyDown(int key)
    {
        return key >= 0 && key <= GLFW_KEY_LAST && gKeyNow[key];
    }

    bool KeyPressed(int key)
    {
        return key >= 0 && key <= GLFW_KEY_LAST && gKeyNow[key] && !gKeyPrev[key];
    }

    bool KeyReleased(int key)
    {
        return key >= 0 && key <= GLFW_KEY_LAST && !gKeyNow[key] && gKeyPrev[key];
    }

    bool MouseDown(int button)
    {
        return button >= 0 && button <= GLFW_MOUSE_BUTTON_LAST && gMouseNow[button];
    }

    bool MousePressed(int button)
    {
        return button >= 0 && button <= GLFW_MOUSE_BUTTON_LAST && gMouseNow[button] &&
               !gMousePrev[button];
    }

    bool MouseReleased(int button)
    {
        return button >= 0 && button <= GLFW_MOUSE_BUTTON_LAST && !gMouseNow[button] &&
               gMousePrev[button];
    }

    void GetMousePos(double& x, double& y)
    {
        x = gMouseX;
        y = gMouseY;
    }

    void GetMouseDelta(double& dx, double& dy)
    {
        dx = gDeltaMouseX;
        dy = gDeltaMouseY;
    }

    void GetScrollDelta(double& sx, double& sy)
    {
        sx = gScrollX;
        sy = gScrollY;
    }

    void SetCursorLocked(bool locked)
    {
        gCursorLocked = locked;
        glfwSetInputMode(gWindow, GLFW_CURSOR, locked ? GLFW_CURSOR_DISABLED : GLFW_CURSOR_NORMAL);
    }

    bool IsCursorLocked()
    {
        return gCursorLocked;
    }

    GLFWwindow* GetWindow()
    {
        return gWindow;
    }
}  // namespace Input
