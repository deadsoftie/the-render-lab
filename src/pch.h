#pragma once

// -----------------------------
// Standard library
// -----------------------------
#include <cstdio>
#include <cstdlib>
#include <iostream>
#include <string>
#include <vector>

// -----------------------------
// Windowing
// -----------------------------
#define GLFW_INCLUDE_NONE
#include <GLFW/glfw3.h>

// -----------------------------
// GLM (Math)
// -----------------------------
#define GLM_FORCE_CTOR_INIT
#define GLM_FORCE_RADIANS
#define GLM_FORCE_SWIZZLE
#include <glm/glm.hpp>

// -----------------------------
// ImGui
// -----------------------------
#include <imgui.h>
#include <imgui_impl_glfw.h>
#include <imgui_impl_opengl3.h>