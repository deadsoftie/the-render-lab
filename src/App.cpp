#include "pch.h"
#include "App.h"

void App::GlfwErrorCallback(int error, const char* msg)
{
    fputs(msg, stderr);  // NOLINT(cert-err33-c)
}