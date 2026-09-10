#include "pch.h"

#include "App.h"

#ifdef TRL_PLATFORM_WINDOWS
extern "C"
{
    // GPU preferred, falls back to integrated if not present
    __declspec(dllexport) unsigned long NvOptimusEnablement = 0x00000001;
    __declspec(dllexport) int AmdPowerXpressRequestHighPerformance = 1;
}
#endif

int main()
{
    App app;
    return app.Run();
}
