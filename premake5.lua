workspace "the-render-lab"
    configurations { "Debug", "Release" }
    architecture "x64"
    startproject "the-render-lab"

engineName = "the-render-lab"
outputdir = "%{cfg.buildcfg}-%{cfg.system}-%{cfg.architecture}"

function ApplyCommonSettings()
    location "build"
    kind "ConsoleApp"
    language "C++"
    cppdialect "C++20"
    staticruntime "off"

    targetdir ("bin/" .. outputdir)
    objdir    ("bin-int/" .. outputdir)

    buildoptions { "/utf-8" }

    filter "system:windows"
        systemversion "latest"
        defines { "TRL_PLATFORM_WINDOWS" }

    filter "toolset:msc*"
        buildoptions { "/Zc:preprocessor" }

    filter "configurations:Debug"
        runtime "Debug"
        symbols "On"
        defines { "TRL_DEBUG" }

    filter "configurations:Release"
        runtime "Release"
        optimize "On"
        defines { "TRL_RELEASE" }

    filter {}
end

project (engineName)
    location (engineName)
    ApplyCommonSettings()
    debugdir "%{wks.basedir}"

    pchheader "pch.h"
    pchsource "src/pch.cpp"

    files
    {
        "src/**.h",
        "src/**.cpp",

        -- Shaders
        "assets/shaders/**.glsl",
        "assets/shaders/**.vert",
        "assets/shaders/**.frag",
        "assets/shaders/**.geom",
        "assets/shaders/**.tesc",
        "assets/shaders/**.tese",
        "assets/shaders/**.comp",
        "assets/shaders/**.hlsl",
        "assets/shaders/**.hlsli",

        -- ImGui core
        "third-party/imgui/imgui.cpp",
        "third-party/imgui/imgui_demo.cpp",
        "third-party/imgui/imgui_draw.cpp",
        "third-party/imgui/imgui_tables.cpp",
        "third-party/imgui/imgui_widgets.cpp",

        -- ImGui backends you actually use
        "third-party/imgui/backends/imgui_impl_glfw.cpp",
        "third-party/imgui/backends/imgui_impl_opengl3.cpp",

        -- GLAD C file
        "third-party/glad/src/glad.c",
    }

    includedirs
    {
        "src",

        "third-party/glad/include",
        "third-party/glm",
        "third-party/glfw/include",

        "third-party/imgui",
        "third-party/imgui/backends",
    }

    libdirs
    {
        "third-party/glfw/lib-vc2022",
    }

    defines
    {
        -- Tell ImGui OpenGL3 backend to use GLAD
        "IMGUI_IMPL_OPENGL_LOADER_GLAD",
    }

    links
    {
        "glfw3dll",   -- prefer no .lib extension
        "opengl32",   -- required on Windows for OpenGL
    }

    -- Compile glad.c as C and disable PCH for it
    filter "files:third-party/glad/src/glad.c"
        language "C"
        flags { "NoPCH" }
    filter {}

    -- ImGui does not include your pch.h, so disable PCH for it
    filter "files:third-party/imgui/**.cpp"
        flags { "NoPCH" }
    filter {}

    -- Copy runtime DLL next to exe (bindirs is not a premake feature)
    filter "system:windows"
        postbuildcommands
        {
            '{MKDIR} "%{cfg.targetdir}"',
            '{COPYFILE} "%{wks.basedir}/third-party/glfw/lib-vc2022/glfw3.dll" "%{cfg.targetdir}/glfw3.dll"'
        }
    filter {}

    filter { "files:**.glsl or files:**.vert or files:**.frag or files:**.geom or files:**.tesc or files:**.tese or files:**.comp or files:**.hlsl or files:**.hlsli" }
        buildaction "None"
    filter {}

