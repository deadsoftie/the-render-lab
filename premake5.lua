workspace "the-render-lab"
    configurations { "Debug", "Release" }
    architecture "x64"
    startproject "the-render-lab"

-- ---------------------------------------------------------
-- Project names and directories
-- ---------------------------------------------------------

engineName = "the-render-lab"
outputdir = "%{cfg.buildcfg}-%{cfg.system}-%{cfg.architecture}"

-- ---------------------------------------------------------
-- Common settings helper
-- ---------------------------------------------------------
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

-- ========================================================= 
-- the-rendering-lab
-- =========================================================

project (engineName)
    location (engineName)

    ApplyCommonSettings()

    pchheader "pch.h"
    pchsource "src/pch.cpp"

    files
    {
        "src/**.h",
        "src/**.cpp",
		
		-- ImGui source files (if needed for extra control)
		"third-party/imgui/**.cpp",
		"third-party/imgui/backends/**.cpp",
        
        -- Custom compilation for glad.c file (better design can be added later)
        "third-party/glad/src/glad.c",
    }

    includedirs
    {
        "src",
		
		-- GLAD
		"third-party/glad/include",

		-- GLM
		"third-party/glm",
		
		-- GLFW
		"third-party/glfw/include",
		
		-- ImGui (headers and sources)
		"third-party/imgui",
		"third-party/imgui/backends",
    }
	
	libdirs
	{
		"third-party/glfw/lib-vc2022",
	}
	
	bindirs
	{
		"third-party/glfw/lib-vc2022",
	}
	
	links
	{
		"glfw3dll.lib",
	}

    filter "files:third-party/glad/src/glad.c"
        language "C"
        flags { "NoPCH" }
    filter {}
	
    -- If you want engine itself to link to some libs, do it here.

    filter {}