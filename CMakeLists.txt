cmake_minimum_required(VERSION 3.20)
project(the-render-lab CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(glm CONFIG REQUIRED)
find_package(Stb REQUIRED)
find_package(glfw3 CONFIG REQUIRED)
find_package(glad CONFIG REQUIRED)
find_package(assimp CONFIG REQUIRED)
find_package(imguizmo CONFIG REQUIRED)
find_package(imgui CONFIG REQUIRED)
find_package(OpenGL REQUIRED)

file(GLOB_RECURSE SOURCES CONFIGURE_DEPENDS
    "${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp"
)

add_executable(the-render-lab ${SOURCES})

target_include_directories(the-render-lab PRIVATE
    "${CMAKE_CURRENT_SOURCE_DIR}/src"
    "${Stb_INCLUDE_DIR}"
)

target_precompile_headers(the-render-lab PRIVATE
    "${CMAKE_CURRENT_SOURCE_DIR}/src/pch.h"
)

target_compile_definitions(the-render-lab PRIVATE
    IMGUI_IMPL_OPENGL_LOADER_GLAD
)

if(WIN32)
    target_compile_definitions(the-render-lab PRIVATE TRL_PLATFORM_WINDOWS)
elseif(UNIX AND NOT APPLE)
    target_compile_definitions(the-render-lab PRIVATE TRL_PLATFORM_LINUX)
elseif(APPLE)
    target_compile_definitions(the-render-lab PRIVATE TRL_PLATFORM_MACOS)
endif()

if(CMAKE_BUILD_TYPE STREQUAL "Debug")
    target_compile_definitions(the-render-lab PRIVATE TRL_DEBUG)
else()
    target_compile_definitions(the-render-lab PRIVATE TRL_RELEASE)
endif()

target_link_libraries(the-render-lab PRIVATE
    glm::glm
    glfw
    glad::glad
    assimp::assimp
    imguizmo::imguizmo
    imgui::imgui
    OpenGL::GL
)

add_custom_command(TARGET the-render-lab POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_directory
        "${CMAKE_CURRENT_SOURCE_DIR}/assets"
        "$<TARGET_FILE_DIR:the-render-lab>/assets"
)
