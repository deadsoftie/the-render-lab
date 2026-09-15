cmake_minimum_required(VERSION 3.20)
project(the-render-lab CXX C)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)

find_package(glm CONFIG REQUIRED)
find_package(Stb REQUIRED)
find_package(glfw3 CONFIG REQUIRED)
find_package(glad CONFIG REQUIRED)
find_package(imguizmo CONFIG REQUIRED)
find_package(imgui CONFIG REQUIRED)
find_package(nlohmann_json CONFIG REQUIRED)
find_package(OpenGL REQUIRED)

file(GLOB_RECURSE SOURCES CONFIGURE_DEPENDS
    "${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp"
)

# ufbx is vendored (single .h/.c, MIT licensed) instead of pulled via vcpkg -
# Assimp's FBX importer has an internal bind-pose parsing bug on this
# project's FBX 7700 assets that no aiProcess flag combination avoids.
add_executable(the-render-lab ${SOURCES} "${CMAKE_CURRENT_SOURCE_DIR}/third_party/ufbx/ufbx.c")

target_include_directories(the-render-lab PRIVATE
    "${CMAKE_CURRENT_SOURCE_DIR}/src"
    "${CMAKE_CURRENT_SOURCE_DIR}/third_party/ufbx"
    "${Stb_INCLUDE_DIR}"
)

# Restricted to CXX: pch.h pulls in glm/imgui headers that ufbx.c (compiled
# as C) cannot parse.
target_precompile_headers(the-render-lab PRIVATE
    "$<$<COMPILE_LANGUAGE:CXX>:${CMAKE_CURRENT_SOURCE_DIR}/src/pch.h>"
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
    imguizmo::imguizmo
    imgui::imgui
    nlohmann_json::nlohmann_json
    OpenGL::GL
)

add_custom_command(TARGET the-render-lab POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_directory
        "${CMAKE_CURRENT_SOURCE_DIR}/assets"
        "$<TARGET_FILE_DIR:the-render-lab>/assets"
)

# Anim::Math is dependency-free by design, so its tests build standalone -
# no vcpkg libs, no pch.
file(GLOB ANIM_MATH_SOURCES CONFIGURE_DEPENDS
    "${CMAKE_CURRENT_SOURCE_DIR}/src/anim/Math/*.cpp"
)
file(GLOB TEST_SOURCES CONFIGURE_DEPENDS
    "${CMAKE_CURRENT_SOURCE_DIR}/tests/*.cpp"
)

add_executable(the-render-lab-tests ${ANIM_MATH_SOURCES} ${TEST_SOURCES})

target_include_directories(the-render-lab-tests PRIVATE
    "${CMAKE_CURRENT_SOURCE_DIR}/src"
    "${CMAKE_CURRENT_SOURCE_DIR}/tests"
)

set_target_properties(the-render-lab-tests PROPERTIES
    CXX_STANDARD 20
    CXX_STANDARD_REQUIRED ON
)

enable_testing()
add_test(NAME MathTests COMMAND the-render-lab-tests)
