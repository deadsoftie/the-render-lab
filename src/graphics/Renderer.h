#pragma once
#include <glm/glm.hpp>

#include "Geometry.h"
#include "Graphics/Shader.h"
#include "Graphics/Mesh.h"
#include "Scene/Camera.h"

#include <array>

struct Light
{
    glm::vec3 position{1.2f, 1.0f, 2.0f};
    glm::vec3 color{1.0f, 1.0f, 1.0f};
};

struct Material
{
    glm::vec3 albedo{0.8f, 0.3f, 0.2f};
    float ambient = 0.08f;
    float shininess = 64.0f;
};

class Renderer
{
   public:
    bool Init();      // call once after GLAD is ready
    void Shutdown();  // optional (Mesh/Shader destructors usually handle it)

    void SetViewport(int w, int h);

    // per-frame
    void RenderFrame(const Camera& camera);

    // debug UI controls
    void DrawDebugUI();

   private:
    bool m_ready = false;

    int m_viewportW = 1280;
    int m_viewportH = 720;

    static constexpr int kMaxLights = 4;
    std::array<Light, kMaxLights> m_lights{};
    int m_lightCount = 1;

    Shader m_litShader;

    Mesh m_cubeMesh;

    Mesh m_cornellMesh;
    Geometry::CornellMesh m_cornell;

    Mesh m_groundMesh;

    Mesh m_sphereMesh;

    Material m_mat;

    glm::mat4 m_model{1.0f};
};
