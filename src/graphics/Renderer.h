#pragma once
#include <glm/glm.hpp>
#include <array>

#include "Geometry.h"
#include "Graphics/Shader.h"
#include "Graphics/Mesh.h"
#include "Scene/Camera.h"
#include "GBuffer.h"
#include "ShadowMap.h"
#include "MomentShadowMap.h"
#include "Texture.h"

struct Light
{
    glm::vec3 position{1.2f, 1.0f, 2.0f};
    glm::vec3 color{1.0f, 1.0f, 1.0f};
    float range = 2.0f;
};

struct Material
{
    glm::vec3 kd{0.8f, 0.3f, 0.2f};  // diffuse
    glm::vec3 ks{0.04f};             // specular / F0
    float ambient = 0.08f;
    float alpha = 64.0f;  // Phong shininess exponent (1..256); used as PBS roughness
};

class Renderer
{
   public:
    bool Init();
    void Shutdown();

    void SetViewport(int w, int h);

    // per-frame
    void RenderFrame(const Camera& camera);

    // debug UI controls
    void DrawDebugUI();

   private:
    void RenderForward(const Camera& camera);
    void RenderDeferred(const Camera& camera);

    void ShadowPass();
    void MSMShadowPass();
    void MSMBlurPass();
    void DrawSceneGeometry(Shader& sh);
    void GBufferPass(const Camera& camera);
    void FullscreenLightPass(const Camera& camera);
    void LocalLightsPass(const Camera& camera);

    void BuildHammersley(int n);
    void BakeIrradiance();
    void EnsureScreenQuad();
    void DestroyScreenQuad();

    void EnsureLightGizmoQuad();
    void DestroyLightGizmoQuad();

    void DrawLightGizmos(const Camera& camera) const;

    enum class DebugView : int
    {
        Final = 0,

        // Deferred
        WorldPos = 1,
        Normal = 2,
        Kd = 3,
        KsAlpha = 4,

        // Many lights
        EyeVec = 5,
        LightGlobes = 6,
        Brightness = 7,

        // MSM
        MSMDepth = 8,
    };

    bool m_ready = false;

    int m_viewportW = 1280;
    int m_viewportH = 720;

    static constexpr int kMaxLights = 5;
    std::array<Light, kMaxLights> m_lights{};
    int m_lightCount = 1;

    // Forward shader (kept for sanity checks)
    Shader m_litShader;

    // Deferred shaders
    Shader m_gbufferShader;
    Shader m_fullscreenShader;
    Shader m_localLightShader;

    // Shadow shader + maps (PCF)
    Shader m_shadowShader;
    std::array<ShadowMap, kMaxLights> m_shadowMaps;
    bool  m_shadowsEnabled  = true;
    float m_shadowBias      = 0.04f;
    float m_shadowPcfRadius = 0.05f;

    // Moment Shadow Maps (MSM)
    std::array<MomentShadowMap, kMaxLights> m_msmMaps;
    Shader m_msmMomentShader;   // shadow_depth.vert + msm_moment_depth.frag
    Shader m_msmBlurHShader;    // blur_h.comp compute shader
    Shader m_msmBlurVShader;    // blur_v.comp compute shader
    bool  m_useMSM      = true;
    float m_msmAlpha    = 1e-3f;
    float m_msmBlurStep = 1.0f;

    // Gizmo shader
    Shader m_lightGizmoShader;

    GBuffer m_gbuffer;

    // Light volume mesh
    Mesh m_lightVolumeSphereMesh;

    // Scene meshes
    Mesh m_cubeMesh;
    Mesh m_cornellMesh;
    Geometry::CornellMesh m_cornell;
    Mesh m_groundMesh;
    Mesh m_sphereMesh;  // used for spheres + local light volumes

    Material m_mat;

    // Gizmos
    Texture m_lightGizmoTex;

    // Lights UI
    float m_lightIntensity[kMaxLights] = {};
    bool m_lightEnabled[kMaxLights] = {};

    int m_debugLightIndex = 0;    // which light to use for Brightness
    float m_globeRadius = 0.06f;  // world-space radius for LightGlobes view

    // Object albedos UI
    glm::vec3 m_albedoTall = glm::vec3(0.85f);
    glm::vec3 m_albedoShort = glm::vec3(0.75f, 0.75f, 0.80f);
    glm::vec3 m_albedoSmall = glm::vec3(0.90f, 0.80f, 0.70f);
    glm::vec3 m_albedoSphere = glm::vec3(0.80f, 0.80f, 0.95f);

    // Lighting mode
    enum class LightingMode { PBS, IBL };
    LightingMode m_lightingMode = LightingMode::PBS;

    // IBL shader (deferred_ibl.frag)
    Shader m_iblShader;

    // Compute shader that bakes the irradiance map from the loaded HDRI
    Shader m_irradianceBakeShader;

    // HDR environment + irradiance maps
    Texture m_hdriTex;
    Texture m_irradianceTex;

    // Hammersley low-discrepancy sample pairs (CPU copy)
    static constexpr int kMaxIBLSamples = 100;
    glm::vec2 m_hammersley[kMaxIBLSamples]{};
    int m_iblSamples = 24;

    // Tone mapping
    float m_exposure = 1.0f;

    // HDRI rotation around Y axis (radians)
    float m_hdriRotation = 0.0f;

    // ImGui file path buffer
    char m_hdriPathBuf[512]  = "assets/hdris/Sierra_Madre_B_Ref.hdr";

    // mode toggles
    bool m_useDeferred = true;
    DebugView m_debugView = DebugView::Final;

    // Screen quad
    unsigned int m_quadVAO = 0;
    unsigned int m_quadVBO = 0;

    // Gizmo icon quad
    unsigned int m_lightGizmoVAO = 0;
    unsigned int m_lightGizmoVBO = 0;

    // Gizmo toggle
    bool m_showLightGizmos = true;
};
