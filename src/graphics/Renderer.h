#pragma once
#include <glm/glm.hpp>
#include <array>
#include <unordered_map>
#include <unordered_set>
#include <ImGuizmo.h>

#include "Geometry.h"
#include "graphics/Shader.h"
#include "graphics/Mesh.h"
#include "scene/Camera.h"
#include "AOBuffer.h"
#include "GBuffer.h"
#include "ShadowMap.h"
#include "MomentShadowMap.h"
#include "Texture.h"
#include "scene/Scene.h"

struct RaycastMesh
{
    std::vector<glm::vec3> positions;
    std::vector<unsigned int> indices;
    glm::vec3 localMin{0.0f};
    glm::vec3 localMax{0.0f};
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
    void DrawSceneObjectsLit(Shader& sh, bool isForwardPass);
    Mesh* ResolveMesh(const std::string& ref);
    void GBufferPass(const Camera& camera);
    void AOPass(const Camera& camera);
    void AOBlurHPass(const Camera& camera);
    void AOBlurVPass(const Camera& camera);
    void FullscreenLightPass(const Camera& camera);
    void LocalLightsPass(const Camera& camera);
    void CelOutlinePass(const Camera& camera);

    void BuildHammersley(int n);
    void BakeIrradiance();
    void ComputeSHCoefficients(const float* pixels, int width, int height);
    void ScanHDRIFolder();
    bool LoadHDRI(const std::string& path);
    void ScanScenesFolder();
    bool SwitchScene(const std::string& path);
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

        // IBL debug
        IrradianceMap = 9,
        DiffuseIBL = 10,
        SpecularIBL = 11,

        // AO debug
        AOMapRaw   = 12,
        AOMapBlurH = 13,
        AOMapBlurV = 14,

        // Cel debug
        CelOutlineMask = 15,
    };

    enum class SelectionKind
    {
        None,
        Light,
        Object
    };

    struct Selection
    {
        SelectionKind kind = SelectionKind::None;
        int index = -1;
    };

    bool RaycastScene(const glm::vec3& rayOrigin, const glm::vec3& rayDir, int& outIndex) const;

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
    bool m_shadowsEnabled = true;
    float m_shadowBias = 0.04f;
    float m_shadowPcfRadius = 0.05f;

    // Moment Shadow Maps (MSM)
    std::array<MomentShadowMap, kMaxLights> m_msmMaps;
    Shader m_msmMomentShader;  // shadow_depth.vert + msm_moment_depth.frag
    Shader m_msmBlurHShader;   // blur_h.comp compute shader
    Shader m_msmBlurVShader;   // blur_v.comp compute shader
    bool m_useMSM = true;
    float m_msmAlpha = 1e-3f;
    float m_msmBlurStep = 1.0f;

    // AO shaders
    Shader m_aoShader;       // deferred_light.vert + ao.frag
    Shader m_aoBlurHShader;  // deferred_light.vert + ao_blur_h.frag
    Shader m_aoBlurVShader;  // deferred_light.vert + ao_blur_v.frag

    // Cel shading
    Shader m_celOutlineShader;

    bool      m_celEnabled          = false;
    bool      m_toonEnabled         = true;
    int       m_toonBands           = 3;
    float     m_outlineThickness    = 1.0f;
    float     m_depthThreshold      = 0.05f;
    float     m_normalThreshold     = 0.3f;
    glm::vec3 m_outlineColor        = glm::vec3(0.0f);

    // AO parameters
    bool  m_aoEnabled    = true;
    int   m_aoSamples    = 16;
    float m_aoRadius     = 1.0f;
    float m_aoScale      = 1.0f;
    float m_aoContrast   = 1.0f;
    float m_aoDelta      = 0.001f;
    float m_aoDepthSigma = 0.01f;
    int   m_aoBlurRadius = 5;
    float m_aoStrength   = 1.0f;

    // Gizmo shader
    Shader m_lightGizmoShader;

    GBuffer   m_gbuffer;
    AOBuffer  m_aoRawBuffer;
    AOBuffer  m_aoBlurHBuffer;
    AOBuffer  m_aoBlurVBuffer;

    // Light volume mesh
    Mesh m_lightVolumeSphereMesh;

    // Scene meshes
    Mesh m_cubeMesh;
    Mesh m_cornellMesh;
    Geometry::CornellMesh m_cornell;
    Mesh m_groundMesh;
    Mesh m_sphereMesh;  // used for spheres + local light volumes

    Scene m_activeScene;
    std::unordered_map<std::string, Mesh> m_modelMeshCache;      // keyed by model file path
    std::unordered_set<std::string> m_failedModelLoads;          // keyed by model file path
    std::unordered_map<std::string, RaycastMesh> m_raycastMeshes;  // keyed by meshRef, for picking

    // Gizmos
    Texture m_lightGizmoTex;

    // Lights UI
    float m_lightIntensity[kMaxLights] = {};
    bool m_lightEnabled[kMaxLights] = {};

    int m_debugLightIndex = 0;    // which light to use for Brightness
    Selection m_selection;        // currently selected light or object (gizmo + inspector target)
    ImGuizmo::OPERATION m_gizmoOperation = ImGuizmo::TRANSLATE;  // object gizmo mode; lights are always translate

    // PBS direct lighting technique: fullscreen "many lights" loop (default) vs
    // additive light-volume geometry. Mutually exclusive - never both, to avoid
    // double-counting direct light. Ignored in IBL mode (always fullscreen).
    bool m_useLightVolumes = false;

    // Cached per-frame camera matrices (set in RenderFrame, used in DrawDebugUI)
    glm::mat4 m_cachedView{1.0f};
    glm::mat4 m_cachedProj{1.0f};
    float m_globeRadius = 0.06f;  // world-space radius for LightGlobes view

    // Lighting mode
    enum class LightingMode
    {
        PBS,
        IBL
    };
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

    // Spherical harmonics irradiance (Part B)
    glm::vec3 m_shCoeffs[9]{};
    GLuint m_shCoeffsUBO = 0;
    bool m_useSHIrradiance = false;

    // Tone mapping
    float m_exposure = 2.5f;

    // Scene-wide ambient term (PBS fullscreen pass + forward path)
    float m_ambient = 0.02f;

    // HDRI rotation around Y axis (radians)
    float m_hdriRotation = 0.0f;

    // HDRI folder browser
    static constexpr const char* kHDRIFolder = "assets/hdris/";
    std::vector<std::string> m_hdriFiles;   // filenames only, sorted
    int m_hdriSelectedIdx = -1;

    // Scene folder browser
    static constexpr const char* kScenesFolder = "assets/scenes/";
    std::vector<std::string> m_sceneFiles;  // filenames only, sorted
    int m_sceneSelectedIdx = -1;
    std::string m_activeScenePath;
    bool m_sceneLoadFailed = false;

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
