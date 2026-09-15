#include "pch.h"
#include <algorithm>
#include <cmath>
#include <cstring>
#include <filesystem>
#include <glad/glad.h>
#include <glm/gtc/constants.hpp>
#include <stb_image.h>

#include "graphics/Renderer.h"
#include "graphics/resources/Geometry.h"
#include "scene/ModelLoader.h"
#include "scene/SceneLoader.h"
#include "scene/SkinnedModelLoader.h"
#include "anim/SkeletalLoader.h"

bool Renderer::Init()
{
    // Shaders
    if (!m_litShader.LoadFromFiles("assets/shaders/basic_lit.vert",
                                   "assets/shaders/basic_lit.frag"))
        return false;

    if (!m_gbufferShader.LoadFromFiles("assets/shaders/gbuffer.vert",
                                       "assets/shaders/gbuffer.frag"))
        return false;

    if (!m_fullscreenShader.LoadFromFiles("assets/shaders/deferred_light.vert",
                                          "assets/shaders/deferred_light.frag"))
        return false;

    if (!m_localLightShader.LoadFromFiles("assets/shaders/local_light.vert",
                                          "assets/shaders/local_light.frag"))
        return false;

    if (!m_iblShader.LoadFromFiles("assets/shaders/deferred_light.vert",
                                   "assets/shaders/deferred_ibl.frag"))
        return false;
    m_iblShader.BindUniformBlock("SHBlock", 2);
    if (!m_lightGizmoShader.LoadFromFiles("assets/shaders/gizmos/light_gizmo.vert",
                                          "assets/shaders/gizmos/light_gizmo.frag"))
    {
        return false;
    }

    if (!m_boneLineShader.LoadFromFiles("assets/shaders/bone_lines.vert",
                                        "assets/shaders/bone_lines.frag"))
        return false;

    if (!m_gbufferSkinnedShader.LoadFromFiles("assets/shaders/gbuffer_skinned.vert",
                                              "assets/shaders/gbuffer.frag"))
        return false;

    if (!m_shadowShader.LoadFromFiles("assets/shaders/shadow_depth.vert",
                                      "assets/shaders/shadow_depth.frag"))
        return false;

    if (!m_msmMomentShader.LoadFromFiles("assets/shaders/shadow_depth.vert",
                                         "assets/shaders/msm_moment_depth.frag"))
        return false;

    if (!m_msmBlurHShader.LoadComputeFromFile("assets/shaders/blur_h.comp"))
        return false;

    if (!m_msmBlurVShader.LoadComputeFromFile("assets/shaders/blur_v.comp"))
        return false;

    if (!m_irradianceBakeShader.LoadComputeFromFile("assets/shaders/irradiance_bake.comp"))
        return false;

    if (!m_aoShader.LoadFromFiles("assets/shaders/deferred_light.vert",
                                  "assets/shaders/ao.frag"))
        return false;

    if (!m_aoBlurHShader.LoadFromFiles("assets/shaders/deferred_light.vert",
                                       "assets/shaders/ao_blur_h.frag"))
        return false;

    if (!m_aoBlurVShader.LoadFromFiles("assets/shaders/deferred_light.vert",
                                       "assets/shaders/ao_blur_v.frag"))
        return false;

    if (!m_celOutlineShader.LoadFromFiles("assets/shaders/cel_outline.vert",
                                          "assets/shaders/cel_outline.frag"))
        return false;

    for (auto& sm : m_shadowMaps)
        if (!sm.Create(512))
            return false;

    for (auto& mm : m_msmMaps)
        if (!mm.Create(512))
            return false;

    // Gizmos
    if (!m_lightGizmoTex.LoadFromFile("assets/textures/light_gizmo_white.png",
                                      false))  // not sRGB, it's UI/debug
        return false;

    EnsureLightGizmoQuad();
    EnsureBoneLineBuffer();

    // Light volume mesh
    auto unitSphere = Geometry::MakeSphere(1.0f, 32, 16);
    m_lightVolumeSphereMesh.Create(unitSphere.vertices, unitSphere.indices);

    // Meshes
    auto cube = Geometry::MakeCube(0.5f);
    m_cubeMesh.Create(cube.vertices, cube.indices);
    m_raycastMeshes["cube"] = BuildRaycastMesh(cube.vertices, cube.indices);

    auto sphere = Geometry::MakeSphere(0.5f, 32, 16);
    m_sphereMesh.Create(sphere.vertices, sphere.indices);
    m_raycastMeshes["sphere"] = BuildRaycastMesh(sphere.vertices, sphere.indices);

    m_cornell = Geometry::MakeCornellBox({1.0f, 1.0f, 1.0f});
    m_cornellMesh.Create(m_cornell.mesh.vertices, m_cornell.mesh.indices);
    m_raycastMeshes["cornell"] = BuildRaycastMesh(m_cornell.mesh.vertices, m_cornell.mesh.indices);

    auto ground = Geometry::MakeGroundPlane(12.0f, 0.0f);
    m_groundMesh.Create(ground.vertices, ground.indices);
    m_raycastMeshes["ground"] = BuildRaycastMesh(ground.vertices, ground.indices);

    if (!SwitchScene(std::string(kScenesFolder) + "animation.json"))
        return false;
    ScanScenesFolder();

    // Defaults
    glEnable(GL_DEPTH_TEST);

    // Screen quad + gbuffer
    EnsureScreenQuad();

    if (!m_gbuffer.Create(m_viewportW, m_viewportH))
        return false;

    if (!m_aoRawBuffer.Create(m_viewportW, m_viewportH))
        return false;
    if (!m_aoBlurHBuffer.Create(m_viewportW, m_viewportH))
        return false;
    if (!m_aoBlurVBuffer.Create(m_viewportW, m_viewportH))
        return false;

    BuildHammersley(m_iblSamples);
    ScanHDRIFolder();

    m_ready = true;
    return true;
}

void Renderer::BuildHammersley(int n)
{
    n = std::clamp(n, 1, kMaxIBLSamples);
    m_iblSamples = n;

    for (int k = 0; k < n; ++k)
    {
        // Van der Corput radical inverse for u
        float u = 0.0f;
        for (float p = 0.5f, kk = static_cast<float>(k); kk > 0.0f;
             p *= 0.5f, kk = std::floor(kk * 0.5f))
            if (static_cast<int>(kk) & 1)
                u += p;

        float v = (k + 0.5f) / static_cast<float>(n);
        m_hammersley[k] = glm::vec2(u, v);
    }
}

void Renderer::BakeIrradiance()
{
    // Allocate (or reallocate) the irradiance output texture (512x256 RGBA16F).
    // RGBA16F is required for the image2D binding in the compute shader.
    m_irradianceTex.CreateF16RGBA(512, 256);

    m_irradianceBakeShader.Bind();

    // Bind HDRI as a sampler on unit 0
    glActiveTexture(GL_TEXTURE0);
    glBindTexture(GL_TEXTURE_2D, m_hdriTex.ID());
    m_irradianceBakeShader.SetInt("uHDRITex", 0);

    // Bind irradiance texture as write-only image on binding point 0
    glBindImageTexture(0, m_irradianceTex.ID(), 0, GL_FALSE, 0, GL_WRITE_ONLY, GL_RGBA16F);

    // Dispatch: 512/8 = 64, 256/8 = 32 workgroups
    m_irradianceBakeShader.Dispatch(64, 32);

    m_irradianceBakeShader.Unbind();
}

// ---------------------------------------------------------------------------
// ComputeSHCoefficients  —  Project the HDRI onto 9 real SH basis functions
// (bands 0, 1, 2) on the CPU, pre-multiply by the cosine-lobe convolution
// factors, and upload to a UBO at binding point 2.
// This implements the Ramamoorthi & Hanrahan (2001) irradiance environment maps
// approach. The resulting E(N) = sum_k c[k]*Y_k(N) matches the texture-baked
// irradiance when both are driven by the same HDRI.
// ---------------------------------------------------------------------------
void Renderer::ComputeSHCoefficients(const float* pixels, int W, int H)
{
    for (auto& c : m_shCoeffs)
        c = glm::vec3(0.0f);

    const float PI = glm::pi<float>();
    const float dPhi = 2.0f * PI / float(W);
    const float dTheta = PI / float(H);

    for (int j = 0; j < H; ++j)
    {
        const float v = (j + 0.5f) / float(H);
        const float theta = PI * v;  // 0 = top (sky), PI = bottom
        const float sinT = std::sin(theta);
        const float cosT = std::cos(theta);
        const float weight = sinT * dTheta * dPhi;  // solid angle of this texel

        for (int i = 0; i < W; ++i)
        {
            const float u = (i + 0.5f) / float(W);
            const float phi = 2.0f * PI * (0.5f - u);  // matches shader's vectorOf()
            const float sinP = std::sin(phi);
            const float cosP = std::cos(phi);

            // World-space direction — Y-up, matching the shader's vectorOf()
            const glm::vec3 d(cosP * sinT, cosT, sinP * sinT);

            const float* px = &pixels[(j * W + i) * 3];
            const glm::vec3 L(px[0], px[1], px[2]);
            const glm::vec3 Lw = L * weight;

            // Project onto the 9 real SH basis functions
            m_shCoeffs[0] += Lw * 0.282095f;                              // Y_00
            m_shCoeffs[1] += Lw * 0.488603f * d.z;                        // Y_10
            m_shCoeffs[2] += Lw * 0.488603f * d.y;                        // Y_11e
            m_shCoeffs[3] += Lw * 0.488603f * d.x;                        // Y_11o
            m_shCoeffs[4] += Lw * 1.092548f * d.x * d.z;                  // Y_21
            m_shCoeffs[5] += Lw * 1.092548f * d.y * d.z;                  // Y_2m1
            m_shCoeffs[6] += Lw * 0.315392f * (3.0f * d.z * d.z - 1.0f);  // Y_20
            m_shCoeffs[7] += Lw * 1.092548f * d.x * d.y;                  // Y_2m2
            m_shCoeffs[8] += Lw * 0.546274f * (d.x * d.x - d.y * d.y);    // Y_22
        }
    }

    // Pre-multiply by the cosine-lobe convolution factors (Ramamoorthi & Hanrahan)
    //   Band 0 → A0 = PI
    //   Band 1 → A1 = 2*PI/3
    //   Band 2 → A2 = PI/4
    const float A0 = PI;
    const float A1 = 2.0f * PI / 3.0f;
    const float A2 = PI / 4.0f;
    m_shCoeffs[0] *= A0;
    m_shCoeffs[1] *= A1;
    m_shCoeffs[2] *= A1;
    m_shCoeffs[3] *= A1;
    m_shCoeffs[4] *= A2;
    m_shCoeffs[5] *= A2;
    m_shCoeffs[6] *= A2;
    m_shCoeffs[7] *= A2;
    m_shCoeffs[8] *= A2;

    // Upload to GPU — std140 pads vec3 to vec4
    glm::vec4 packed[9];
    for (int k = 0; k < 9; ++k)
        packed[k] = glm::vec4(m_shCoeffs[k], 0.0f);

    if (m_shCoeffsUBO == 0)
        glGenBuffers(1, &m_shCoeffsUBO);

    glBindBuffer(GL_UNIFORM_BUFFER, m_shCoeffsUBO);
    glBufferData(GL_UNIFORM_BUFFER, sizeof(packed), packed, GL_DYNAMIC_DRAW);
    glBindBuffer(GL_UNIFORM_BUFFER, 0);

    // Keep the UBO permanently attached to binding point 2
    glBindBufferBase(GL_UNIFORM_BUFFER, 2, m_shCoeffsUBO);

    std::cout << "[Renderer] SH coefficients computed from " << W << "x" << H << " HDRI\n";
}

static std::vector<std::string> ScanFolderForExtension(const std::string& folder,
                                                        const std::string& ext)
{
    std::vector<std::string> files;
    namespace fs = std::filesystem;
    std::error_code ec;
    for (const auto& entry : fs::directory_iterator(folder, ec))
    {
        if (!entry.is_regular_file(ec)) continue;
        std::string entryExt = entry.path().extension().string();
        std::transform(entryExt.begin(), entryExt.end(), entryExt.begin(), ::tolower);
        if (entryExt == ext)
            files.push_back(entry.path().filename().string());
    }
    std::sort(files.begin(), files.end());
    return files;
}

void Renderer::ScanHDRIFolder()
{
    m_hdriFiles = ScanFolderForExtension(kHDRIFolder, ".hdr");
    m_hdriSelectedIdx = -1;
}

bool Renderer::LoadHDRI(const std::string& path)
{
    // Decode once here and reuse the pixel buffer for both the GPU upload
    // and the CPU-side SH projection, instead of decoding the file twice.
    stbi_set_flip_vertically_on_load(false);
    int w = 0, h = 0, channels = 0;
    float* pixels = stbi_loadf(path.c_str(), &w, &h, &channels, 3);

    bool ok = pixels && m_hdriTex.UploadHDR(w, h, pixels);
    if (ok)
    {
        BakeIrradiance();
        ComputeSHCoefficients(pixels, w, h);
    }
    else
    {
        std::cerr << "[Renderer] Failed to load HDRI: " << path << "\n";
    }

    if (pixels)
        stbi_image_free(pixels);
    return ok;
}

void Renderer::ScanScenesFolder()
{
    m_sceneFiles = ScanFolderForExtension(kScenesFolder, ".json");
    m_sceneSelectedIdx = -1;

    for (size_t i = 0; i < m_sceneFiles.size(); ++i)
        if (std::string(kScenesFolder) + m_sceneFiles[i] == m_activeScenePath)
            m_sceneSelectedIdx = static_cast<int>(i);
}

bool Renderer::SwitchScene(const std::string& path)
{
    Scene loaded;
    if (!SceneLoader::Load(path, loaded))
        return false;

    m_activeScene = std::move(loaded);
    m_activeScenePath = path;
    m_selection = Selection{};

    LoadSkeletalObjects();

    m_lightCount = std::clamp(static_cast<int>(m_activeScene.lights.size()), 0, kMaxLights);
    for (int i = 0; i < kMaxLights; ++i)
    {
        if (i < m_lightCount)
        {
            const SceneLight& L = m_activeScene.lights[i];
            m_lights[i] = L;
            m_lightIntensity[i] = L.intensity;
            m_lightEnabled[i] = L.enabled;
        }
        else
        {
            m_lights[i] = Light{};
            m_lightIntensity[i] = 1.0f;
            m_lightEnabled[i] = false;
        }
    }

    const ScenePipeline& p = m_activeScene.pipeline;
    m_shadowsEnabled = p.shadowsEnabled;
    m_useMSM = p.useMSM;
    m_aoEnabled = p.aoEnabled;
    m_celEnabled = p.celEnabled;
    m_useLightVolumes = p.useLightVolumes;
    m_exposure = p.exposure;
    m_hdriRotation = p.hdriRotation;
    m_ambient = p.ambient;
    m_useSHIrradiance = p.useSHIrradiance;
    m_toonEnabled = p.toonEnabled;
    m_toonBands = p.toonBands;
    m_outlineThickness = p.outlineThickness;
    m_depthThreshold = p.depthThreshold;
    m_normalThreshold = p.normalThreshold;

    if (!p.hdriFile.empty())
    {
        m_lightingMode = LoadHDRI(std::string(kHDRIFolder) + p.hdriFile) ? LightingMode::IBL
                                                                         : LightingMode::PBS;
    }
    else
    {
        m_hdriTex.Destroy();
        m_irradianceTex.Destroy();
        m_lightingMode = LightingMode::PBS;
    }

    return true;
}

// Resolves the first scene object with a non-empty skeleton.modelFile (see Scene.h) into m_skeleton/m_animationClips/m_animator/m_skinnedMesh; only one is supported at a time, state is cleared first regardless of whether a new one is found.
void Renderer::LoadSkeletalObjects()
{
    m_skeletalObjectIndex = -1;
    m_skeleton = Anim::Skeleton{};
    m_animationClips.clear();
    m_animator = Anim::Animator{};
    m_animSelectedClipIndex = 0;
    m_animSelectedBoneIndex = 0;

    for (size_t i = 0; i < m_activeScene.objects.size(); ++i)
    {
        const SkeletonBinding& binding = m_activeScene.objects[i].skeleton;
        if (binding.modelFile.empty())
            continue;

        if (!SkeletalLoader::LoadSkeleton(binding.modelFile, m_skeleton))
        {
            std::cerr << "[Renderer] Failed to load skeleton from " << binding.modelFile << "\n";
            continue;
        }

        Geometry::SkinnedMeshData meshData;
        std::vector<Geometry::SubmeshRange> submeshes;
        if (!SkinnedModelLoader::Load(binding.modelFile, m_skeleton, meshData, submeshes))
        {
            std::cerr << "[Renderer] Failed to load skinned mesh from " << binding.modelFile
                      << "\n";
            m_skeleton = Anim::Skeleton{};
            continue;
        }
        m_skinnedMesh.Create(meshData.vertices, meshData.indices);
        m_skinnedSubmeshes = std::move(submeshes);

        for (const std::string& animPath : binding.animationFiles)
        {
            Anim::AnimationClip clip;
            if (SkeletalLoader::LoadAnimationClip(animPath, m_skeleton, clip))
                m_animationClips.push_back(std::move(clip));
        }

        if (!m_animationClips.empty())
            Anim::SetClip(m_animator, m_skeleton, m_animationClips[0]);

        m_skeletalObjectIndex = static_cast<int>(i);
        break;
    }

    ScanSkeletalModelsFolder();
}

void Renderer::ScanSkeletalModelsFolder()
{
    namespace fs = std::filesystem;
    std::error_code ec;

    // (display name, full path), sorted together by name.
    std::vector<std::pair<std::string, std::string>> found;
    for (const auto& entry : fs::directory_iterator(kModelsFolder, ec))
    {
        if (!entry.is_directory(ec))
            continue;

        std::string dirName = entry.path().filename().string();
        fs::path fbxPath = entry.path() / (dirName + ".fbx");
        if (fs::exists(fbxPath, ec))
            found.emplace_back(dirName, fbxPath.lexically_normal().generic_string());
    }
    std::sort(found.begin(), found.end());

    m_skeletalMeshNames.clear();
    m_skeletalMeshFiles.clear();
    for (auto& [name, path] : found)
    {
        m_skeletalMeshNames.push_back(std::move(name));
        m_skeletalMeshFiles.push_back(std::move(path));
    }

    m_skeletalMeshSelectedIdx = -1;
    if (m_skeletalObjectIndex < 0)
        return;

    const std::string& activeMeshFile = m_activeScene.objects[m_skeletalObjectIndex].skeleton.modelFile;
    for (size_t i = 0; i < m_skeletalMeshFiles.size(); ++i)
        if (m_skeletalMeshFiles[i] == activeMeshFile)
            m_skeletalMeshSelectedIdx = static_cast<int>(i);
}

bool Renderer::SwapSkeletalMesh(const std::string& meshFilePath)
{
    if (m_skeletalObjectIndex < 0)
        return false;

    Geometry::SkinnedMeshData meshData;
    std::vector<Geometry::SubmeshRange> submeshes;
    if (!SkinnedModelLoader::Load(meshFilePath, m_skeleton, meshData, submeshes))
    {
        std::cerr << "[Renderer] Failed to load skinned mesh from " << meshFilePath << "\n";
        return false;
    }

    m_skinnedMesh.Create(meshData.vertices, meshData.indices);
    m_skinnedSubmeshes = std::move(submeshes);
    return true;
}

void Renderer::Shutdown()
{
    DestroyScreenQuad();
    DestroyLightGizmoQuad();
    DestroyBoneLineBuffer();
    m_gbuffer.Destroy();
    m_aoRawBuffer.Destroy();
    m_aoBlurHBuffer.Destroy();
    m_aoBlurVBuffer.Destroy();
    for (auto& sm : m_shadowMaps)
        sm.Destroy();
    for (auto& mm : m_msmMaps)
        mm.Destroy();
    if (m_shCoeffsUBO)
    {
        glDeleteBuffers(1, &m_shCoeffsUBO);
        m_shCoeffsUBO = 0;
    }
    m_ready = false;
}

void Renderer::SetViewport(int w, int h)
{
    m_viewportW = (w > 0) ? w : 1;
    m_viewportH = (h > 0) ? h : 1;

    if (m_ready)
    {
        m_gbuffer.Resize(m_viewportW, m_viewportH);
        m_aoRawBuffer.Resize(m_viewportW, m_viewportH);
        m_aoBlurHBuffer.Resize(m_viewportW, m_viewportH);
        m_aoBlurVBuffer.Resize(m_viewportW, m_viewportH);
    }
}

void Renderer::RenderFrame(const Camera& camera, float deltaSeconds)
{
    if (!m_ready)
        return;

    m_cachedView = camera.GetView();
    m_cachedProj = camera.GetProj();

    Anim::Advance(m_animator, deltaSeconds);

    if (m_useDeferred)
        RenderDeferred(camera);
    else
        RenderForward(camera);
}

static constexpr const char* kModelMeshPrefix = "model:";

Mesh* Renderer::ResolveMesh(const std::string& ref)
{
    if (ref == "cornell") return &m_cornellMesh;
    if (ref == "ground") return &m_groundMesh;
    if (ref == "cube") return &m_cubeMesh;
    if (ref == "sphere") return &m_sphereMesh;

    if (ref.rfind(kModelMeshPrefix, 0) == 0)
    {
        std::string path = ref.substr(strlen(kModelMeshPrefix));

        auto it = m_modelMeshCache.find(path);
        if (it != m_modelMeshCache.end())
            return &it->second;

        if (m_failedModelLoads.count(path))
            return nullptr;

        Geometry::MeshData data;
        std::vector<Geometry::SubmeshRange> submeshes;
        if (!ModelLoader::Load(path, data, submeshes))
        {
            m_failedModelLoads.insert(path);
            return nullptr;
        }

        Mesh& mesh = m_modelMeshCache[path];
        mesh.Create(data.vertices, data.indices);
        m_modelSubmeshCache[path] = std::move(submeshes);
        m_raycastMeshes[ref] = BuildRaycastMesh(data.vertices, data.indices);
        return &mesh;
    }

    std::cerr << "[Renderer] Unknown meshRef: " << ref << "\n";
    return nullptr;
}

const std::vector<Geometry::SubmeshRange>* Renderer::ResolveSubmeshes(const std::string& ref)
{
    if (ref.rfind(kModelMeshPrefix, 0) != 0)
        return nullptr;

    std::string path = ref.substr(strlen(kModelMeshPrefix));
    auto it = m_modelSubmeshCache.find(path);
    return it != m_modelSubmeshCache.end() ? &it->second : nullptr;
}

Texture* Renderer::ResolveModelTexture(const std::string& path, bool srgb)
{
    auto it = m_modelTextureCache.find(path);
    if (it != m_modelTextureCache.end())
        return &it->second;

    if (m_failedTextureLoads.count(path))
        return nullptr;

    Texture& tex = m_modelTextureCache[path];
    if (!tex.LoadFromFile(path, srgb))
    {
        m_modelTextureCache.erase(path);
        m_failedTextureLoads.insert(path);
        return nullptr;
    }

    return &tex;
}
