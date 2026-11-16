#include "pch.h"
#include <algorithm>
#include <cmath>
#include <cstring>
#include <filesystem>
#include <glad/glad.h>
#include <glm/ext/matrix_transform.hpp>
#include <glm/gtc/constants.hpp>
#include <imgui.h>
#include <stb_image.h>

#include "graphics/Renderer.h"
#include "graphics/resources/Geometry.h"
#include "input/Input.h"
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

    if (!m_pathOverlayShader.LoadFromFiles("assets/shaders/path_overlay.vert",
                                           "assets/shaders/path_overlay.frag"))
        return false;

    if (!m_gbufferSkinnedShader.LoadFromFiles("assets/shaders/gbuffer_skinned.vert",
                                              "assets/shaders/gbuffer.frag"))
        return false;

    if (!m_shadowShader.LoadFromFiles("assets/shaders/shadow_depth.vert",
                                      "assets/shaders/shadow_depth.frag"))
        return false;

    if (!m_shadowSkinnedShader.LoadFromFiles("assets/shaders/shadow_depth_skinned.vert",
                                             "assets/shaders/shadow_depth.frag"))
        return false;

    if (!m_msmMomentShader.LoadFromFiles("assets/shaders/shadow_depth.vert",
                                         "assets/shaders/msm_moment_depth.frag"))
        return false;

    if (!m_msmMomentSkinnedShader.LoadFromFiles("assets/shaders/shadow_depth_skinned.vert",
                                                "assets/shaders/msm_moment_depth.frag"))
        return false;

    if (!m_sunShadowShader.LoadFromFiles("assets/shaders/shadow_depth_directional.vert",
                                         "assets/shaders/shadow_depth_directional.frag"))
        return false;

    if (!m_sunShadowSkinnedShader.LoadFromFiles(
            "assets/shaders/shadow_depth_directional_skinned.vert",
            "assets/shaders/shadow_depth_directional.frag"))
        return false;

    if (!m_msmBlurHShader.LoadComputeFromFile("assets/shaders/blur_h.comp"))
        return false;

    if (!m_msmBlurVShader.LoadComputeFromFile("assets/shaders/blur_v.comp"))
        return false;

    if (!m_irradianceBakeShader.LoadComputeFromFile("assets/shaders/irradiance_bake.comp"))
        return false;

    if (!m_aoShader.LoadFromFiles("assets/shaders/deferred_light.vert", "assets/shaders/ao.frag"))
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

    if (!m_sunShadowMap.Create(kSunShadowResolution))
        return false;

    // Gizmos
    if (!m_lightGizmoTex.LoadFromFile("assets/textures/light_gizmo_white.png",
                                      false))  // not sRGB, it's UI/debug
        return false;

    EnsureLightGizmoQuad();
    EnsureBoneLineBuffer();
    EnsurePathOverlayBuffers();

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

    auto ground = Geometry::MakeGroundPlane(kGroundHalfSize, 0.0f);
    m_groundMesh.Create(ground.vertices, ground.indices);
    m_raycastMeshes["ground"] = BuildRaycastMesh(ground.vertices, ground.indices);

    // Startup scene; also binds the path follower and requests the Frame Path camera.
    if (!SwitchScene(std::string(kScenesFolder) + "ik_reach.json"))
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

        // Second Hammersley coordinate: stratified evenly in [0,1].
        float v = (k + 0.5f) / static_cast<float>(n);
        m_hammersley[k] = glm::vec2(u, v);
    }
}

void Renderer::BakeIrradiance()
{
    // RGBA16F irradiance output (512x256), required for the compute shader's image2D binding
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

// Projects the HDRI onto 9 real SH basis functions (Ramamoorthi & Hanrahan 2001 irradiance
// environment maps) and uploads to the UBO at binding point 2.
void Renderer::ComputeSHCoefficients(const float* pixels, int W, int H)
{
    // Accumulated from scratch each time a new HDRI loads.
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

            const glm::vec3 d(cosP * sinT,
                              cosT,
                              sinP * sinT);  // world-space dir, Y-up, matches shader's vectorOf()

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

    // Cosine-lobe convolution factors per band (Ramamoorthi & Hanrahan): A0=PI, A1=2*PI/3, A2=PI/4
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

    glm::vec4 packed[9];  // std140 pads vec3 to vec4
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

// Approximates the HDRI's dominant light direction as a luminance-weighted mean of every texel's
// direction.
void Renderer::ComputeHDRISunDirection(const float* pixels, int width, int height)
{
    const float PI = glm::pi<float>();
    const float dPhi = 2.0f * PI / float(width);
    const float dTheta = PI / float(height);

    glm::vec3 weightedDirSum(0.0f);

    for (int j = 0; j < height; ++j)
    {
        const float v = (j + 0.5f) / float(height);
        const float theta = PI * v;  // 0 = top (sky), PI = bottom
        const float sinT = std::sin(theta);
        const float cosT = std::cos(theta);
        const float solidAngle = sinT * dTheta * dPhi;

        for (int i = 0; i < width; ++i)
        {
            const float u = (i + 0.5f) / float(width);
            const float phi = 2.0f * PI * (0.5f - u);  // matches shader's vectorOf()

            const glm::vec3 d(
                std::cos(phi) * sinT,
                cosT,
                std::sin(phi) * sinT);  // world-space dir, Y-up, matches shader's vectorOf()

            const float* px = &pixels[(j * width + i) * 3];
            const float luminance = 0.2126f * px[0] + 0.7152f * px[1] + 0.0722f * px[2];

            weightedDirSum += d * (luminance * solidAngle);
        }
    }

    const float len = glm::length(weightedDirSum);
    if (len > 1e-6f)
    {
        m_hdriSunDir = weightedDirSum / len;
        m_hasHDRISun = true;
    }
    else
    {
        m_hasHDRISun = false;
    }
}

static std::vector<std::string> ScanFolderForExtension(const std::string& folder,
                                                       const std::string& ext)
{
    std::vector<std::string> files;
    namespace fs = std::filesystem;
    std::error_code ec;
    for (const auto& entry : fs::directory_iterator(folder, ec))
    {
        if (!entry.is_regular_file(ec))
            continue;
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
    // Decode once, reuse the pixel buffer for both the GPU upload and the CPU-side SH projection
    stbi_set_flip_vertically_on_load(false);
    int w = 0, h = 0, channels = 0;
    float* pixels = stbi_loadf(path.c_str(), &w, &h, &channels, 3);

    bool ok = pixels && m_hdriTex.UploadHDR(w, h, pixels);
    if (ok)
    {
        BakeIrradiance();
        ComputeSHCoefficients(pixels, w, h);
        ComputeHDRISunDirection(pixels, w, h);
    }
    else
    {
        std::cerr << "[Renderer] Failed to load HDRI: " << path << "\n";
        m_hasHDRISun = false;
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
        m_lightingMode =
            LoadHDRI(std::string(kHDRIFolder) + p.hdriFile) ? LightingMode::IBL : LightingMode::PBS;
    }
    else
    {
        m_hdriTex.Destroy();
        m_irradianceTex.Destroy();
        m_lightingMode = LightingMode::PBS;
    }

    return true;
}

// Resolves the first scene object with a non-empty skeleton.modelFile (see Scene.h); only one
// skeletal object is supported at a time.
void Renderer::LoadSkeletalObjects()
{
    m_skeletalObjectIndex = -1;
    m_skeleton = Anim::Skeleton{};
    m_animationClips.clear();
    m_animator = Anim::Animator{};
    m_animSelectedClipIndex = 0;
    m_animSelectedBoneIndex = 0;
    m_ikTiers.clear();
    m_ikTargetIndex = -1;
    m_ikResult = {};

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
        BuildIKTiers();
        break;
    }

    ScanSkeletalModelsFolder();
    BindPathMotion();
    SetupReach();
}

void Renderer::BindPathMotion()
{
    m_pathFollower = Anim::PathFollower{};
    m_pathMotionActive = false;
    m_pathLocomotionIndex = -1;
    m_pathMotionStatus.clear();
    m_pathSettingsError.clear();
    m_timelineOwnsPose = false;
    m_pathOverlayDirty = true;
    m_pathDefaultPointsXZ.clear();
    m_pathSelectedPoint = -1;
    m_pathEditError.clear();

    const SceneObject* bound = nullptr;
    for (const SceneObject& obj : m_activeScene.objects)
    {
        if (!obj.pathMotion.controlPointsXZ.empty())
        {
            bound = &obj;
            break;
        }
    }
    if (!bound)
        return;

    if (m_skeletalObjectIndex < 0)
    {
        m_pathMotionStatus =
            "Path motion inactive: skeletal object '" + bound->name + "' failed to load";
        return;
    }
    if (&m_activeScene.objects[m_skeletalObjectIndex] != bound)
    {
        m_pathMotionStatus = "Path motion inactive: only the first skeletal object ('" +
                             m_activeScene.objects[m_skeletalObjectIndex].name +
                             "') can carry a path binding";
        return;
    }

    const PathMotionBinding& binding = bound->pathMotion;
    const LocomotionCalibration& calib = binding.locomotion[binding.activeLocomotion];
    int clipIndex = -1;
    for (size_t i = 0; i < m_animationClips.size(); ++i)
        if (m_animationClips[i].name == calib.clip)
            clipIndex = static_cast<int>(i);
    if (clipIndex < 0)
    {
        m_pathMotionStatus =
            "Path motion inactive: locomotion clip '" + calib.clip + "' failed to load";
        return;
    }

    std::vector<Anim::Vec3> points;
    points.reserve(binding.controlPointsXZ.size());
    for (const glm::vec2& p : binding.controlPointsXZ)
        points.push_back(Anim::Vec3{p.x, 0.0f, p.y});

    std::string error;
    Anim::CubicPath path;
    Anim::PathFollowerSettings settings;
    settings.cruiseSpeed = calib.cruiseSpeed * bound->scale.y;
    settings.easeInEnd = binding.easeInEnd;
    settings.easeOutStart = binding.easeOutStart;
    settings.lookAheadMeters = binding.lookAheadMeters;
    settings.strideMeters = calib.strideMeters * bound->scale.y;
    if (!path.SetControlPoints(points, &error) ||
        !m_pathFollower.Initialize(path, settings, {}, &error) ||
        !ValidatePathOnFloor(m_pathFollower, error))
    {
        m_pathFollower = Anim::PathFollower{};
        m_pathMotionStatus = "Path motion inactive: " + error;
        return;
    }

    m_pathMotionActive = true;
    UpdatePathBounds();
    m_pathDefaultPointsXZ = binding.controlPointsXZ;
    m_animSelectedClipIndex = clipIndex;
    Anim::SetClip(m_animator, m_skeleton, m_animationClips[clipIndex]);
    SyncPathLocomotion();
    char summary[128];
    snprintf(summary,
             sizeof(summary),
             "Path motion ready: %zu points, %.2f m",
             points.size(),
             m_pathFollower.GetTables().GetTotalLength());
    m_pathMotionStatus = summary;
    RequestPathCameraFrame();
}

bool Renderer::TryApplyPathPoints(const std::vector<glm::vec2>& pointsXZ, std::string& error)
{
    std::vector<Anim::Vec3> points;
    points.reserve(pointsXZ.size());
    for (const glm::vec2& p : pointsXZ)
        points.push_back(Anim::Vec3{p.x, 0.0f, p.y});

    Anim::CubicPath path;
    if (!path.SetControlPoints(points, &error))
        return false;

    Anim::PathFollower candidate;
    if (!candidate.Initialize(path,
                              m_pathFollower.GetSettings(),
                              m_pathFollower.GetTables().GetOptions(),
                              &error))
        return false;

    if (!ValidatePathOnFloor(candidate, error))
        return false;

    // Same normalized lap progress and gait cycle on the reshaped path, so an edit moves the
    // character without changing its pose.
    const Anim::PathFollowerState& old = m_pathFollower.GetState();
    candidate.Seek(old.normalizedDistance, old.lapCount);
    candidate.ReanchorGait(old.gaitCycles);
    candidate.SetPlaying(old.playing);
    m_pathFollower = std::move(candidate);

    m_activeScene.objects[m_skeletalObjectIndex].pathMotion.controlPointsXZ = pointsXZ;
    m_pathOverlayDirty = true;
    UpdatePathBounds();
    return true;
}

// Uniform Catmull-Rom can overshoot between points, so the whole sampled curve is checked, not just
// the points.
bool Renderer::ValidatePathOnFloor(const Anim::PathFollower& follower, std::string& error) const
{
    for (const SceneObject& obj : m_activeScene.objects)
    {
        if (obj.meshRef != "ground")
            continue;
        constexpr float kMargin = 0.3f;
        glm::vec2 center(obj.position.x, obj.position.z);
        glm::vec2 half = glm::vec2(kGroundHalfSize * obj.scale.x, kGroundHalfSize * obj.scale.z) -
                         glm::vec2(kMargin);
        const Anim::CubicPath& path = follower.GetPath();
        for (const Anim::PathArcLengthSample& s : follower.GetTables().GetGlobalSamples())
        {
            Anim::Vec3 p =
                path.Evaluate(s.location.segmentIndex, static_cast<float>(s.location.parameter));
            if (std::fabs(p.x - center.x) > half.x || std::fabs(p.z - center.y) > half.y)
            {
                char msg[96];
                snprintf(msg, sizeof(msg), "curve leaves the floor near (%.1f, %.1f)", p.x, p.z);
                error = msg;
                return false;
            }
        }
        break;
    }
    return true;
}

void Renderer::UpdatePathBounds()
{
    const Anim::CubicPath& path = m_pathFollower.GetPath();
    glm::vec3 lo(1e30f), hi(-1e30f);
    for (const Anim::PathArcLengthSample& s : m_pathFollower.GetTables().GetGlobalSamples())
    {
        Anim::Vec3 p =
            path.Evaluate(s.location.segmentIndex, static_cast<float>(s.location.parameter));
        lo = glm::min(lo, glm::vec3(p.x, p.y, p.z));
        hi = glm::max(hi, glm::vec3(p.x, p.y, p.z));
    }
    m_pathBoundsMin = lo;
    m_pathBoundsMax = hi;
}

void Renderer::RequestPathCameraFrame()
{
    m_cameraFrameRequested = m_pathMotionActive;
}

bool Renderer::ConsumeCameraFrameRequest(float fovY, glm::vec3& position, glm::vec3& target)
{
    if (!m_cameraFrameRequested)
        return false;
    m_cameraFrameRequested = false;

    // Fit the path's bounding sphere in the vertical FOV, looking down from the +Z side at ~40
    // degrees.
    glm::vec3 center = 0.5f * (m_pathBoundsMin + m_pathBoundsMax);
    float radius = std::max(0.5f * glm::length(m_pathBoundsMax - m_pathBoundsMin), 1.0f);
    float distance = 1.15f * radius / std::sin(fovY * 0.5f);
    target = center;
    position = center + glm::normalize(glm::vec3(0.0f, 0.65f, 0.76f)) * distance;
    return true;
}

void Renderer::SyncPathLocomotion()
{
    m_pathLocomotionIndex = -1;
    if (!m_pathMotionActive)
        return;

    const SceneObject& obj = m_activeScene.objects[m_skeletalObjectIndex];
    const PathMotionBinding& binding = obj.pathMotion;
    if (m_animator.clip)
        for (size_t i = 0; i < binding.locomotion.size(); ++i)
            if (binding.locomotion[i].clip == m_animator.clip->name)
                m_pathLocomotionIndex = static_cast<int>(i);

    // Mirror the panel-edited binding into the follower; SetSettings keeps distance and gait phase,
    // so edits never snap.
    Anim::PathFollowerSettings settings = m_pathFollower.GetSettings();
    Anim::PathFollowerSettings wanted = settings;
    wanted.easeInEnd = binding.easeInEnd;
    wanted.easeOutStart = binding.easeOutStart;
    wanted.lookAheadMeters = binding.lookAheadMeters;
    if (m_pathLocomotionIndex >= 0)
    {
        // Stride and speed scale with the object so a resized character keeps the same cadence.
        const LocomotionCalibration& calib = binding.locomotion[m_pathLocomotionIndex];
        float stride = calib.strideMeters;
        float cruise = calib.cruiseSpeed;
        // Mid-blend, stride and speed follow the pose weight so the feet keep matching the ground.
        if (m_animator.blendFromClip)
        {
            float weight = Anim::BlendWeight(m_animator);
            for (const LocomotionCalibration& from : binding.locomotion)
            {
                if (from.clip != m_animator.blendFromClip->name)
                    continue;
                stride = from.strideMeters + (calib.strideMeters - from.strideMeters) * weight;
                cruise = from.cruiseSpeed + (calib.cruiseSpeed - from.cruiseSpeed) * weight;
                break;
            }
        }
        wanted.strideMeters = stride * obj.scale.y;
        wanted.cruiseSpeed = cruise * obj.scale.y;
    }

    if (wanted.easeInEnd == settings.easeInEnd && wanted.easeOutStart == settings.easeOutStart &&
        wanted.lookAheadMeters == settings.lookAheadMeters &&
        wanted.strideMeters == settings.strideMeters && wanted.cruiseSpeed == settings.cruiseSpeed)
        return;

    std::string error;
    if (!m_pathFollower.SetSettings(wanted, &error))
        m_pathSettingsError = error;
    else
        m_pathSettingsError.clear();
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

        std::vector<std::string> fbxNames;
        for (const auto& sub : fs::directory_iterator(entry.path(), ec))
            if (sub.is_regular_file(ec) && sub.path().extension() == ".fbx")
                fbxNames.push_back(sub.path().filename().string());
        if (fbxNames.size() != 1)
            continue;

        std::string dirName = entry.path().filename().string();
        fs::path fbxPath = entry.path() / fbxNames.front();
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

    const std::string& activeMeshFile =
        m_activeScene.objects[m_skeletalObjectIndex].skeleton.modelFile;
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
    size_t unresolvedInfluences = 0;
    if (!SkinnedModelLoader::Load(meshFilePath,
                                  m_skeleton,
                                  meshData,
                                  submeshes,
                                  &unresolvedInfluences))
    {
        std::cerr << "[Renderer] Failed to load skinned mesh from " << meshFilePath << "\n";
        return false;
    }

    if (unresolvedInfluences > 0)
    {
        std::cerr << "[Renderer] " << meshFilePath
                  << " has a different rig than the active skeleton, keeping current mesh\n";
        return false;
    }

    std::vector<Anim::Mat4> inverseBindPoses;
    if (!SkeletalLoader::LoadInverseBindPoses(meshFilePath, m_skeleton, inverseBindPoses))
    {
        std::cerr << "[Renderer] Failed to load bind pose from " << meshFilePath
                  << ", keeping current mesh\n";
        return false;
    }

    m_skinnedMesh.Create(meshData.vertices, meshData.indices);
    m_skinnedSubmeshes = std::move(submeshes);
    Anim::SetMeshBindPose(m_animator, std::move(inverseBindPoses));
    return true;
}

void Renderer::Shutdown()
{
    DestroyScreenQuad();
    DestroyLightGizmoQuad();
    DestroyBoneLineBuffer();
    DestroyPathOverlayBuffers();
    m_gbuffer.Destroy();
    m_aoRawBuffer.Destroy();
    m_aoBlurHBuffer.Destroy();
    m_aoBlurVBuffer.Destroy();
    for (auto& sm : m_shadowMaps)
        sm.Destroy();
    for (auto& mm : m_msmMaps)
        mm.Destroy();
    m_sunShadowMap.Destroy();
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
        if (!m_gbuffer.Resize(m_viewportW, m_viewportH))
            std::cerr << "[Renderer] GBuffer resize to " << m_viewportW << "x" << m_viewportH
                      << " failed\n";
        if (!m_aoRawBuffer.Resize(m_viewportW, m_viewportH))
            std::cerr << "[Renderer] AO raw buffer resize failed\n";
        if (!m_aoBlurHBuffer.Resize(m_viewportW, m_viewportH))
            std::cerr << "[Renderer] AO blur-H buffer resize failed\n";
        if (!m_aoBlurVBuffer.Resize(m_viewportW, m_viewportH))
            std::cerr << "[Renderer] AO blur-V buffer resize failed\n";
    }
}

// Refreshes the view/proj matrices DrawDebugUI's gizmo and raycast picking read, so they're never
// stale.
void Renderer::UpdateCameraCache(const Camera& camera)
{
    m_cachedView = camera.GetView();
    m_cachedProj = camera.GetProj();
}

void Renderer::Update(float deltaSeconds)
{
    if (!m_ready)
        return;

    // A stall (debugger, window drag, minimize) would otherwise jump the character several laps in
    // one frame.
    deltaSeconds = std::min(deltaSeconds, kMaxSimulationStep);

    if (m_pathMotionActive)
    {
        // Ahead of the sync so speed, stride and pose all use this frame's blend weight.
        if (m_animator.playing)
            Anim::AdvanceBlend(m_animator, deltaSeconds);
        // Applies panel edits and Inspector scale changes from the previous UI pass; no-op when
        // nothing changed.
        SyncPathLocomotion();
        // Shared play/pause: the Animation panel's Pause also stops travel.
        if (m_animator.playing)
            m_timelineOwnsPose = false;
        if (m_pathFollower.GetState().playing != m_animator.playing)
            m_pathFollower.SetPlaying(m_animator.playing);
        try
        {
            m_pathFollower.Advance(!m_reachActive || m_reachWalking ? deltaSeconds : 0.0f);
        }
        catch (const std::exception& e)
        {
            m_pathFollower.Reset();
            m_pathSettingsError = std::string("Playback reset to lap start: ") + e.what();
        }
    }

    if (m_pathMotionActive && m_pathPaceCoupled && m_pathLocomotionIndex >= 0)
    {
        // Clip time comes from travelled distance; an Animation-timeline scrub keeps the pose until
        // Play/scrub/reset.
        if (!m_timelineOwnsPose)
            Anim::SeekTo(m_animator,
                         static_cast<float>(m_pathFollower.GetState().gaitPhase) *
                             m_animator.clip->duration);
    }
    else
    {
        Anim::Advance(m_animator, deltaSeconds);
    }

    UpdateIKTargetInput(deltaSeconds);
    ClampIKTarget();
    UpdateReach(deltaSeconds);
}

void Renderer::BuildIKTiers()
{
    m_ikTiers.clear();
    m_ikTargetIndex = -1;
    if (m_skeletalObjectIndex < 0)
        return;

    const IKReachBinding& binding = m_activeScene.objects[m_skeletalObjectIndex].ikReach;
    if (binding.effectorBone.empty())
        return;

    std::vector<int> bones;
    for (const IKJointBinding& joint : binding.chain)
        bones.push_back(Anim::FindBone(m_skeleton, joint.bone));
    bones.push_back(Anim::FindBone(m_skeleton, binding.effectorBone));
    for (size_t i = 0; i < bones.size(); ++i)
    {
        const std::string& name =
            i < binding.chain.size() ? binding.chain[i].bone : binding.effectorBone;
        if (bones[i] < 0)
        {
            std::cerr << "[Renderer] ikReach bone '" << name << "' not found\n";
            return;
        }
        if (i > 0 && m_skeleton.bones[bones[i]].parentIndex != bones[i - 1])
        {
            std::cerr << "[Renderer] ikReach bone '" << name
                      << "' is not a child of the previous chain bone\n";
            return;
        }
    }

    int targetIndex = -1;
    for (size_t i = 0; i < m_activeScene.objects.size(); ++i)
    {
        if (static_cast<int>(i) != m_skeletalObjectIndex &&
            m_activeScene.objects[i].name == binding.targetObject)
        {
            targetIndex = static_cast<int>(i);
            break;
        }
    }
    if (targetIndex < 0)
    {
        std::cerr << "[Renderer] ikReach target '" << binding.targetObject << "' not found\n";
        return;
    }

    std::vector<Anim::VQS> bindWorld(m_skeleton.bones.size());
    for (size_t i = 0; i < m_skeleton.bones.size(); ++i)
    {
        int parent = m_skeleton.bones[i].parentIndex;
        bindWorld[i] = Anim::Concat(parent >= 0 ? bindWorld[parent] : Anim::VQS{},
                                    m_skeleton.bones[i].localBindPose);
    }

    auto toAnim = [](const glm::vec3& v) { return Anim::Vec3{v.x, v.y, v.z}; };

    std::vector<Anim::IKJoint> joints;
    for (size_t i = 0; i < binding.chain.size(); ++i)
    {
        const IKJointBinding& source = binding.chain[i];
        int parent = m_skeleton.bones[bones[i]].parentIndex;
        Anim::Quat parentBind = parent >= 0 ? bindWorld[parent].q : Anim::Quat{};

        Anim::IKJoint joint;
        joint.bone = bones[i];
        joint.stiffness = source.stiffness;
        joint.limit.type = source.limit == IKLimitType::Cone    ? Anim::JointLimitType::Cone
                           : source.limit == IKLimitType::Hinge ? Anim::JointLimitType::Hinge
                                                                : Anim::JointLimitType::None;
        joint.limit.axis =
            Anim::Rotate(Anim::Conjugate(parentBind), Anim::Normalize(toAnim(source.axis)));
        joint.limit.followAnimation = source.followAnimation;
        joint.limit.minAngle = glm::radians(source.minDegrees);
        joint.limit.maxAngle = glm::radians(source.maxDegrees);
        joint.limit.maxTwist = glm::radians(source.maxTwistDegrees);
        joints.push_back(joint);
    }

    Anim::IKJoint effector;
    effector.bone = bones.back();

    size_t previousSize = 0;
    for (int tier = 1; tier <= binding.chain.front().tier; ++tier)
    {
        size_t first = joints.size();
        while (first > 0 && binding.chain[first - 1].tier <= tier)
            --first;
        if (joints.size() - first == previousSize)
            continue;
        previousSize = joints.size() - first;

        std::vector<Anim::IKJoint> chain(joints.begin() + first, joints.end());
        chain.push_back(effector);
        m_ikTiers.push_back(std::move(chain));
    }

    m_ikPole = Anim::Normalize(toAnim(binding.pole));
    m_ikTargetIndex = targetIndex;
}

bool Renderer::MouseRay(glm::vec3& origin, glm::vec3& direction) const
{
    ImGuiIO& io = ImGui::GetIO();
    if (io.DisplaySize.x <= 0.0f || io.DisplaySize.y <= 0.0f)
        return false;

    ImVec2 mp = ImGui::GetMousePos();
    float ndcX = (mp.x / io.DisplaySize.x) * 2.0f - 1.0f;
    float ndcY = 1.0f - (mp.y / io.DisplaySize.y) * 2.0f;

    glm::mat4 invVP = glm::inverse(m_cachedProj * m_cachedView);
    glm::vec4 nearP = invVP * glm::vec4(ndcX, ndcY, -1.0f, 1.0f);
    glm::vec4 farP = invVP * glm::vec4(ndcX, ndcY, 1.0f, 1.0f);
    nearP /= nearP.w;
    farP /= farP.w;

    origin = glm::vec3(nearP);
    direction = glm::normalize(glm::vec3(farP - nearP));
    return true;
}

void Renderer::UpdateIKTargetInput(float deltaSeconds)
{
    if (m_ikTargetIndex < 0)
        return;

    glm::vec3& target = m_activeScene.objects[m_ikTargetIndex].position;
    ImGuiIO& io = ImGui::GetIO();

    if (!io.WantCaptureMouse && io.KeyShift && Input::MouseDown(GLFW_MOUSE_BUTTON_LEFT) &&
        !ImGuizmo::IsOver() && !ImGuizmo::IsUsing())
    {
        glm::vec3 origin, direction;
        if (MouseRay(origin, direction) && direction.y < -1e-4f)
        {
            float t = -origin.y / direction.y;
            if (t > 0.0f)
            {
                target.x = origin.x + direction.x * t;
                target.z = origin.z + direction.z * t;
            }
        }
    }

    if (io.WantCaptureKeyboard)
        return;

    glm::mat4 cameraToWorld = glm::inverse(m_cachedView);
    glm::vec3 forward(-cameraToWorld[2].x, 0.0f, -cameraToWorld[2].z);
    forward =
        glm::dot(forward, forward) > 1e-8f ? glm::normalize(forward) : glm::vec3(0.0f, 0.0f, -1.0f);
    glm::vec3 right(-forward.z, 0.0f, forward.x);

    glm::vec2 move(0.0f);
    if (Input::KeyDown(GLFW_KEY_RIGHT))
        move.x += 1.0f;
    if (Input::KeyDown(GLFW_KEY_LEFT))
        move.x -= 1.0f;
    if (Input::KeyDown(GLFW_KEY_UP))
        move.y += 1.0f;
    if (Input::KeyDown(GLFW_KEY_DOWN))
        move.y -= 1.0f;
    if (move != glm::vec2(0.0f))
    {
        move = glm::normalize(move);
        target += (right * move.x + forward * move.y) * (kIKTargetMoveSpeed * deltaSeconds);
    }

    float lift = 0.0f;
    if (Input::KeyDown(GLFW_KEY_PAGE_UP))
        lift += 1.0f;
    if (Input::KeyDown(GLFW_KEY_PAGE_DOWN))
        lift -= 1.0f;
    target.y += lift * kIKTargetLiftSpeed * deltaSeconds;
}

void Renderer::ClampIKTarget()
{
    if (m_ikTargetIndex < 0)
        return;

    glm::vec3& target = m_activeScene.objects[m_ikTargetIndex].position;
    target.x = std::clamp(target.x, -kGroundHalfSize, kGroundHalfSize);
    target.z = std::clamp(target.z, -kGroundHalfSize, kGroundHalfSize);
    target.y = std::clamp(target.y, kIKTargetMinHeight, kIKTargetMaxHeight);
}

void Renderer::SetupReach()
{
    m_reachActive = false;
    m_reachEnabled = true;
    m_reachWalking = false;
    m_reachPathValid = false;
    m_reachIdleClip = -1;
    m_reachWalkClip = -1;
    m_reachRootPosition = {};
    m_reachRootForward = {0.0f, 0.0f, 1.0f};
    m_reach = Anim::ReachController{};
    m_reachOutput = {};
    m_reachStatus.clear();

    if (m_skeletalObjectIndex < 0 || m_ikTiers.empty() || m_ikTargetIndex < 0)
        return;
    if (!m_pathMotionActive)
    {
        m_reachStatus = "IK reach inactive: walking needs an active pathMotion binding";
        return;
    }

    const SceneObject& obj = m_activeScene.objects[m_skeletalObjectIndex];
    const IKReachBinding& binding = obj.ikReach;
    const PathMotionBinding& path = obj.pathMotion;
    auto findClip = [&](const std::string& name)
    {
        for (size_t i = 0; i < m_animationClips.size(); ++i)
            if (m_animationClips[i].name == name)
                return static_cast<int>(i);
        return -1;
    };
    m_reachIdleClip = findClip(binding.idleClip);
    m_reachWalkClip = findClip(path.locomotion[path.activeLocomotion].clip);
    if (m_reachIdleClip < 0 || m_reachWalkClip < 0)
    {
        m_reachStatus = "IK reach inactive: the idle or walk clip failed to load";
        return;
    }

    Anim::ReachSettings settings;
    settings.reachSeconds = binding.reachSeconds;
    settings.settleSeconds = path.locomotionBlendSeconds;
    Anim::ApproachSettings approach;
    approach.standoff = binding.standoffMeters;
    approach.minStandoff = std::min(approach.minStandoff, approach.standoff);
    try
    {
        m_reach.SetSettings(settings, approach);
    }
    catch (const std::exception& e)
    {
        m_reachStatus = std::string("IK reach inactive: ") + e.what();
        return;
    }

    m_reachActive = true;
    m_animSelectedClipIndex = m_reachIdleClip;
    Anim::SetClip(m_animator, m_skeleton, m_animationClips[m_reachIdleClip]);
    SyncPathLocomotion();
}

void Renderer::UpdateReach(float deltaSeconds)
{
    m_ikResult = {};
    if (!m_reachActive || m_animator.localPose.size() != m_skeleton.bones.size() ||
        m_animator.worldPose.size() != m_skeleton.bones.size())
        return;

    if (m_reachWalking)
    {
        const Anim::PathFollowerState& state = m_pathFollower.GetState();
        m_reachRootPosition = state.position;
        m_reachRootForward = state.forward;
    }

    glm::mat4 model = GetObjectModelMatrix(m_skeletalObjectIndex);
    auto worldOf = [&](int bone)
    {
        const Anim::Vec3& p = m_animator.worldPose[bone].v;
        glm::vec3 w = glm::vec3(model * glm::vec4(p.x, p.y, p.z, 1.0f));
        return Anim::Vec3{w.x, w.y, w.z};
    };

    const std::vector<Anim::IKJoint>& arm = m_ikTiers.front();
    float reach = 0.0f;
    for (size_t i = 1; i < arm.size(); ++i)
        reach += Anim::Length(worldOf(arm[i].bone) - worldOf(arm[i - 1].bone));

    const SceneObject& target = m_activeScene.objects[m_ikTargetIndex];
    Anim::ReachInput input;
    input.hasTarget = m_reachEnabled && target.visible;
    input.approach.position = m_reachRootPosition;
    input.approach.forward = m_reachRootForward;
    input.approach.shoulder = worldOf(arm.front().bone);
    input.approach.target = Anim::Vec3{target.position.x, target.position.y, target.position.z};
    input.approach.reach = reach;
    input.hand = worldOf(arm.back().bone);
    input.walkFinished = m_reachWalking && m_pathFollower.GetState().finished;

    bool wasTurning = m_reach.GetState() == Anim::ReachState::Turning;
    try
    {
        m_reachOutput = m_reach.Update(m_animator.playing ? deltaSeconds : 0.0f, input);
    }
    catch (const std::exception& e)
    {
        m_reachStatus = std::string("IK reach stopped: ") + e.what();
        m_reachEnabled = false;
        m_reach.Reset();
        m_reachOutput = {};
        if (m_reachWalking)
            SettleReach();
        return;
    }

    if (m_reachOutput.event == Anim::ReachEvent::StartWalk)
        StartReachWalk();
    else if (m_reachOutput.event == Anim::ReachEvent::Settle)
        SettleReach();

    if (wasTurning || m_reach.GetState() == Anim::ReachState::Turning)
        m_reachRootForward = m_reachOutput.facing;

    if (m_reachOutput.weight > 0.0f)
        ApplyReachIK(m_reachOutput.weight, m_reachOutput.goal);
}

void Renderer::StartReachWalk()
{
    std::string error;
    Anim::CubicPath path;
    Anim::PathFollower follower;
    if (!Anim::BuildApproachPath(m_reach.GetPlan(), path, &error) ||
        !follower.Initialize(path,
                             m_pathFollower.GetSettings(),
                             m_pathFollower.GetTables().GetOptions(),
                             &error) ||
        !ValidatePathOnFloor(follower, error))
    {
        m_reachStatus = "Approach path rejected: " + error;
        m_reachEnabled = false;
        m_reach.Reset();
        m_reachOutput = {};
        if (m_reachWalking)
            SettleReach();
        return;
    }

    if (m_reachWalking)
        follower.ReanchorGait(m_pathFollower.GetState().gaitCycles);
    follower.SetPlaying(m_animator.playing);
    m_pathFollower = std::move(follower);
    m_pathOverlayDirty = true;
    m_reachPathValid = true;
    m_reachStatus.clear();
    UpdatePathBounds();

    if (!m_reachWalking)
    {
        const PathMotionBinding& binding = m_activeScene.objects[m_skeletalObjectIndex].pathMotion;
        Anim::BeginBlendTo(m_animator,
                           m_skeleton,
                           m_animationClips[m_reachWalkClip],
                           binding.locomotionBlendSeconds,
                           false);
        m_animSelectedClipIndex = m_reachWalkClip;
        m_timelineOwnsPose = false;
        m_reachWalking = true;
        SyncPathLocomotion();
    }
}

void Renderer::SettleReach()
{
    m_reachWalking = false;
    Anim::BeginBlendTo(m_animator,
                       m_skeleton,
                       m_animationClips[m_reachIdleClip],
                       m_reach.GetSettings().settleSeconds,
                       false);
    m_animSelectedClipIndex = m_reachIdleClip;
    m_timelineOwnsPose = false;
    SyncPathLocomotion();
}

void Renderer::ApplyReachIK(float weight, const Anim::Vec3& goal)
{
    glm::mat4 worldToModel = glm::inverse(GetObjectModelMatrix(m_skeletalObjectIndex));
    glm::vec3 target = glm::vec3(worldToModel * glm::vec4(goal.x, goal.y, goal.z, 1.0f));

    Anim::ChainSolveSettings settings;
    settings.useConstraints = m_ikUseConstraints;
    settings.usePole = m_ikUsePole;
    settings.pole = m_ikPole;

    std::vector<std::vector<Anim::IKJoint>> tiers = m_ikTiers;
    if (!m_ikUseSpine)
        tiers.resize(1);

    const std::vector<Anim::VQS> animated = m_animator.localPose;
    m_ikResult = Anim::SolvePrioritized(m_skeleton,
                                        tiers,
                                        Anim::Vec3{target.x, target.y, target.z},
                                        settings,
                                        m_animator.localPose,
                                        m_animator.worldPose);
    if (weight < 1.0f)
        for (const Anim::IKJoint& joint : m_ikTiers.back())
            m_animator.localPose[joint.bone].q =
                Anim::Slerp(animated[joint.bone].q, m_animator.localPose[joint.bone].q, weight);
    Anim::RebuildWorldPose(m_animator);
}

glm::mat4 Renderer::GetPathPoseMatrix(int objectIndex) const
{
    if (!m_pathMotionActive || objectIndex < 0 || objectIndex != m_skeletalObjectIndex)
        return glm::mat4(1.0f);

    const PathMotionBinding& binding = m_activeScene.objects[objectIndex].pathMotion;
    const Anim::PathFollowerState& state = m_pathFollower.GetState();
    const Anim::Vec3& root = m_reachActive ? m_reachRootPosition : state.position;
    const Anim::Vec3& forward = m_reachActive ? m_reachRootForward : state.forward;

    // Follower orientation is yaw-only, mapping local +Z onto the XZ heading.
    float headingYaw = std::atan2(forward.x, forward.z);
    glm::vec3 position(root.x, root.y + binding.floorOffset, root.z);

    glm::mat4 M = glm::translate(glm::mat4(1.0f), position);
    return glm::rotate(M,
                       headingYaw + glm::radians(binding.facingYawDegrees),
                       glm::vec3(0.0f, 1.0f, 0.0f));
}

glm::mat4 Renderer::GetObjectModelMatrix(int objectIndex) const
{
    if (objectIndex < 0 || objectIndex >= static_cast<int>(m_activeScene.objects.size()))
        return glm::mat4(1.0f);
    return GetPathPoseMatrix(objectIndex) * ComputeModelMatrix(m_activeScene.objects[objectIndex]);
}

void Renderer::RenderFrame(const Camera& camera)
{
    if (!m_ready)
        return;

    RefreshBoneMatrixUpload();

    if (m_useDeferred)
        RenderDeferred(camera);
    else
        RenderForward(camera);
}

// meshRef "model:<path>" loads an external file on first use; other refs name built-in meshes.
static constexpr const char* kModelMeshPrefix = "model:";

Mesh* Renderer::ResolveMesh(const std::string& ref)
{
    if (ref == "cornell")
        return &m_cornellMesh;
    if (ref == "ground")
        return &m_groundMesh;
    if (ref == "cube")
        return &m_cubeMesh;
    if (ref == "sphere")
        return &m_sphereMesh;

    if (ref.rfind(kModelMeshPrefix, 0) == 0)
    {
        std::string path = ref.substr(strlen(kModelMeshPrefix));

        auto it = m_modelMeshCache.find(path);
        if (it != m_modelMeshCache.end())
            return &it->second;

        // Remember failures so a missing file isn't re-read every frame.
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
        // Keyed by the full meshRef, matching how RaycastScene looks objects up.
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
    // The same file can be needed as both sRGB color and linear data, so the color space is part of
    // the key.
    const std::string key = path + (srgb ? "|srgb" : "|linear");
    auto it = m_modelTextureCache.find(key);
    if (it != m_modelTextureCache.end())
        return &it->second;

    if (m_failedTextureLoads.count(key))
        return nullptr;

    Texture& tex = m_modelTextureCache[key];
    if (!tex.LoadFromFile(path, srgb))
    {
        m_modelTextureCache.erase(key);
        m_failedTextureLoads.insert(key);
        return nullptr;
    }

    return &tex;
}
