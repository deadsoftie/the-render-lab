#include "pch.h"
#include <algorithm>
#include <cmath>
#include <cstring>
#include <filesystem>
#include <glad/glad.h>
#include <glm/ext/matrix_transform.hpp>
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

    auto ground = Geometry::MakeGroundPlane(12.0f, 0.0f);
    m_groundMesh.Create(ground.vertices, ground.indices);
    m_raycastMeshes["ground"] = BuildRaycastMesh(ground.vertices, ground.indices);

    if (!SwitchScene(std::string(kScenesFolder) + "path_motion.json"))
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

// Projects the HDRI onto 9 real SH basis functions (Ramamoorthi & Hanrahan 2001 irradiance environment maps) and uploads to the UBO at binding point 2.
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

            const glm::vec3 d(cosP * sinT, cosT, sinP * sinT);  // world-space dir, Y-up, matches shader's vectorOf()

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

// Approximates the HDRI's dominant light direction as a luminance-weighted mean of every texel's direction.
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

            const glm::vec3 d(std::cos(phi) * sinT, cosT, std::sin(phi) * sinT);  // world-space dir, Y-up, matches shader's vectorOf()

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

// Resolves the first scene object with a non-empty skeleton.modelFile (see Scene.h); only one skeletal object is supported at a time.
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
    BindPathMotion();
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

    if (m_skeletalObjectIndex < 0 || &m_activeScene.objects[m_skeletalObjectIndex] != bound)
    {
        m_pathMotionStatus = "Path motion inactive: skeletal object '" + bound->name +
                             "' failed to load";
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
        m_pathMotionStatus = "Path motion inactive: locomotion clip '" + calib.clip +
                             "' failed to load";
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
    if (!path.SetControlPoints(points, &error) || !m_pathFollower.Initialize(path, settings, {}, &error))
    {
        m_pathMotionStatus = "Path motion inactive: " + error;
        return;
    }

    m_pathMotionActive = true;
    m_pathDefaultPointsXZ = binding.controlPointsXZ;
    m_animSelectedClipIndex = clipIndex;
    Anim::SetClip(m_animator, m_skeleton, m_animationClips[clipIndex]);
    SyncPathLocomotion();
    char summary[128];
    snprintf(summary, sizeof(summary), "Path motion ready: %zu points, %.2f m",
             points.size(), m_pathFollower.GetTables().GetTotalLength());
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
    if (!candidate.Initialize(path, m_pathFollower.GetSettings(),
                              m_pathFollower.GetTables().GetOptions(), &error))
        return false;

    // Uniform Catmull-Rom can overshoot between points, so the whole sampled curve is checked, not just the points.
    for (const SceneObject& obj : m_activeScene.objects)
    {
        if (obj.meshRef != "ground")
            continue;
        constexpr float kGroundHalfSize = 12.0f;  // matches MakeGroundPlane in Init
        constexpr float kMargin = 0.3f;
        glm::vec2 center(obj.position.x, obj.position.z);
        glm::vec2 half = glm::vec2(kGroundHalfSize * obj.scale.x, kGroundHalfSize * obj.scale.z) -
                         glm::vec2(kMargin);
        for (const Anim::PathArcLengthSample& s : candidate.GetTables().GetGlobalSamples())
        {
            Anim::Vec3 p = path.Evaluate(s.location.segmentIndex, static_cast<float>(s.location.parameter));
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

    // Keep the same normalized lap progress on the reshaped path; Seek samples gait from absolute distance.
    const Anim::PathFollowerState& old = m_pathFollower.GetState();
    candidate.Seek(old.normalizedDistance, old.lapCount);
    candidate.SetPlaying(old.playing);
    m_pathFollower = std::move(candidate);

    m_activeScene.objects[m_skeletalObjectIndex].pathMotion.controlPointsXZ = pointsXZ;
    m_pathOverlayDirty = true;
    return true;
}

void Renderer::RequestPathCameraFrame()
{
    if (!m_pathMotionActive)
        return;

    const Anim::CubicPath& path = m_pathFollower.GetPath();
    glm::vec3 lo(1e30f), hi(-1e30f);
    for (const Anim::PathArcLengthSample& s : m_pathFollower.GetTables().GetGlobalSamples())
    {
        Anim::Vec3 p = path.Evaluate(s.location.segmentIndex, static_cast<float>(s.location.parameter));
        lo = glm::min(lo, glm::vec3(p.x, p.y, p.z));
        hi = glm::max(hi, glm::vec3(p.x, p.y, p.z));
    }

    // Fit the bounding sphere in the vertical FOV, looking down from the +Z side at ~40 degrees.
    glm::vec3 center = 0.5f * (lo + hi);
    float radius = std::max(0.5f * glm::length(hi - lo), 1.0f);
    float distance = 1.15f * radius / std::sin(glm::radians(60.0f) * 0.5f);
    glm::vec3 dir = glm::normalize(glm::vec3(0.0f, 0.65f, 0.76f));

    m_cameraFrameTarget = center;
    m_cameraFramePosition = center + dir * distance;
    m_cameraFrameRequested = true;
}

bool Renderer::ConsumeCameraFrameRequest(glm::vec3& position, glm::vec3& target)
{
    if (!m_cameraFrameRequested)
        return false;
    m_cameraFrameRequested = false;
    position = m_cameraFramePosition;
    target = m_cameraFrameTarget;
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

    // The binding is the editable source of truth (Path Motion panel writes it); mirror it into the follower when it differs.
    // SetSettings preserves travelled distance and re-anchors gait cycles, so edits never snap position or phase.
    Anim::PathFollowerSettings settings = m_pathFollower.GetSettings();
    Anim::PathFollowerSettings wanted = settings;
    wanted.easeInEnd = binding.easeInEnd;
    wanted.easeOutStart = binding.easeOutStart;
    wanted.lookAheadMeters = binding.lookAheadMeters;
    if (m_pathLocomotionIndex >= 0)
    {
        // Stride and speed scale with the object so a resized character keeps the same cadence.
        const LocomotionCalibration& calib = binding.locomotion[m_pathLocomotionIndex];
        wanted.strideMeters = calib.strideMeters * obj.scale.y;
        wanted.cruiseSpeed = calib.cruiseSpeed * obj.scale.y;
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

// Refreshes the view/proj matrices DrawDebugUI's gizmo and raycast picking read, so they're never stale.
void Renderer::UpdateCameraCache(const Camera& camera)
{
    m_cachedView = camera.GetView();
    m_cachedProj = camera.GetProj();
}

void Renderer::Update(float deltaSeconds)
{
    if (!m_ready)
        return;

    if (m_pathMotionActive)
    {
        // Applies panel edits and Inspector scale changes from the previous UI pass; no-op when nothing changed.
        SyncPathLocomotion();
        // Shared play/pause: the Animation panel's Pause also stops travel.
        if (m_animator.playing)
            m_timelineOwnsPose = false;
        m_pathFollower.SetPlaying(m_animator.playing);
        try
        {
            m_pathFollower.Advance(deltaSeconds);
        }
        catch (const std::exception& e)
        {
            m_pathMotionActive = false;
            m_pathLocomotionIndex = -1;
            m_pathMotionStatus = std::string("Path motion stopped: ") + e.what();
        }
    }

    if (m_pathMotionActive && m_pathPaceCoupled && m_pathLocomotionIndex >= 0)
    {
        // Gait is a function of travelled distance, not wall time, so stopping freezes the feet and easing slows them.
        // A manual Animation-timeline scrub takes the pose until Play, path scrub, or reset hands it back.
        if (!m_timelineOwnsPose)
            Anim::SeekTo(m_animator,
                         static_cast<float>(m_pathFollower.GetState().gaitPhase) *
                             m_animator.clip->duration);
    }
    else
    {
        Anim::Advance(m_animator, deltaSeconds);
    }
}

glm::mat4 Renderer::GetPathPoseMatrix(int objectIndex) const
{
    if (!m_pathMotionActive || objectIndex < 0 || objectIndex != m_skeletalObjectIndex)
        return glm::mat4(1.0f);

    const PathMotionBinding& binding = m_activeScene.objects[objectIndex].pathMotion;
    const Anim::PathFollowerState& state = m_pathFollower.GetState();

    // Follower orientation is yaw-only, mapping local +Z onto the XZ heading.
    float headingYaw = std::atan2(state.forward.x, state.forward.z);
    glm::vec3 position(state.position.x, state.position.y + binding.floorOffset, state.position.z);

    glm::mat4 M = glm::translate(glm::mat4(1.0f), position);
    return glm::rotate(M, headingYaw + glm::radians(binding.facingYawDegrees),
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

    if (m_useDeferred)
        RenderDeferred(camera);
    else
        RenderForward(camera);
}

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
