#include "pch.h"
#include <algorithm>
#include <cmath>
#include <filesystem>
#include <glad/glad.h>
#include <glm/ext/matrix_transform.hpp>
#include <glm/ext/matrix_clip_space.hpp>
#include <glm/gtc/constants.hpp>
#include <glm/gtc/matrix_inverse.hpp>
#include <stb_image.h>

#include "graphics/Renderer.h"
#include "graphics/Geometry.h"

// ---------------------------------------------------------------------------
// IBL probe sphere presets — 8 spheres arranged in a row in front of the
// Cornell box.  First 4: dielectric (F0=0.04), varying roughness.
// Last 4: fixed roughness, varying F0 from plastic → gold → chrome mirror.
// ---------------------------------------------------------------------------
struct IBLProbeMat
{
    glm::vec3 kd;
    glm::vec3 ks;  // F0
    float alpha;   // Phong shininess (higher = smoother)
};

// All 8 spheres sweep alpha (Phong shininess) from fully matte to mirror-smooth.
// Fixed white dielectric (F0 = 0.04) so specular blur is the only variable.
static constexpr IBLProbeMat kIBLProbes[8] = {
    {{0.80f, 0.80f, 0.80f}, {0.04f, 0.04f, 0.04f}, 2.0f},    // 0 — very rough (matte)
    {{0.80f, 0.80f, 0.80f}, {0.04f, 0.04f, 0.04f}, 4.0f},    // 1 — rough
    {{0.80f, 0.80f, 0.80f}, {0.04f, 0.04f, 0.04f}, 8.0f},    // 2 — medium-rough
    {{0.80f, 0.80f, 0.80f}, {0.04f, 0.04f, 0.04f}, 24.0f},   // 3 — medium
    {{0.80f, 0.80f, 0.80f}, {0.04f, 0.04f, 0.04f}, 64.0f},   // 4 — medium-smooth
    {{0.80f, 0.80f, 0.80f}, {0.04f, 0.04f, 0.04f}, 128.0f},  // 5 — smooth
    {{0.80f, 0.80f, 0.80f}, {0.04f, 0.04f, 0.04f}, 200.0f},  // 6 — very smooth
    {{0.80f, 0.80f, 0.80f}, {0.04f, 0.04f, 0.04f}, 256.0f},  // 7 — mirror
};

// All 8 probes sit in a row along X in front of the Cornell box.
static glm::vec3 IBLProbePosition(int i)
{
    return {-1.75f + i * 0.5f, -1.10f, 1.2f};
}

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

    // Light volume mesh
    auto unitSphere = Geometry::MakeSphere(1.0f, 32, 16);
    m_lightVolumeSphereMesh.Create(unitSphere.vertices, unitSphere.indices);

    // Meshes
    auto cube = Geometry::MakeCube(0.5f);
    m_cubeMesh.Create(cube.vertices, cube.indices);

    auto sphere = Geometry::MakeSphere(0.5f, 32, 16);
    m_sphereMesh.Create(sphere.vertices, sphere.indices);

    m_cornell = Geometry::MakeCornellBox({1.0f, 1.0f, 1.0f});
    m_cornellMesh.Create(m_cornell.mesh.vertices, m_cornell.mesh.indices);

    auto ground = Geometry::MakeGroundPlane(12.0f, -1.25f);
    m_groundMesh.Create(ground.vertices, ground.indices);

    // Defaults
    glEnable(GL_DEPTH_TEST);

    // Screen quad + gbuffer
    EnsureScreenQuad();

    if (!m_gbuffer.Create(m_viewportW, m_viewportH))
        return false;

    m_lightCount = 5;

    // Key / main warm light (top)
    m_lights[0] = {.position = {0.0f, 0.90f, 0.0f},  // near ceiling
                   .color = {1.0f, 0.85f, 0.70f},    // warm white
                   .range = 1.8f};
    m_lightIntensity[0] = 0.45f;

    // Cool rim / side
    m_lights[1] = {.position = {-0.65f, 0.70f, 0.0f}, .color = {0.45f, 0.60f, 1.0f}, .range = 1.6f};
    m_lightIntensity[1] = 0.30f;

    // Warm accent
    m_lights[2] = {.position = {0.65f, 0.60f, -0.45f},
                   .color = {1.0f, 0.55f, 0.35f},
                   .range = 1.4f};
    m_lightIntensity[2] = 0.25f;

    // Soft fill (center, low intensity)
    m_lights[3] = {.position = {0.0f, 0.45f, 0.0f}, .color = {0.8f, 0.85f, 0.9f}, .range = 1.2f};
    m_lightIntensity[3] = 0.15f;

    // Back accent / color contrast
    m_lights[4] = {.position = {0.0f, 0.75f, 0.65f}, .color = {0.6f, 1.0f, 0.7f}, .range = 1.3f};
    m_lightIntensity[4] = 0.20f;

    for (int i = 0; i < kMaxLights; ++i)
    {
        m_lightEnabled[i] = true;
        if (m_lightIntensity[i] <= 0.0f)
            m_lightIntensity[i] = 1.0f;
        if (m_lights[i].range <= 0.0f)
            m_lights[i].range = 1.5f;
    }

    m_mat.ambient = 0.02f;
    m_mat.ks = glm::vec3(0.06f);

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
void Renderer::ComputeSHCoefficients(const std::string& path)
{
    stbi_set_flip_vertically_on_load(false);
    int W = 0, H = 0, ch = 0;
    float* data = stbi_loadf(path.c_str(), &W, &H, &ch, 3);
    if (!data)
    {
        std::cerr << "[Renderer] SH: failed to load HDRI at " << path << "\n";
        return;
    }

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

            const float* px = &data[(j * W + i) * 3];
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
    stbi_image_free(data);

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

void Renderer::ScanHDRIFolder()
{
    m_hdriFiles.clear();
    m_hdriSelectedIdx = -1;

    namespace fs = std::filesystem;
    std::error_code ec;
    for (const auto& entry : fs::directory_iterator(kHDRIFolder, ec))
    {
        if (!entry.is_regular_file(ec)) continue;
        std::string ext = entry.path().extension().string();
        std::transform(ext.begin(), ext.end(), ext.begin(), ::tolower);
        if (ext == ".hdr")
            m_hdriFiles.push_back(entry.path().filename().string());
    }
    std::sort(m_hdriFiles.begin(), m_hdriFiles.end());
}

void Renderer::Shutdown()
{
    DestroyScreenQuad();
    DestroyLightGizmoQuad();
    m_gbuffer.Destroy();
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
        m_gbuffer.Resize(m_viewportW, m_viewportH);
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

void Renderer::RenderForward(const Camera& camera)
{
    glViewport(0, 0, m_viewportW, m_viewportH);
    glBindFramebuffer(GL_FRAMEBUFFER, 0);

    glClearColor(0.1f, 0.12f, 0.15f, 1.0f);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    glEnable(GL_DEPTH_TEST);

    m_litShader.Bind();

    m_litShader.SetMat4("uModel", glm::mat4(1.0f));
    m_litShader.SetMat3("uNormalMatrix", glm::mat3(1.0f));
    m_litShader.SetMat4("uView", camera.GetView());
    m_litShader.SetMat4("uProj", camera.GetProj());
    m_litShader.SetVec3("uCamPos", camera.GetPosition());

    m_litShader.SetVec3("uAlbedo", m_mat.kd);
    m_litShader.SetVec3("uKs", m_mat.ks);
    m_litShader.SetFloat("uAmbient", m_mat.ambient);
    m_litShader.SetFloat("uAlpha", m_mat.alpha);

    int count = std::clamp(m_lightCount, 0, kMaxLights);
    m_litShader.SetInt("uLightCount", count);

    glm::vec3 pos[kMaxLights];
    glm::vec3 col[kMaxLights];
    for (int i = 0; i < count; ++i)
    {
        pos[i] = m_lights[i].position;
        col[i] = (m_lightEnabled[i] ? m_lights[i].color * m_lightIntensity[i] : glm::vec3(0.0f));
    }
    m_litShader.SetVec3Array("uLightPos", pos, count);
    m_litShader.SetVec3Array("uLightColor", col, count);

    // Cornell walls
    for (const auto& part : m_cornell.parts)
    {
        m_litShader.SetVec3("uAlbedo", part.albedo);
        m_cornellMesh.DrawRange(part.indexStart, part.indexCount);
    }

    // Ground
    m_litShader.SetVec3("uAlbedo", glm::vec3(0.20f));
    m_groundMesh.Draw();

    constexpr float cornellFloorY = -1.0f;

    auto DrawCube = [&](glm::mat4 model, glm::vec3 albedo)
    {
        m_litShader.SetMat4("uModel", model);
        m_litShader.SetMat3("uNormalMatrix", glm::mat3(glm::transpose(glm::inverse(model))));
        m_litShader.SetVec3("uAlbedo", albedo);
        m_cubeMesh.Draw();
    };

    auto DrawSphere = [&](glm::mat4 model, glm::vec3 albedo)
    {
        m_litShader.SetMat4("uModel", model);
        m_litShader.SetMat3("uNormalMatrix", glm::mat3(glm::transpose(glm::inverse(model))));
        m_litShader.SetVec3("uAlbedo", albedo);
        m_sphereMesh.Draw();
    };

    // Tall cube (left)
    glm::vec3 tallPos(-0.45f, 0.0f, -0.20f);
    glm::vec3 tallScl(0.25f, 0.90f, 0.25f);
    {
        glm::mat4 M(1.0f);
        float centerY = cornellFloorY + 0.5f * tallScl.y;
        M = glm::translate(M, glm::vec3(tallPos.x, centerY, tallPos.z));
        M = glm::scale(M, tallScl);
        DrawCube(M, m_albedoTall);
    }

    // Short cube (right)
    glm::vec3 shortPos(0.0f, 0.0f, 0.20f);
    glm::vec3 shortScl(0.45f, 0.35f, 0.45f);
    {
        glm::mat4 M(1.0f);
        float centerY = cornellFloorY + 0.5f * shortScl.y;
        M = glm::translate(M, glm::vec3(shortPos.x, centerY, shortPos.z));
        M = glm::scale(M, shortScl);
        DrawCube(M, m_albedoShort);
    }

    // Small cube (center)
    glm::vec3 smallScl(0.20f);
    {
        glm::mat4 M(1.0f);
        float topShortY = cornellFloorY + shortScl.y;
        float centerY = topShortY + 0.5f * smallScl.y;
        M = glm::translate(M, glm::vec3(shortPos.x, centerY, shortPos.z));
        M = glm::scale(M, smallScl);
        DrawCube(M, m_albedoSmall);
    }

    // Sphere on the floor
    {
        glm::vec3 sphereScl(0.35f);
        float radius = 0.5f * sphereScl.y;
        glm::mat4 M(1.0f);
        M = glm::translate(M, glm::vec3(0.55f, cornellFloorY + radius, -0.25f));
        M = glm::scale(M, sphereScl);
        DrawSphere(M, m_albedoSphere);
    }

    DrawLightGizmos(camera);

    m_litShader.Unbind();
}

void Renderer::RenderDeferred(const Camera& camera)
{
    if (m_useMSM)
    {
        MSMShadowPass();
        MSMBlurPass();
    }
    else
    {
        ShadowPass();
    }
    GBufferPass(camera);
    FullscreenLightPass(camera);
    LocalLightsPass(camera);

    DrawLightGizmos(camera);
}

// Draw all scene geometry using the supplied shader (uModel must exist in shader).
// Used for both GBuffer and shadow passes.
void Renderer::DrawSceneGeometry(Shader& sh)
{
    constexpr float cornellFloorY = -1.0f;

    // Cornell walls (identity model)
    sh.SetMat4("uModel", glm::mat4(1.0f));
    m_cornellMesh.Draw();

    // Ground (identity model)
    m_groundMesh.Draw();

    // Tall cube
    glm::vec3 tallPos(-0.45f, 0.0f, -0.20f);
    glm::vec3 tallScl(0.25f, 0.90f, 0.25f);
    {
        glm::mat4 M(1.0f);
        float centerY = cornellFloorY + 0.5f * tallScl.y;
        M = glm::translate(M, glm::vec3(tallPos.x, centerY, tallPos.z));
        M = glm::scale(M, tallScl);
        sh.SetMat4("uModel", M);
        m_cubeMesh.Draw();
    }

    // Short cube
    glm::vec3 shortPos(0.0f, 0.0f, 0.20f);
    glm::vec3 shortScl(0.45f, 0.35f, 0.45f);
    {
        glm::mat4 M(1.0f);
        float centerY = cornellFloorY + 0.5f * shortScl.y;
        M = glm::translate(M, glm::vec3(shortPos.x, centerY, shortPos.z));
        M = glm::scale(M, shortScl);
        sh.SetMat4("uModel", M);
        m_cubeMesh.Draw();
    }

    // Small cube (stacked on short)
    glm::vec3 smallScl(0.20f);
    {
        glm::mat4 M(1.0f);
        float topShortY = cornellFloorY + shortScl.y;
        float centerY = topShortY + 0.5f * smallScl.y;
        M = glm::translate(M, glm::vec3(shortPos.x, centerY, shortPos.z));
        M = glm::scale(M, smallScl);
        sh.SetMat4("uModel", M);
        m_cubeMesh.Draw();
    }

    // Sphere
    {
        glm::vec3 sphereScl(0.35f);
        float radius = 0.5f * sphereScl.y;
        glm::mat4 M(1.0f);
        M = glm::translate(M, glm::vec3(0.55f, cornellFloorY + radius, -0.25f));
        M = glm::scale(M, sphereScl);
        sh.SetMat4("uModel", M);
        m_sphereMesh.Draw();
    }

    // IBL probe spheres
    if (m_showIBLProbes)
    {
        for (int i = 0; i < 8; ++i)
        {
            glm::mat4 M(1.0f);
            M = glm::translate(M, IBLProbePosition(i));
            M = glm::scale(M, glm::vec3(0.3f));
            sh.SetMat4("uModel", M);
            m_sphereMesh.Draw();
        }
    }
}

void Renderer::ShadowPass()
{
    if (!m_shadowsEnabled)
        return;

    // Six cube face view directions (target offsets and up vectors)
    static const glm::vec3 targets[6] = {
        {1, 0, 0},
        {-1, 0, 0},
        {0, 1, 0},
        {0, -1, 0},
        {0, 0, 1},
        {0, 0, -1},
    };
    static const glm::vec3 ups[6] = {
        {0, -1, 0},
        {0, -1, 0},
        {0, 0, 1},
        {0, 0, -1},
        {0, -1, 0},
        {0, -1, 0},
    };

    glEnable(GL_DEPTH_TEST);
    glDisable(GL_CULL_FACE);
    glDisable(GL_BLEND);

    m_shadowShader.Bind();

    int count = std::clamp(m_lightCount, 0, kMaxLights);
    for (int i = 0; i < count; ++i)
    {
        if (!m_lightEnabled[i])
            continue;

        const glm::vec3 lightPos = m_lights[i].position;
        const float farPlane = m_lights[i].range * 1.5f;
        const glm::mat4 proj = glm::perspective(glm::radians(90.0f), 1.0f, 0.01f, farPlane);

        m_shadowShader.SetVec3("uLightPos", lightPos);
        m_shadowShader.SetFloat("uFarPlane", farPlane);

        for (int face = 0; face < 6; ++face)
        {
            m_shadowMaps[i].BindForFace(face);
            glViewport(0, 0, m_shadowMaps[i].Resolution(), m_shadowMaps[i].Resolution());
            glClear(GL_DEPTH_BUFFER_BIT);

            glm::mat4 view = glm::lookAt(lightPos, lightPos + targets[face], ups[face]);
            m_shadowShader.SetMat4("uLightVP", proj * view);

            DrawSceneGeometry(m_shadowShader);
        }

        ShadowMap::Unbind();
    }

    m_shadowShader.Unbind();

    // Restore viewport for subsequent passes
    glViewport(0, 0, m_viewportW, m_viewportH);
}

void Renderer::MSMShadowPass()
{
    if (!m_shadowsEnabled)
        return;

    static const glm::vec3 targets[6] = {
        {1, 0, 0},
        {-1, 0, 0},
        {0, 1, 0},
        {0, -1, 0},
        {0, 0, 1},
        {0, 0, -1},
    };
    static const glm::vec3 ups[6] = {
        {0, -1, 0},
        {0, -1, 0},
        {0, 0, 1},
        {0, 0, -1},
        {0, -1, 0},
        {0, -1, 0},
    };

    glEnable(GL_DEPTH_TEST);
    glDisable(GL_CULL_FACE);
    glDisable(GL_BLEND);

    m_msmMomentShader.Bind();

    int count = std::clamp(m_lightCount, 0, kMaxLights);
    for (int i = 0; i < count; ++i)
    {
        if (!m_lightEnabled[i])
            continue;

        const glm::vec3 lightPos = m_lights[i].position;
        const float farPlane = m_lights[i].range * 1.5f;
        const glm::mat4 proj = glm::perspective(glm::radians(90.0f), 1.0f, 0.01f, farPlane);

        m_msmMomentShader.SetVec3("uLightPos", lightPos);
        m_msmMomentShader.SetFloat("uFarPlane", farPlane);

        for (int face = 0; face < 6; ++face)
        {
            m_msmMaps[i].BindForCapture(face);
            glViewport(0, 0, m_msmMaps[i].Resolution(), m_msmMaps[i].Resolution());
            glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

            glm::mat4 view = glm::lookAt(lightPos, lightPos + targets[face], ups[face]);
            m_msmMomentShader.SetMat4("uLightVP", proj * view);

            DrawSceneGeometry(m_msmMomentShader);
        }

        MomentShadowMap::Unbind();
    }

    m_msmMomentShader.Unbind();
    glViewport(0, 0, m_viewportW, m_viewportH);
}

void Renderer::MSMBlurPass()
{
    if (!m_shadowsEnabled)
        return;

    int count = std::clamp(m_lightCount, 0, kMaxLights);
    for (int i = 0; i < count; ++i)
    {
        if (!m_lightEnabled[i])
            continue;

        for (int face = 0; face < 6; ++face)
            m_msmMaps[i].Blur(face, m_msmBlurHShader, m_msmBlurVShader, m_msmBlurStep);
    }

    glMemoryBarrier(GL_TEXTURE_FETCH_BARRIER_BIT);
    glUseProgram(0);
}

void Renderer::GBufferPass(const Camera& camera)
{
    m_gbuffer.BindForWriting();

    glViewport(0, 0, m_viewportW, m_viewportH);
    glEnable(GL_DEPTH_TEST);
    glDisable(GL_BLEND);
    glDisable(GL_CULL_FACE);

    glClear(GL_DEPTH_BUFFER_BIT);

    // Clear each colour attachment individually so attachment 0 (world position)
    // gets w=0.  The shader uses wp.w < 0.5 to detect background pixels.
    // glClear(GL_COLOR_BUFFER_BIT) with glClearColor would set w=1 on every
    // attachment, making background indistinguishable from geometry.
    const float kClearZero[4] = {0.f, 0.f, 0.f, 0.f};
    glClearBufferfv(GL_COLOR, 0, kClearZero);  // world pos  — w=0 → background
    glClearBufferfv(GL_COLOR, 1, kClearZero);  // normal
    glClearBufferfv(GL_COLOR, 2, kClearZero);  // Kd
    glClearBufferfv(GL_COLOR, 3, kClearZero);  // Ks + alpha

    m_gbufferShader.Bind();
    m_gbufferShader.SetMat4("uView", camera.GetView());
    m_gbufferShader.SetMat4("uProj", camera.GetProj());

    // helper lambdas
    auto SetMaterial = [&](const glm::vec3& kd)
    {
        m_gbufferShader.SetVec3("uKd", kd);
        m_gbufferShader.SetVec3("uKs", m_mat.ks);
        m_gbufferShader.SetFloat("uAlpha", m_mat.alpha);
    };

    // Cornell walls and ground both use identity model — normal matrix = identity
    m_gbufferShader.SetMat4("uModel", glm::mat4(1.0f));
    m_gbufferShader.SetMat3("uNormalMatrix", glm::mat3(1.0f));
    for (const auto& part : m_cornell.parts)
    {
        SetMaterial(part.albedo);
        m_cornellMesh.DrawRange(part.indexStart, part.indexCount);
    }

    // Ground — moderately polished surface so IBL specular is visible
    SetMaterial(glm::vec3(0.18f));
    m_gbufferShader.SetVec3("uKs", glm::vec3(0.25f));
    m_gbufferShader.SetFloat("uAlpha", 180.0f);
    m_groundMesh.Draw();

    constexpr float cornellFloorY = -1.0f;

    auto DrawCube = [&](const glm::mat4& model, glm::vec3 kd)
    {
        m_gbufferShader.SetMat4("uModel", model);
        m_gbufferShader.SetMat3("uNormalMatrix", glm::mat3(glm::transpose(glm::inverse(model))));
        SetMaterial(kd);
        m_cubeMesh.Draw();
    };

    auto DrawSphere = [&](const glm::mat4& model, glm::vec3 kd)
    {
        m_gbufferShader.SetMat4("uModel", model);
        m_gbufferShader.SetMat3("uNormalMatrix", glm::mat3(glm::transpose(glm::inverse(model))));
        SetMaterial(kd);
        m_sphereMesh.Draw();
    };

    // Tall cube
    glm::vec3 tallPos(-0.45f, 0.0f, -0.20f);
    glm::vec3 tallScl(0.25f, 0.90f, 0.25f);
    {
        glm::mat4 M(1.0f);
        float centerY = cornellFloorY + 0.5f * tallScl.y;
        M = glm::translate(M, glm::vec3(tallPos.x, centerY, tallPos.z));
        M = glm::scale(M, tallScl);
        DrawCube(M, m_albedoTall);
    }

    // Short cube
    glm::vec3 shortPos(0.0f, 0.0f, 0.20f);
    glm::vec3 shortScl(0.45f, 0.35f, 0.45f);
    {
        glm::mat4 M(1.0f);
        float centerY = cornellFloorY + 0.5f * shortScl.y;
        M = glm::translate(M, glm::vec3(shortPos.x, centerY, shortPos.z));
        M = glm::scale(M, shortScl);
        DrawCube(M, m_albedoShort);
    }

    // Small cube
    glm::vec3 smallScl(0.20f);
    {
        glm::mat4 M(1.0f);
        float topShortY = cornellFloorY + shortScl.y;
        float centerY = topShortY + 0.5f * smallScl.y;
        M = glm::translate(M, glm::vec3(shortPos.x, centerY, shortPos.z));
        M = glm::scale(M, smallScl);
        DrawCube(M, m_albedoSmall);
    }

    // Sphere
    {
        glm::vec3 sphereScl(0.35f);
        float radius = 0.5f * sphereScl.y;
        glm::mat4 M(1.0f);
        M = glm::translate(M, glm::vec3(0.55f, cornellFloorY + radius, -0.25f));
        M = glm::scale(M, sphereScl);
        DrawSphere(M, m_albedoSphere);
    }

    // IBL probe spheres — each has unique ks/F0 and roughness
    if (m_showIBLProbes)
    {
        // Probe spheres use uniform scale — normal matrix is the upper-left 3x3 of the model
        for (int i = 0; i < 8; ++i)
        {
            const IBLProbeMat& p = kIBLProbes[i];
            glm::mat4 M(1.0f);
            M = glm::translate(M, IBLProbePosition(i));
            M = glm::scale(M, glm::vec3(0.3f));
            m_gbufferShader.SetMat4("uModel", M);
            m_gbufferShader.SetMat3("uNormalMatrix", glm::mat3(M));
            m_gbufferShader.SetVec3("uKd", glm::vec3(m_probeKd));
            m_gbufferShader.SetVec3("uKs", glm::vec3(m_probeF0));
            m_gbufferShader.SetFloat("uAlpha", p.alpha);
            m_sphereMesh.Draw();
        }
    }

    m_gbufferShader.Unbind();
    GBuffer::UnbindWriting();
}

static void BindGBufferTextures(const GBuffer& gb, const Shader& sh)
{
    // Texture unit mapping
    glActiveTexture(GL_TEXTURE0);
    glBindTexture(GL_TEXTURE_2D, gb.TexWorldPos());
    sh.SetInt("uWorldPosTex", 0);

    glActiveTexture(GL_TEXTURE1);
    glBindTexture(GL_TEXTURE_2D, gb.TexNormal());
    sh.SetInt("uNormalTex", 1);

    glActiveTexture(GL_TEXTURE2);
    glBindTexture(GL_TEXTURE_2D, gb.TexKd());
    sh.SetInt("uKdTex", 2);

    glActiveTexture(GL_TEXTURE3);
    glBindTexture(GL_TEXTURE_2D, gb.TexKsAlpha());
    sh.SetInt("uKsAlphaTex", 3);
}

void Renderer::FullscreenLightPass(const Camera& camera)
{
    glBindFramebuffer(GL_FRAMEBUFFER, 0);
    glViewport(0, 0, m_viewportW, m_viewportH);

    glDisable(GL_DEPTH_TEST);
    glDisable(GL_BLEND);
    glDisable(GL_CULL_FACE);

    glClearColor(0.03f, 0.03f, 0.03f, 1.0f);
    glClear(GL_COLOR_BUFFER_BIT);

    // Select shader: IBL when both maps are loaded, otherwise PBS
    const bool useIBL = (m_lightingMode == LightingMode::IBL) && (m_hdriTex.ID() != 0) &&
                        (m_irradianceTex.ID() != 0);
    Shader& sh = useIBL ? m_iblShader : m_fullscreenShader;

    sh.Bind();
    BindGBufferTextures(m_gbuffer, sh);

    // Shadow cubemaps — texture units 4..8
    static const char* kShadowMapNames[5] = {"uShadowMaps[0]",
                                             "uShadowMaps[1]",
                                             "uShadowMaps[2]",
                                             "uShadowMaps[3]",
                                             "uShadowMaps[4]"};
    static const char* kMSMapNames[5] = {"uMSMaps[0]",
                                         "uMSMaps[1]",
                                         "uMSMaps[2]",
                                         "uMSMaps[3]",
                                         "uMSMaps[4]"};

    float farPlanes[kMaxLights];
    for (int i = 0; i < kMaxLights; ++i)
    {
        glActiveTexture(GL_TEXTURE4 + i);
        glBindTexture(GL_TEXTURE_CUBE_MAP, m_shadowMaps[i].TexCube());
        sh.SetInt(kShadowMapNames[i], 4 + i);
        farPlanes[i] = m_lights[i].range * 1.5f;
    }
    sh.SetInt("uShadowsEnabled", m_shadowsEnabled ? 1 : 0);
    sh.SetFloat("uShadowBias", m_shadowBias);
    sh.SetFloat("uShadowPcfRadius", m_shadowPcfRadius);
    sh.SetFloatArray("uShadowFarPlane", farPlanes, kMaxLights);

    // MSM cubemaps — texture units 9..13
    for (int i = 0; i < kMaxLights; ++i)
    {
        glActiveTexture(GL_TEXTURE9 + i);
        glBindTexture(GL_TEXTURE_CUBE_MAP, m_msmMaps[i].TexBlurred());
        sh.SetInt(kMSMapNames[i], 9 + i);
    }
    sh.SetInt("uUseMSM", m_useMSM ? 1 : 0);
    sh.SetFloat("uMSMAlpha", m_msmAlpha);
    sh.SetFloatArray("uMSMFarPlane", farPlanes, kMaxLights);

    sh.SetInt("uDebugView", static_cast<int>(m_debugView));
    sh.SetVec3("uCamPos", camera.GetPosition());
    sh.SetFloat("uExposure", m_exposure);

    // Lights
    int count = std::clamp(m_lightCount, 0, kMaxLights);
    sh.SetInt("uLightCount", count);
    glm::vec3 pos[kMaxLights];
    glm::vec3 col[kMaxLights];
    float rng[kMaxLights];
    for (int i = 0; i < count; ++i)
    {
        pos[i] = m_lights[i].position;
        col[i] = m_lightEnabled[i] ? m_lights[i].color * m_lightIntensity[i] : glm::vec3(0.0f);
        rng[i] = m_lights[i].range;
    }
    sh.SetVec3Array("uLightPos", pos, count);
    sh.SetVec3Array("uLightColor", col, count);
    sh.SetFloatArray("uLightRange", rng, count);
    sh.SetInt("uDebugLightIndex", m_debugLightIndex);
    sh.SetFloat("uGlobeRadius", m_globeRadius);

    if (useIBL)
    {
        // HDRI environment map — texture unit 14
        glActiveTexture(GL_TEXTURE14);
        glBindTexture(GL_TEXTURE_2D, m_hdriTex.ID());
        sh.SetInt("uHDRITex", 14);

        // Irradiance map — texture unit 15
        glActiveTexture(GL_TEXTURE15);
        glBindTexture(GL_TEXTURE_2D, m_irradianceTex.ID());
        sh.SetInt("uIrradianceTex", 15);

        sh.SetInt("uHDRIWidth", m_hdriTex.Width());
        sh.SetInt("uHDRIHeight", m_hdriTex.Height());
        sh.SetInt("uIBLSamples", m_iblSamples);
        sh.SetVec2Array("uHammersley", m_hammersley, m_iblSamples);
        sh.SetFloat("uHDRIRotation", m_hdriRotation);
        sh.SetInt("uUseSHIrradiance", m_useSHIrradiance ? 1 : 0);

        // SH coefficients UBO — keep bound at binding point 2
        if (m_shCoeffsUBO != 0)
            glBindBufferBase(GL_UNIFORM_BUFFER, 2, m_shCoeffsUBO);

        // Inverse view-projection for skydome ray reconstruction
        glm::mat4 invVP = glm::inverse(camera.GetProj() * camera.GetView());
        sh.SetMat4("uInvViewProj", invVP);
    }
    else
    {
        sh.SetFloat("uAmbient", m_mat.ambient);
    }

    glBindVertexArray(m_quadVAO);
    glDrawArrays(GL_TRIANGLES, 0, 6);
    glBindVertexArray(0);

    sh.Unbind();
}

void Renderer::LocalLightsPass(const Camera& camera)
{
    // Skip in IBL mode — the IBL fullscreen pass handles all direct lights.
    if (m_lightingMode == LightingMode::IBL && m_hdriTex.ID() != 0 && m_irradianceTex.ID() != 0)
        return;

    // If we are in debug view mode, skip local lights so we can see raw gbuffer.
    if (m_debugView != DebugView::Final)
        return;

    glBindFramebuffer(GL_FRAMEBUFFER, 0);
    glViewport(0, 0, m_viewportW, m_viewportH);

    glDisable(GL_DEPTH_TEST);
    glEnable(GL_BLEND);
    glBlendFunc(GL_ONE, GL_ONE);
    glEnable(GL_CULL_FACE);
    glCullFace(GL_FRONT);

    m_localLightShader.Bind();
    BindGBufferTextures(m_gbuffer, m_localLightShader);

    m_localLightShader.SetVec3("uCamPos", camera.GetPosition());
    m_localLightShader.SetMat4("uView", camera.GetView());
    m_localLightShader.SetMat4("uProj", camera.GetProj());

    m_localLightShader.SetVec2("uInvResolution", glm::vec2(1.0f / m_viewportW, 1.0f / m_viewportH));

    int count = std::clamp(m_lightCount, 0, kMaxLights);
    for (int i = 0; i < count; ++i)
    {
        if (!m_lightEnabled[i] || m_lightIntensity[i] <= 0.0f)
            continue;

        glm::vec3 lightCol = m_lights[i].color * m_lightIntensity[i];

        glActiveTexture(GL_TEXTURE4);
        glBindTexture(GL_TEXTURE_CUBE_MAP, m_shadowMaps[i].TexCube());
        m_localLightShader.SetInt("uShadowMap", 4);
        glActiveTexture(GL_TEXTURE5);
        glBindTexture(GL_TEXTURE_CUBE_MAP, m_msmMaps[i].TexBlurred());
        m_localLightShader.SetInt("uMSMMap", 5);
        m_localLightShader.SetInt("uShadowsActive", m_shadowsEnabled ? 1 : 0);
        m_localLightShader.SetFloat("uShadowFarPlane", m_lights[i].range * 1.5f);
        m_localLightShader.SetFloat("uShadowBias", m_shadowBias);
        m_localLightShader.SetFloat("uShadowPcfRadius", m_shadowPcfRadius);
        m_localLightShader.SetInt("uUseMSM", m_useMSM ? 1 : 0);
        m_localLightShader.SetFloat("uMSMAlpha", m_msmAlpha);

        m_localLightShader.SetVec3("uLightPos", m_lights[i].position);
        m_localLightShader.SetVec3("uLightColor", lightCol);
        m_localLightShader.SetFloat("uLightRange", m_lights[i].range);

        glm::mat4 M(1.0f);
        M = glm::translate(M, m_lights[i].position);
        M = glm::scale(M, glm::vec3(m_lights[i].range));
        m_localLightShader.SetMat4("uModel", M);

        m_lightVolumeSphereMesh.Draw();
    }

    m_localLightShader.Unbind();

    glCullFace(GL_BACK);
    glDisable(GL_CULL_FACE);
    glDisable(GL_BLEND);
}

void Renderer::EnsureScreenQuad()
{
    if (m_quadVAO != 0)
        return;

    // Two triangles fullscreen quad
    // aPos (NDC), aUV
    constexpr float verts[] = {
        //  x,  y,   u,  v
        -1.f, -1.f, 0.f, 0.f, 1.f, -1.f, 1.f, 0.f, 1.f,  1.f, 1.f, 1.f,

        -1.f, -1.f, 0.f, 0.f, 1.f, 1.f,  1.f, 1.f, -1.f, 1.f, 0.f, 1.f,
    };

    glGenVertexArrays(1, &m_quadVAO);
    glGenBuffers(1, &m_quadVBO);

    glBindVertexArray(m_quadVAO);
    glBindBuffer(GL_ARRAY_BUFFER, m_quadVBO);
    glBufferData(GL_ARRAY_BUFFER, sizeof(verts), verts, GL_STATIC_DRAW);

    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE, 4 * sizeof(float), static_cast<void*>(0));

    glEnableVertexAttribArray(1);
    glVertexAttribPointer(1,
                          2,
                          GL_FLOAT,
                          GL_FALSE,
                          4 * sizeof(float),
                          reinterpret_cast<void*>(2 * sizeof(float)));

    glBindBuffer(GL_ARRAY_BUFFER, 0);
    glBindVertexArray(0);
}

void Renderer::DestroyScreenQuad()
{
    if (m_quadVBO)
    {
        glDeleteBuffers(1, &m_quadVBO);
        m_quadVBO = 0;
    }
    if (m_quadVAO)
    {
        glDeleteVertexArrays(1, &m_quadVAO);
        m_quadVAO = 0;
    }
}

void Renderer::EnsureLightGizmoQuad()
{
    if (m_lightGizmoVAO != 0)
        return;

    // Centered quad in local space, billboard shader uses aPos.x/y and uSize
    const float quad[] = {
        //  x,    y,   z,   u,  v
        -0.5f, -0.5f, 0.f, 0.f, 0.f, 0.5f, -0.5f, 0.f, 1.f, 0.f, 0.5f,  0.5f, 0.f, 1.f, 1.f,

        -0.5f, -0.5f, 0.f, 0.f, 0.f, 0.5f, 0.5f,  0.f, 1.f, 1.f, -0.5f, 0.5f, 0.f, 0.f, 1.f,
    };

    glGenVertexArrays(1, &m_lightGizmoVAO);
    glGenBuffers(1, &m_lightGizmoVBO);

    glBindVertexArray(m_lightGizmoVAO);
    glBindBuffer(GL_ARRAY_BUFFER, m_lightGizmoVBO);
    glBufferData(GL_ARRAY_BUFFER, sizeof(quad), quad, GL_STATIC_DRAW);

    // layout(location=0) vec3 aPos
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 5 * sizeof(float), static_cast<void*>(0));

    // layout(location=1) vec2 aUV
    glEnableVertexAttribArray(1);
    glVertexAttribPointer(1,
                          2,
                          GL_FLOAT,
                          GL_FALSE,
                          5 * sizeof(float),
                          reinterpret_cast<void*>(3 * sizeof(float)));

    glBindBuffer(GL_ARRAY_BUFFER, 0);
    glBindVertexArray(0);
}

void Renderer::DestroyLightGizmoQuad()
{
    if (m_lightGizmoVBO)
    {
        glDeleteBuffers(1, &m_lightGizmoVBO);
        m_lightGizmoVBO = 0;
    }
    if (m_lightGizmoVAO)
    {
        glDeleteVertexArrays(1, &m_lightGizmoVAO);
        m_lightGizmoVAO = 0;
    }
}

void Renderer::DrawLightGizmos(const Camera& camera) const
{
    if (!m_showLightGizmos)
        return;

    // This VAO must exist
    if (m_lightGizmoVAO == 0)
        return;

    glDisable(GL_DEPTH_TEST);  // icons are debug-only
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    glDisable(GL_CULL_FACE);

    m_lightGizmoShader.Bind();

    m_lightGizmoShader.SetMat4("uView", camera.GetView());
    m_lightGizmoShader.SetMat4("uProj", camera.GetProj());
    m_lightGizmoShader.SetFloat("uSize", 0.2f);  // world-space size

    m_lightGizmoTex.Bind(0);
    m_lightGizmoShader.SetInt("uIcon", 0);

    glBindVertexArray(m_lightGizmoVAO);

    int count = std::clamp(m_lightCount, 0, kMaxLights);
    for (int i = 0; i < count; ++i)
    {
        if (!m_lightEnabled[i])
            continue;

        m_lightGizmoShader.SetVec3("uWorldPos", m_lights[i].position);
        m_lightGizmoShader.SetVec3("uColor", m_lights[i].color * m_lightIntensity[i]);

        glDrawArrays(GL_TRIANGLES, 0, 6);
    }

    glBindVertexArray(0);
    m_lightGizmoShader.Unbind();

    glDisable(GL_BLEND);
}

// ------------------------------------------------------------
// ImGui
// ------------------------------------------------------------
void Renderer::DrawDebugUI()
{
    if (!m_ready)
        return;

    ImGui::Begin("Renderer");

    ImGui::Checkbox("Use Deferred", &m_useDeferred);

    if (m_useDeferred)
    {
        const char* items[] = {"Final",
                               "WorldPos",
                               "Normal",
                               "Kd",
                               "Ks+Alpha",
                               "EyeVec",
                               "LightGlobes",
                               "Brightness",
                               "MSM Depth",
                               "Irradiance Map",
                               "Diffuse IBL only",
                               "Specular IBL only"};

        int mode = static_cast<int>(m_debugView);
        if (ImGui::Combo("Deferred View", &mode, items, IM_ARRAYSIZE(items)))
            m_debugView = static_cast<DebugView>(mode);

        if (m_debugView == DebugView::Brightness || m_debugView == DebugView::MSMDepth)
        {
            ImGui::SliderInt("Debug Light Index",
                             &m_debugLightIndex,
                             0,
                             std::max(0, m_lightCount - 1));
        }
        if (m_debugView == DebugView::LightGlobes)
        {
            ImGui::DragFloat("Globe Radius", &m_globeRadius, 0.005f, 0.005f, 0.25f);
        }
    }

    ImGui::Separator();

    ImGui::Text("Lights");
    ImGui::Checkbox("Show Light Gizmos", &m_showLightGizmos);
    ImGui::SliderInt("Light Count", &m_lightCount, 1, kMaxLights);

    for (int i = 0; i < m_lightCount; ++i)
    {
        ImGui::PushID(i);

        ImGui::Text("Light %d", i);

        ImGui::Checkbox("Enabled", &m_lightEnabled[i]);
        ImGui::SameLine();
        ImGui::DragFloat("Intensity", &m_lightIntensity[i], 0.05f, 0.0f, 50.0f);

        ImGui::DragFloat3("Pos", &m_lights[i].position.x, 0.05f);
        ImGui::ColorEdit3("Color", &m_lights[i].color.x);
        ImGui::DragFloat("Range", &m_lights[i].range, 0.05f, 0.1f, 20.0f);

        ImGui::Separator();
        ImGui::PopID();
    }

    ImGui::Separator();
    ImGui::Text("Shadows");
    ImGui::Checkbox("Enable Shadows", &m_shadowsEnabled);
    if (m_shadowsEnabled)
    {
        ImGui::Checkbox("Use MSM", &m_useMSM);
        if (m_useMSM)
        {
            ImGui::SliderFloat("MSM Blur Step", &m_msmBlurStep, 0.5f, 10.0f);
            ImGui::DragFloat("MSM Alpha", &m_msmAlpha, 1e-5f, 1e-5f, 1e-1f, "%.5f");
        }
        else
        {
            ImGui::DragFloat("Shadow Bias", &m_shadowBias, 0.001f, 0.0f, 0.2f);
            ImGui::DragFloat("PCF Disk Radius", &m_shadowPcfRadius, 0.005f, 0.0f, 0.3f);
        }
    }

    ImGui::Separator();

    ImGui::Text("Objects");
    ImGui::Checkbox("Show IBL Probe Spheres", &m_showIBLProbes);
    if (m_showIBLProbes)
    {
        ImGui::SliderFloat("Probe F0 (Ks)", &m_probeF0, 0.0f, 1.0f, "%.2f");
        if (ImGui::IsItemHovered())
            ImGui::SetTooltip("Specular reflectance at normal incidence for all 8 probe spheres.\n"
                              "Dielectric: ~0.04   |   Iron: ~0.56   |   Chrome: ~0.95");
        ImGui::SliderFloat("Probe Reflectance (Kd)", &m_probeKd, 0.0f, 1.0f, "%.2f");
        if (ImGui::IsItemHovered())
            ImGui::SetTooltip("Diffuse reflectance intensity for all 8 probe spheres.\n"
                              "0.0 = fully absorbing   |   0.5 = mid-grey   |   1.0 = fully reflecting");
    }
    ImGui::ColorEdit3("Tall Cube Albedo", &m_albedoTall.x);
    ImGui::ColorEdit3("Short Cube Albedo", &m_albedoShort.x);
    ImGui::ColorEdit3("Small Cube Albedo", &m_albedoSmall.x);
    ImGui::ColorEdit3("Sphere Albedo", &m_albedoSphere.x);

    ImGui::Separator();
    ImGui::Text("Tone Mapping");
    ImGui::DragFloat("Exposure", &m_exposure, 0.05f, 0.001f, 10000.0f, "%.3f");

    ImGui::Separator();
    ImGui::Text("Lighting Mode");
    const bool iblReady = m_hdriTex.ID() != 0;
    ImGui::TextColored(iblReady ? ImVec4(0.4f, 1.0f, 0.4f, 1.0f) : ImVec4(1.0f, 0.7f, 0.3f, 1.0f),
                       iblReady ? "IBL Active" : "PBS Active (no HDRI loaded)");

    // HDRI dropdown
    {
        // Build display list
        std::vector<const char*> items;
        items.reserve(m_hdriFiles.size());
        for (const auto& f : m_hdriFiles) items.push_back(f.c_str());

        if (items.empty())
        {
            ImGui::TextDisabled("No .hdr files found in %s", kHDRIFolder);
        }
        else
        {
            ImGui::SetNextItemWidth(-1.0f);
            ImGui::Combo("##hdri_pick", &m_hdriSelectedIdx,
                         items.data(), static_cast<int>(items.size()));
        }
    }

    const bool canLoad = m_hdriSelectedIdx >= 0 &&
                         m_hdriSelectedIdx < static_cast<int>(m_hdriFiles.size());

    if (!canLoad) ImGui::BeginDisabled();
    if (ImGui::Button("Load HDRI"))
    {
        std::string path = std::string(kHDRIFolder) + m_hdriFiles[m_hdriSelectedIdx];
        if (m_hdriTex.LoadHDR(path))
        {
            BakeIrradiance();
            ComputeSHCoefficients(path);
            m_lightingMode = LightingMode::IBL;
        }
        else
            std::cerr << "[Renderer] Failed to load HDRI: " << path << "\n";
    }
    if (!canLoad) ImGui::EndDisabled();
    ImGui::SameLine();
    if (ImGui::Button("Refresh"))
        ScanHDRIFolder();
    ImGui::SameLine();
    if (ImGui::Button("Clear HDRI"))
    {
        m_hdriTex.Destroy();
        m_irradianceTex.Destroy();
        m_lightingMode = LightingMode::PBS;
    }

    if (iblReady)
    {
        ImGui::SliderAngle("HDRI Rotation", &m_hdriRotation, 0.0f, 360.0f);

        int prevN = m_iblSamples;
        if (ImGui::SliderInt("IBL Samples", &m_iblSamples, 1, kMaxIBLSamples))
            if (m_iblSamples != prevN)
                BuildHammersley(m_iblSamples);

        ImGui::Checkbox("Use SH Irradiance (Part B)", &m_useSHIrradiance);
        if (ImGui::IsItemHovered())
            ImGui::SetTooltip(
                "Replace irradiance texture lookup with 9-coefficient\n"
                "spherical harmonics reconstruction (Ramamoorthi 2001).\n"
                "Both should look similar; SH is faster but lower frequency.");
    }

    ImGui::Separator();
    ImGui::Text("Material");
    if (!iblReady)
        ImGui::DragFloat("Ambient", &m_mat.ambient, 0.001f, 0.0f, 1.0f);
    ImGui::DragFloat("Roughness (alpha)", &m_mat.alpha, 1.0f, 1.0f, 256.0f);
    if (ImGui::IsItemHovered())
        ImGui::SetTooltip("1 = rough, 256 = mirror-smooth");
    ImGui::ColorEdit3("F0 / Ks", &m_mat.ks.x);
    if (ImGui::IsItemHovered())
        ImGui::SetTooltip(
            "Specular reflectance at normal incidence (F0).\nNon-metals: ~0.04  |  Metals: albedo "
            "colour");

    ImGui::End();
}
