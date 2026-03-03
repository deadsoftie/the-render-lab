#include "pch.h"
#include <algorithm>
#include <glad/glad.h>
#include <glm/ext/matrix_transform.hpp>
#include <glm/ext/matrix_clip_space.hpp>

#include "graphics/Renderer.h"
#include "graphics/Geometry.h"

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
    if (!m_lightGizmoShader.LoadFromFiles("assets/shaders/gizmos/light_gizmo.vert",
                                          "assets/shaders/gizmos/light_gizmo.frag"))
    {
        return false;
    }

    if (!m_shadowShader.LoadFromFiles("assets/shaders/shadow_depth.vert",
                                      "assets/shaders/shadow_depth.frag"))
        return false;

    for (auto& sm : m_shadowMaps)
        if (!sm.Create(512)) return false;

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

    m_ready = true;
    return true;
}

void Renderer::Shutdown()
{
    DestroyScreenQuad();
    DestroyLightGizmoQuad();
    m_gbuffer.Destroy();
    for (auto& sm : m_shadowMaps)
        sm.Destroy();
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
    m_litShader.SetMat4("uView", camera.GetView());
    m_litShader.SetMat4("uProj", camera.GetProj());
    m_litShader.SetVec3("uCamPos", camera.GetPosition());

    m_litShader.SetVec3("uAlbedo", m_mat.kd);
    m_litShader.SetFloat("uAmbient", m_mat.ambient);
    m_litShader.SetFloat("uShininess", m_mat.shininess);

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
        m_litShader.SetVec3("uAlbedo", albedo);
        m_cubeMesh.Draw();
    };

    auto DrawSphere = [&](glm::mat4 model, glm::vec3 albedo)
    {
        m_litShader.SetMat4("uModel", model);
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
    ShadowPass();
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
        float centerY   = topShortY + 0.5f * smallScl.y;
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
}

void Renderer::ShadowPass()
{
    if (!m_shadowsEnabled)
        return;

    // Six cube face view directions (target offsets and up vectors)
    static const glm::vec3 targets[6] = {
        { 1, 0, 0}, {-1, 0, 0},
        { 0, 1, 0}, { 0,-1, 0},
        { 0, 0, 1}, { 0, 0,-1},
    };
    static const glm::vec3 ups[6] = {
        { 0,-1, 0}, { 0,-1, 0},
        { 0, 0, 1}, { 0, 0,-1},
        { 0,-1, 0}, { 0,-1, 0},
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
        const float     farPlane = m_lights[i].range * 1.5f;
        const glm::mat4 proj     = glm::perspective(glm::radians(90.0f), 1.0f, 0.01f, farPlane);

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

void Renderer::GBufferPass(const Camera& camera)
{
    m_gbuffer.BindForWriting();

    glViewport(0, 0, m_viewportW, m_viewportH);
    glEnable(GL_DEPTH_TEST);
    glDisable(GL_BLEND);
    glDisable(GL_CULL_FACE);

    glClearColor(0, 0, 0, 1);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    m_gbufferShader.Bind();
    m_gbufferShader.SetMat4("uView", camera.GetView());
    m_gbufferShader.SetMat4("uProj", camera.GetProj());

    // helper lambdas
    auto SetMaterial = [&](const glm::vec3& kd)
    {
        m_gbufferShader.SetVec3("uKd", kd);
        m_gbufferShader.SetVec3("uKs", m_mat.ks);
        m_gbufferShader.SetFloat("uAlpha", m_mat.shininess);
    };

    // Cornell walls
    m_gbufferShader.SetMat4("uModel", glm::mat4(1.0f));
    for (const auto& part : m_cornell.parts)
    {
        SetMaterial(part.albedo);
        m_cornellMesh.DrawRange(part.indexStart, part.indexCount);
    }

    // Ground
    m_gbufferShader.SetMat4("uModel", glm::mat4(1.0f));
    SetMaterial(glm::vec3(0.20f));
    m_groundMesh.Draw();

    constexpr float cornellFloorY = -1.0f;

    auto DrawCube = [&](glm::mat4 model, glm::vec3 kd)
    {
        m_gbufferShader.SetMat4("uModel", model);
        SetMaterial(kd);
        m_cubeMesh.Draw();
    };

    auto DrawSphere = [&](glm::mat4 model, glm::vec3 kd)
    {
        m_gbufferShader.SetMat4("uModel", model);
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

    m_fullscreenShader.Bind();
    BindGBufferTextures(m_gbuffer, m_fullscreenShader);

    // Bind shadow cube maps to texture units 4..4+kMaxLights-1
    for (int i = 0; i < kMaxLights; ++i)
    {
        glActiveTexture(GL_TEXTURE4 + i);
        glBindTexture(GL_TEXTURE_CUBE_MAP, m_shadowMaps[i].TexCube());
        m_fullscreenShader.SetInt(("uShadowMaps[" + std::to_string(i) + "]").c_str(), 4 + i);
    }
    m_fullscreenShader.SetInt("uShadowsEnabled", m_shadowsEnabled ? 1 : 0);
    m_fullscreenShader.SetFloat("uShadowBias", m_shadowBias);
    m_fullscreenShader.SetFloat("uShadowPcfRadius", m_shadowPcfRadius);

    float farPlanes[kMaxLights];
    for (int i = 0; i < kMaxLights; ++i)
        farPlanes[i] = m_lights[i].range * 1.5f;
    m_fullscreenShader.SetFloatArray("uShadowFarPlane", farPlanes, kMaxLights);

    m_fullscreenShader.SetInt("uDebugView", static_cast<int>(m_debugView));

    m_fullscreenShader.SetVec3("uCamPos", camera.GetPosition());
    m_fullscreenShader.SetFloat("uAmbient", m_mat.ambient);

    // push lights (used for Final + the new debug views)
    int count = std::clamp(m_lightCount, 0, kMaxLights);
    m_fullscreenShader.SetInt("uLightCount", count);

    // NOTE: shader MAX_LIGHTS is 64. keep count <=64
    glm::vec3 pos[kMaxLights];
    glm::vec3 col[kMaxLights];
    float rng[kMaxLights];
    for (int i = 0; i < count; ++i)
    {
        pos[i] = m_lights[i].position;
        col[i] = (m_lightEnabled[i] ? m_lights[i].color * m_lightIntensity[i] : glm::vec3(0.0f));
        rng[i] = m_lights[i].range;
    }
    m_fullscreenShader.SetVec3Array("uLightPos", pos, count);
    m_fullscreenShader.SetVec3Array("uLightColor", col, count);

    m_fullscreenShader.SetFloatArray("uLightRange", rng, count);

    m_fullscreenShader.SetInt("uDebugLightIndex", m_debugLightIndex);
    m_fullscreenShader.SetFloat("uGlobeRadius", m_globeRadius);

    glBindVertexArray(m_quadVAO);
    glDrawArrays(GL_TRIANGLES, 0, 6);
    glBindVertexArray(0);

    m_fullscreenShader.Unbind();
}

void Renderer::LocalLightsPass(const Camera& camera)
{
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
        m_localLightShader.SetInt("uShadowsActive", m_shadowsEnabled ? 1 : 0);
        m_localLightShader.SetFloat("uShadowFarPlane", m_lights[i].range * 1.5f);
        m_localLightShader.SetFloat("uShadowBias", m_shadowBias);
        m_localLightShader.SetFloat("uShadowPcfRadius", m_shadowPcfRadius);

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
                               "Brightness"};

        int mode = static_cast<int>(m_debugView);
        if (ImGui::Combo("Deferred View", &mode, items, IM_ARRAYSIZE(items)))
            m_debugView = static_cast<DebugView>(mode);

        if (m_debugView == DebugView::Brightness)
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
        ImGui::DragFloat("Shadow Bias",     &m_shadowBias,      0.001f, 0.0f, 0.2f);
        ImGui::DragFloat("PCF Disk Radius", &m_shadowPcfRadius, 0.005f, 0.0f, 0.3f);
    }

    ImGui::Separator();

    ImGui::Text("Objects");
    ImGui::ColorEdit3("Tall Cube Albedo", &m_albedoTall.x);
    ImGui::ColorEdit3("Short Cube Albedo", &m_albedoShort.x);
    ImGui::ColorEdit3("Small Cube Albedo", &m_albedoSmall.x);
    ImGui::ColorEdit3("Sphere Albedo", &m_albedoSphere.x);

    ImGui::Separator();
    ImGui::Text("Material");
    ImGui::DragFloat("Ambient", &m_mat.ambient, 0.001f, 0.0f, 1.0f);
    ImGui::DragFloat("Shininess (alpha)", &m_mat.shininess, 1.0f, 1.0f, 256.0f);
    ImGui::ColorEdit3("Ks", &m_mat.ks.x);

    ImGui::End();
}
