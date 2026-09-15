#include "pch.h"
#include <algorithm>
#include <glad/glad.h>
#include <glm/ext/matrix_transform.hpp>
#include <glm/ext/matrix_clip_space.hpp>

#include "graphics/Renderer.h"

// ---------------------------------------------------------------------------
// Cubemap face directions — shared between ShadowPass and MSMShadowPass.
// ---------------------------------------------------------------------------
static const glm::vec3 kCubeFaceTargets[6] = {
    { 1,  0,  0}, {-1,  0,  0},
    { 0,  1,  0}, { 0, -1,  0},
    { 0,  0,  1}, { 0,  0, -1},
};
static const glm::vec3 kCubeFaceUps[6] = {
    { 0, -1,  0}, { 0, -1,  0},
    { 0,  0,  1}, { 0,  0, -1},
    { 0, -1,  0}, { 0, -1,  0},
};

// Matches deferred_ibl.frag's RotateY() - needed to keep the sun shadow aligned with uHDRIRotation.
static glm::vec3 RotateYAxis(const glm::vec3& d, float yaw)
{
    float c = glm::cos(yaw), s = glm::sin(yaw);
    return glm::vec3(d.x * c + d.z * s, d.y, -d.x * s + d.z * c);
}

void Renderer::RenderForward(const Camera& camera)
{
    glViewport(0, 0, m_viewportW, m_viewportH);
    glBindFramebuffer(GL_FRAMEBUFFER, 0);

    glClearColor(0.1f, 0.12f, 0.15f, 1.0f);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    glEnable(GL_DEPTH_TEST);

    m_litShader.Bind();

    m_litShader.SetMat4("uView", camera.GetView());
    m_litShader.SetMat4("uProj", camera.GetProj());
    m_litShader.SetVec3("uCamPos", camera.GetPosition());

    m_litShader.SetFloat("uAmbient", m_ambient);

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

    DrawSceneObjectsLit(m_litShader, true);

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
    DirectionalShadowPass();
    GBufferPass(camera);
    if (m_aoEnabled)
    {
        AOPass(camera);
        AOBlurHPass(camera);
        AOBlurVPass(camera);
    }
    FullscreenLightPass(camera);
    LocalLightsPass(camera);

    if (m_celEnabled || m_debugView == DebugView::CelOutlineMask)
        CelOutlinePass(camera);

    DrawLightGizmos(camera);

    glm::mat4 skeletalModel = (m_skeletalObjectIndex >= 0)
                                  ? ComputeModelMatrix(m_activeScene.objects[m_skeletalObjectIndex])
                                  : glm::mat4(1.0f);
    DrawBoneLines(camera, skeletalModel, m_skeleton, m_animator.worldPose);
}

// Draw all scene geometry using the supplied shader (uModel must exist in shader).
// Used for both GBuffer and shadow passes.
void Renderer::DrawSceneGeometry(Shader& sh)
{
    for (const auto& obj : m_activeScene.objects)
    {
        if (!obj.visible || !obj.skeleton.modelFile.empty())
            continue;

        Mesh* mesh = ResolveMesh(obj.meshRef);
        if (!mesh)
            continue;

        sh.SetMat4("uModel", ComputeModelMatrix(obj));
        mesh->Draw();
    }
}

// Binds sh and uploads uModel/uBoneMatrices for the active skeletal object; returns false if there's nothing to cast.
bool Renderer::BindSkinnedShadowCaster(Shader& sh)
{
    if (m_skeletalObjectIndex < 0 || !m_showMesh ||
        !m_activeScene.objects[m_skeletalObjectIndex].visible)
        return false;

    const SceneObject& obj = m_activeScene.objects[m_skeletalObjectIndex];
    glm::mat4 model = ComputeModelMatrix(obj);

    std::vector<float> boneMatrices;
    boneMatrices.reserve(m_animator.skinningMatrices.size() * 16);
    for (const Anim::Mat4& bm : m_animator.skinningMatrices)
        boneMatrices.insert(boneMatrices.end(), bm.m, bm.m + 16);

    sh.Bind();
    sh.SetMat4("uModel", model);
    sh.SetMat4Array("uBoneMatrices", boneMatrices.data(),
                    static_cast<int>(m_animator.skinningMatrices.size()));
    return true;
}

struct AuxMaterialTextures
{
    Texture* specular = nullptr;
    Texture* roughness = nullptr;
    Texture* metallic = nullptr;
    Texture* normal = nullptr;
};

struct MaterialStrengths
{
    float albedo = 1.0f;
    float specular = 1.0f;
    float roughness = 1.0f;
    float metallic = 1.0f;
    float normal = 1.0f;
};

static void SetObjectMaterial(Shader& sh, bool isForwardPass, const glm::vec3& kd,
                              const glm::vec3& ks, float alpha, float metallic,
                              Texture* albedoTex = nullptr,
                              const AuxMaterialTextures& aux = {},
                              const MaterialStrengths& strength = {})
{
    if (albedoTex)
    {
        albedoTex->Bind(0);
        sh.SetInt("uAlbedoTex", 0);
        sh.SetInt("uHasAlbedoTex", 1);
    }
    else
    {
        sh.SetInt("uHasAlbedoTex", 0);
    }

    if (aux.specular)
    {
        aux.specular->Bind(1);
        sh.SetInt("uSpecularTex", 1);
        sh.SetInt("uHasSpecularTex", 1);
    }
    else
    {
        sh.SetInt("uHasSpecularTex", 0);
    }

    if (aux.roughness)
    {
        aux.roughness->Bind(2);
        sh.SetInt("uRoughnessTex", 2);
        sh.SetInt("uHasRoughnessTex", 1);
    }
    else
    {
        sh.SetInt("uHasRoughnessTex", 0);
    }

    if (aux.metallic)
    {
        aux.metallic->Bind(3);
        sh.SetInt("uMetallicTex", 3);
        sh.SetInt("uHasMetallicTex", 1);
    }
    else
    {
        sh.SetInt("uHasMetallicTex", 0);
    }

    if (aux.normal)
    {
        aux.normal->Bind(4);
        sh.SetInt("uNormalTex", 4);
        sh.SetInt("uHasNormalTex", 1);
    }
    else
    {
        sh.SetInt("uHasNormalTex", 0);
    }

    sh.SetVec3(isForwardPass ? "uAlbedo" : "uKd", kd);
    sh.SetVec3("uKs", ks);
    sh.SetFloat("uAlpha", alpha);
    sh.SetFloat("uMetallic", metallic);

    sh.SetFloat("uAlbedoStrength", strength.albedo);
    sh.SetFloat("uSpecularStrength", strength.specular);
    sh.SetFloat("uRoughnessStrength", strength.roughness);
    sh.SetFloat("uMetallicStrength", strength.metallic);
    sh.SetFloat("uNormalStrength", strength.normal);
}

// Material-aware scene draw, shared by GBufferPass and RenderForward.
void Renderer::DrawSceneObjectsLit(Shader& sh, bool isForwardPass)
{
    for (const auto& obj : m_activeScene.objects)
    {
        if (!obj.visible || !obj.skeleton.modelFile.empty())
            continue;

        if (obj.role == ObjectRole::Cornell)
        {
            glm::mat4 M = ComputeModelMatrix(obj);
            sh.SetMat4("uModel", M);
            sh.SetMat3("uNormalMatrix", glm::mat3(glm::transpose(glm::inverse(M))));
            MaterialStrengths cornellStrength{
                obj.material.albedoStrength, obj.material.specularStrength,
                obj.material.roughnessStrength, obj.material.metallicStrength,
                obj.material.normalStrength};
            for (const auto& part : m_cornell.parts)
            {
                SetObjectMaterial(sh, isForwardPass, part.albedo, obj.material.ks,
                                  obj.material.alpha, obj.material.metallic, nullptr, {},
                                  cornellStrength);
                m_cornellMesh.DrawRange(part.indexStart, part.indexCount);
            }
            continue;
        }

        Mesh* mesh = ResolveMesh(obj.meshRef);
        if (!mesh)
            continue;

        glm::mat4 M = ComputeModelMatrix(obj);
        glm::mat3 N = glm::mat3(glm::transpose(glm::inverse(M)));

        sh.SetMat4("uModel", M);
        sh.SetMat3("uNormalMatrix", N);

        MaterialStrengths matStrength{
            obj.material.albedoStrength, obj.material.specularStrength,
            obj.material.roughnessStrength, obj.material.metallicStrength,
            obj.material.normalStrength};

        const std::vector<Geometry::SubmeshRange>* submeshes = ResolveSubmeshes(obj.meshRef);
        if (submeshes && !submeshes->empty())
        {
            for (const auto& part : *submeshes)
            {
                Texture* tex =
                    part.albedoTexture.empty() ? nullptr : ResolveModelTexture(part.albedoTexture);
                glm::vec3 kd = tex ? part.albedo : obj.material.kd;

                AuxMaterialTextures aux;
                if (!part.specularTexture.empty())
                    aux.specular = ResolveModelTexture(part.specularTexture, /*srgb=*/false);
                if (!part.roughnessTexture.empty())
                    aux.roughness = ResolveModelTexture(part.roughnessTexture, /*srgb=*/false);
                if (!part.metallicTexture.empty())
                    aux.metallic = ResolveModelTexture(part.metallicTexture, /*srgb=*/false);
                if (!part.normalTexture.empty())
                    aux.normal = ResolveModelTexture(part.normalTexture, /*srgb=*/false);

                SetObjectMaterial(sh, isForwardPass, kd, obj.material.ks, obj.material.alpha,
                                  obj.material.metallic, tex, aux, matStrength);
                mesh->DrawRange(part.indexStart, part.indexCount);
            }
            continue;
        }

        SetObjectMaterial(sh, isForwardPass, obj.material.kd, obj.material.ks, obj.material.alpha,
                          obj.material.metallic, nullptr, {}, matStrength);
        mesh->Draw();
    }
}

void Renderer::ShadowPass()
{
    if (!m_shadowsEnabled)
        return;

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
        const float farPlane = kShadowFarPlane;
        const glm::mat4 proj = glm::perspective(glm::radians(90.0f), 1.0f, 0.01f, farPlane);

        m_shadowShader.SetVec3("uLightPos", lightPos);
        m_shadowShader.SetFloat("uFarPlane", farPlane);

        for (int face = 0; face < 6; ++face)
        {
            m_shadowMaps[i].BindForFace(face);
            glViewport(0, 0, m_shadowMaps[i].Resolution(), m_shadowMaps[i].Resolution());
            glClear(GL_DEPTH_BUFFER_BIT);

            glm::mat4 view = glm::lookAt(lightPos, lightPos + kCubeFaceTargets[face], kCubeFaceUps[face]);
            m_shadowShader.SetMat4("uLightVP", proj * view);

            DrawSceneGeometry(m_shadowShader);
        }

        ShadowMap::Unbind();
    }

    m_shadowShader.Unbind();

    if (BindSkinnedShadowCaster(m_shadowSkinnedShader))
    {
        for (int i = 0; i < count; ++i)
        {
            if (!m_lightEnabled[i])
                continue;

            const glm::vec3 lightPos = m_lights[i].position;
            const float farPlane = kShadowFarPlane;
            const glm::mat4 proj = glm::perspective(glm::radians(90.0f), 1.0f, 0.01f, farPlane);

            m_shadowSkinnedShader.SetVec3("uLightPos", lightPos);
            m_shadowSkinnedShader.SetFloat("uFarPlane", farPlane);

            for (int face = 0; face < 6; ++face)
            {
                m_shadowMaps[i].BindForFace(face);
                glViewport(0, 0, m_shadowMaps[i].Resolution(), m_shadowMaps[i].Resolution());

                glm::mat4 view =
                    glm::lookAt(lightPos, lightPos + kCubeFaceTargets[face], kCubeFaceUps[face]);
                m_shadowSkinnedShader.SetMat4("uLightVP", proj * view);

                m_skinnedMesh.Draw();
            }

            ShadowMap::Unbind();
        }

        m_shadowSkinnedShader.Unbind();
    }

    // Restore viewport for subsequent passes
    glViewport(0, 0, m_viewportW, m_viewportH);
}

void Renderer::MSMShadowPass()
{
    if (!m_shadowsEnabled)
        return;

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
        const float farPlane = kShadowFarPlane;
        const glm::mat4 proj = glm::perspective(glm::radians(90.0f), 1.0f, 0.01f, farPlane);

        m_msmMomentShader.SetVec3("uLightPos", lightPos);
        m_msmMomentShader.SetFloat("uFarPlane", farPlane);

        for (int face = 0; face < 6; ++face)
        {
            m_msmMaps[i].BindForCapture(face);
            glViewport(0, 0, m_msmMaps[i].Resolution(), m_msmMaps[i].Resolution());
            glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

            glm::mat4 view = glm::lookAt(lightPos, lightPos + kCubeFaceTargets[face], kCubeFaceUps[face]);
            m_msmMomentShader.SetMat4("uLightVP", proj * view);

            DrawSceneGeometry(m_msmMomentShader);
        }

        MomentShadowMap::Unbind();
    }

    m_msmMomentShader.Unbind();

    if (BindSkinnedShadowCaster(m_msmMomentSkinnedShader))
    {
        for (int i = 0; i < count; ++i)
        {
            if (!m_lightEnabled[i])
                continue;

            const glm::vec3 lightPos = m_lights[i].position;
            const float farPlane = kShadowFarPlane;
            const glm::mat4 proj = glm::perspective(glm::radians(90.0f), 1.0f, 0.01f, farPlane);

            m_msmMomentSkinnedShader.SetVec3("uLightPos", lightPos);
            m_msmMomentSkinnedShader.SetFloat("uFarPlane", farPlane);

            for (int face = 0; face < 6; ++face)
            {
                m_msmMaps[i].BindForCapture(face);
                glViewport(0, 0, m_msmMaps[i].Resolution(), m_msmMaps[i].Resolution());

                glm::mat4 view =
                    glm::lookAt(lightPos, lightPos + kCubeFaceTargets[face], kCubeFaceUps[face]);
                m_msmMomentSkinnedShader.SetMat4("uLightVP", proj * view);

                m_skinnedMesh.Draw();
            }

            MomentShadowMap::Unbind();
        }

        m_msmMomentSkinnedShader.Unbind();
    }

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

// Single orthographic depth pass from the HDRI's approximated dominant direction (see ComputeHDRISunDirection).
void Renderer::DirectionalShadowPass()
{
    // Only sampled from deferred_ibl.frag's ambient term - skip the render entirely outside IBL mode.
    if (!m_shadowsEnabled || !m_hasHDRISun || m_lightingMode != LightingMode::IBL)
        return;

    const glm::vec3 sceneCenter(0.0f);
    const glm::vec3 sunDir = RotateYAxis(m_hdriSunDir, -m_hdriRotation);  // inverse of the shader's sample-direction rotation
    const glm::vec3 lightPos = sceneCenter + sunDir * kSunShadowDistance;
    const glm::vec3 up = (glm::abs(sunDir.y) > 0.99f) ? glm::vec3(0, 0, 1) : glm::vec3(0, 1, 0);
    const glm::mat4 view = glm::lookAt(lightPos, sceneCenter, up);
    const glm::mat4 proj = glm::ortho(-kSunShadowHalfExtent, kSunShadowHalfExtent,
                                      -kSunShadowHalfExtent, kSunShadowHalfExtent,
                                      0.1f, kSunShadowDistance * 2.0f);
    m_sunLightVP = proj * view;

    glEnable(GL_DEPTH_TEST);
    glDisable(GL_CULL_FACE);
    glDisable(GL_BLEND);

    m_sunShadowMap.BindForWriting();
    glViewport(0, 0, m_sunShadowMap.Resolution(), m_sunShadowMap.Resolution());
    glClear(GL_DEPTH_BUFFER_BIT);

    m_sunShadowShader.Bind();
    m_sunShadowShader.SetMat4("uLightVP", m_sunLightVP);
    DrawSceneGeometry(m_sunShadowShader);
    m_sunShadowShader.Unbind();

    if (BindSkinnedShadowCaster(m_sunShadowSkinnedShader))
    {
        m_sunShadowSkinnedShader.SetMat4("uLightVP", m_sunLightVP);
        m_skinnedMesh.Draw();
        m_sunShadowSkinnedShader.Unbind();
    }

    DirectionalShadowMap::Unbind();
    glViewport(0, 0, m_viewportW, m_viewportH);
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
    glClearBufferfv(GL_COLOR, 2, kClearZero);  // Kd + metallic
    glClearBufferfv(GL_COLOR, 3, kClearZero);  // Ks + roughness

    m_gbufferShader.Bind();
    m_gbufferShader.SetMat4("uView", camera.GetView());
    m_gbufferShader.SetMat4("uProj", camera.GetProj());

    DrawSceneObjectsLit(m_gbufferShader, false);

    m_gbufferShader.Unbind();

    DrawSkinnedObject(camera);

    GBuffer::UnbindWriting();
}

// Draws the single skeletal scene object (see LoadSkeletalObjects); not in DrawSceneGeometry/DrawSceneObjectsLit since it needs its own vertex format and shader. Shadow casting handled separately in ShadowPass/MSMShadowPass.
void Renderer::DrawSkinnedObject(const Camera& camera)
{
    if (m_skeletalObjectIndex < 0 ||
        m_skeletalObjectIndex >= static_cast<int>(m_activeScene.objects.size()))
        return;

    if (!m_showMesh)
        return;

    const SceneObject& obj = m_activeScene.objects[m_skeletalObjectIndex];
    if (!obj.visible)
        return;

    glm::mat4 M = ComputeModelMatrix(obj);
    glm::mat3 N = glm::mat3(glm::transpose(glm::inverse(M)));

    m_gbufferSkinnedShader.Bind();
    m_gbufferSkinnedShader.SetMat4("uView", camera.GetView());
    m_gbufferSkinnedShader.SetMat4("uProj", camera.GetProj());
    m_gbufferSkinnedShader.SetMat4("uModel", M);
    m_gbufferSkinnedShader.SetMat3("uNormalMatrix", N);

    // Anim::Mat4 is already column-major / GL layout, so the flattened bone array uploads straight through SetMat4Array without touching glm.
    std::vector<float> boneMatrices;
    boneMatrices.reserve(m_animator.skinningMatrices.size() * 16);
    for (const Anim::Mat4& bm : m_animator.skinningMatrices)
        boneMatrices.insert(boneMatrices.end(), bm.m, bm.m + 16);
    m_gbufferSkinnedShader.SetMat4Array(
        "uBoneMatrices", boneMatrices.data(), static_cast<int>(m_animator.skinningMatrices.size()));

    MaterialStrengths skinnedStrength{
        obj.material.albedoStrength, obj.material.specularStrength,
        obj.material.roughnessStrength, obj.material.metallicStrength,
        obj.material.normalStrength};

    if (!m_skinnedSubmeshes.empty())
    {
        for (const auto& part : m_skinnedSubmeshes)
        {
            Texture* tex =
                part.albedoTexture.empty() ? nullptr : ResolveModelTexture(part.albedoTexture);
            glm::vec3 kd = tex ? part.albedo : obj.material.kd;

            AuxMaterialTextures aux;
            if (!part.specularTexture.empty())
                aux.specular = ResolveModelTexture(part.specularTexture, /*srgb=*/false);
            if (!part.roughnessTexture.empty())
                aux.roughness = ResolveModelTexture(part.roughnessTexture, /*srgb=*/false);
            if (!part.metallicTexture.empty())
                aux.metallic = ResolveModelTexture(part.metallicTexture, /*srgb=*/false);
            if (!part.normalTexture.empty())
                aux.normal = ResolveModelTexture(part.normalTexture, /*srgb=*/false);

            SetObjectMaterial(m_gbufferSkinnedShader, false, kd, obj.material.ks,
                              obj.material.alpha, obj.material.metallic, tex, aux,
                              skinnedStrength);
            m_skinnedMesh.DrawRange(part.indexStart, part.indexCount);
        }
    }
    else
    {
        SetObjectMaterial(m_gbufferSkinnedShader, false, obj.material.kd, obj.material.ks,
                          obj.material.alpha, obj.material.metallic, nullptr, {},
                          skinnedStrength);
        m_skinnedMesh.Draw();
    }

    m_gbufferSkinnedShader.Unbind();
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

void Renderer::AOPass(const Camera& camera)
{
    m_aoRawBuffer.BindForWriting();
    glViewport(0, 0, m_viewportW, m_viewportH);
    glDisable(GL_DEPTH_TEST);
    glDisable(GL_BLEND);

    m_aoShader.Bind();

    glActiveTexture(GL_TEXTURE0);
    glBindTexture(GL_TEXTURE_2D, m_gbuffer.TexWorldPos());
    m_aoShader.SetInt("uWorldPosTex", 0);

    glActiveTexture(GL_TEXTURE1);
    glBindTexture(GL_TEXTURE_2D, m_gbuffer.TexNormal());
    m_aoShader.SetInt("uNormalTex", 1);

    m_aoShader.SetMat4("uView", camera.GetView());
    m_aoShader.SetFloat("uR", m_aoRadius);
    m_aoShader.SetInt("uN", m_aoSamples);
    m_aoShader.SetFloat("uC", 0.1f * m_aoRadius);
    m_aoShader.SetFloat("uDelta", m_aoDelta);
    m_aoShader.SetFloat("uScaleS", m_aoScale);
    m_aoShader.SetFloat("uContrastK", m_aoContrast);

    glBindVertexArray(m_quadVAO);
    glDrawArrays(GL_TRIANGLES, 0, 6);
    glBindVertexArray(0);

    m_aoShader.Unbind();
    AOBuffer::UnbindWriting();
}

void Renderer::AOBlurHPass(const Camera& camera)
{
    m_aoBlurHBuffer.BindForWriting();
    glViewport(0, 0, m_viewportW, m_viewportH);
    glDisable(GL_DEPTH_TEST);
    glDisable(GL_BLEND);

    m_aoBlurHShader.Bind();

    glActiveTexture(GL_TEXTURE0);
    glBindTexture(GL_TEXTURE_2D, m_aoRawBuffer.TexAO());
    m_aoBlurHShader.SetInt("uAOTex", 0);

    glActiveTexture(GL_TEXTURE1);
    glBindTexture(GL_TEXTURE_2D, m_gbuffer.TexWorldPos());
    m_aoBlurHShader.SetInt("uWorldPosTex", 1);

    glActiveTexture(GL_TEXTURE2);
    glBindTexture(GL_TEXTURE_2D, m_gbuffer.TexNormal());
    m_aoBlurHShader.SetInt("uNormalTex", 2);

    m_aoBlurHShader.SetMat4("uView", camera.GetView());
    m_aoBlurHShader.SetVec2("uTexelDir",
        glm::vec2(1.0f / static_cast<float>(m_viewportW), 0.0f));
    m_aoBlurHShader.SetFloat("uDepthSigma", m_aoDepthSigma);
    m_aoBlurHShader.SetInt("uBlurRadius", m_aoBlurRadius);

    glBindVertexArray(m_quadVAO);
    glDrawArrays(GL_TRIANGLES, 0, 6);
    glBindVertexArray(0);

    m_aoBlurHShader.Unbind();
    AOBuffer::UnbindWriting();
}

void Renderer::AOBlurVPass(const Camera& camera)
{
    m_aoBlurVBuffer.BindForWriting();
    glViewport(0, 0, m_viewportW, m_viewportH);
    glDisable(GL_DEPTH_TEST);
    glDisable(GL_BLEND);

    m_aoBlurVShader.Bind();

    glActiveTexture(GL_TEXTURE0);
    glBindTexture(GL_TEXTURE_2D, m_aoBlurHBuffer.TexAO());
    m_aoBlurVShader.SetInt("uAOTex", 0);

    glActiveTexture(GL_TEXTURE1);
    glBindTexture(GL_TEXTURE_2D, m_gbuffer.TexWorldPos());
    m_aoBlurVShader.SetInt("uWorldPosTex", 1);

    glActiveTexture(GL_TEXTURE2);
    glBindTexture(GL_TEXTURE_2D, m_gbuffer.TexNormal());
    m_aoBlurVShader.SetInt("uNormalTex", 2);

    m_aoBlurVShader.SetMat4("uView", camera.GetView());
    m_aoBlurVShader.SetVec2("uTexelDir",
        glm::vec2(0.0f, 1.0f / static_cast<float>(m_viewportH)));
    m_aoBlurVShader.SetFloat("uDepthSigma", m_aoDepthSigma);
    m_aoBlurVShader.SetInt("uBlurRadius", m_aoBlurRadius);

    glBindVertexArray(m_quadVAO);
    glDrawArrays(GL_TRIANGLES, 0, 6);
    glBindVertexArray(0);

    m_aoBlurVShader.Unbind();
    AOBuffer::UnbindWriting();
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

    // No-op on deferred_ibl.frag (uniform doesn't exist there — always direct-lit).
    // On deferred_light.frag, disables the direct-light loop when LocalLightsPass
    // (light volumes) is the one supplying direct light instead.
    sh.SetInt("uDirectLightingEnabled", m_useLightVolumes ? 0 : 1);

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
        farPlanes[i] = kShadowFarPlane;
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

    sh.SetFloat("uShadowStrength", m_shadowStrength);

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

        // Directional "sun" shadow (approximated HDRI dominant direction) — unit 17
        glActiveTexture(GL_TEXTURE17);
        glBindTexture(GL_TEXTURE_2D, m_sunShadowMap.Tex());
        sh.SetInt("uSunShadowMap", 17);
        sh.SetMat4("uSunLightVP", m_sunLightVP);
        sh.SetInt("uHasSunShadow", m_hasHDRISun ? 1 : 0);
        sh.SetFloat("uSunShadowBias", m_sunShadowBias);

        // SH coefficients UBO — keep bound at binding point 2
        if (m_shCoeffsUBO != 0)
            glBindBufferBase(GL_UNIFORM_BUFFER, 2, m_shCoeffsUBO);

        // Inverse view-projection for skydome ray reconstruction
        glm::mat4 invVP = glm::inverse(camera.GetProj() * camera.GetView());
        sh.SetMat4("uInvViewProj", invVP);
    }
    else
    {
        sh.SetFloat("uAmbient", m_ambient);
    }

    // AO texture — unit 16.
    // Debug views 12/13/14 each show a different stage of the AO pipeline;
    // the shader reads uAOTex for those early-out paths too, so we just point
    // it at the right buffer here rather than duplicating texture binds.
    GLuint aoTex = 0;
    if (m_aoEnabled)
    {
        switch (m_debugView)
        {
            case DebugView::AOMapRaw:   aoTex = m_aoRawBuffer.TexAO();   break;
            case DebugView::AOMapBlurH: aoTex = m_aoBlurHBuffer.TexAO(); break;
            default:                    aoTex = m_aoBlurVBuffer.TexAO(); break;
        }
    }
    glActiveTexture(GL_TEXTURE16);
    glBindTexture(GL_TEXTURE_2D, aoTex);
    sh.SetInt("uAOTex", 16);
    sh.SetInt("uAOEnabled", m_aoEnabled ? 1 : 0);
    sh.SetFloat("uAOStrength", m_aoStrength);

    sh.SetInt("uToonEnabled", (m_celEnabled && m_toonEnabled) ? 1 : 0);
    sh.SetInt("uToonBands",   m_toonBands);

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

    // Mutually exclusive with the fullscreen "many lights" loop in
    // deferred_light.frag — only one of the two may contribute direct light.
    if (!m_useLightVolumes)
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
        m_localLightShader.SetFloat("uShadowFarPlane", kShadowFarPlane);
        m_localLightShader.SetFloat("uShadowBias", m_shadowBias);
        m_localLightShader.SetFloat("uShadowPcfRadius", m_shadowPcfRadius);
        m_localLightShader.SetInt("uUseMSM", m_useMSM ? 1 : 0);
        m_localLightShader.SetFloat("uMSMAlpha", m_msmAlpha);
        m_localLightShader.SetFloat("uShadowStrength", m_shadowStrength);

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

void Renderer::CelOutlinePass(const Camera& camera)
{
    glBindFramebuffer(GL_FRAMEBUFFER, 0);
    glDisable(GL_DEPTH_TEST);
    glDisable(GL_CULL_FACE);

    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    m_celOutlineShader.Bind();

    // GBuffer world-pos → unit 0, normals → unit 1
    glActiveTexture(GL_TEXTURE0);
    glBindTexture(GL_TEXTURE_2D, m_gbuffer.TexWorldPos());
    m_celOutlineShader.SetInt("uWorldPosTex", 0);

    glActiveTexture(GL_TEXTURE1);
    glBindTexture(GL_TEXTURE_2D, m_gbuffer.TexNormal());
    m_celOutlineShader.SetInt("uNormalTex", 1);

    m_celOutlineShader.SetMat4("uView", camera.GetView());
    m_celOutlineShader.SetVec2("uTexelSize",
        glm::vec2(1.0f / float(m_viewportW), 1.0f / float(m_viewportH)));
    m_celOutlineShader.SetFloat("uThickness",        m_outlineThickness);
    m_celOutlineShader.SetFloat("uDepthThreshold",   m_depthThreshold);
    m_celOutlineShader.SetFloat("uNormalThreshold",  m_normalThreshold);
    m_celOutlineShader.SetVec3("uOutlineColor",      m_outlineColor);
    m_celOutlineShader.SetInt("uDebugOutline",
        (m_debugView == DebugView::CelOutlineMask) ? 1 : 0);

    glBindVertexArray(m_quadVAO);
    glDrawArrays(GL_TRIANGLES, 0, 6);
    glBindVertexArray(0);

    m_celOutlineShader.Unbind();

    glDisable(GL_BLEND);
    glEnable(GL_DEPTH_TEST);
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

void Renderer::EnsureBoneLineBuffer()
{
    if (m_boneLineVAO != 0)
        return;

    glGenVertexArrays(1, &m_boneLineVAO);
    glGenBuffers(1, &m_boneLineVBO);

    glBindVertexArray(m_boneLineVAO);
    glBindBuffer(GL_ARRAY_BUFFER, m_boneLineVBO);

    // layout(location=0) vec3 aPos; buffer contents rebuilt every DrawBoneLines call
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), static_cast<void*>(0));

    glBindBuffer(GL_ARRAY_BUFFER, 0);
    glBindVertexArray(0);
}

void Renderer::DestroyBoneLineBuffer()
{
    if (m_boneLineVBO)
    {
        glDeleteBuffers(1, &m_boneLineVBO);
        m_boneLineVBO = 0;
    }
    if (m_boneLineVAO)
    {
        glDeleteVertexArrays(1, &m_boneLineVAO);
        m_boneLineVAO = 0;
    }
}

void Renderer::DrawBoneLines(const Camera& camera, const glm::mat4& modelMatrix,
                             const Anim::Skeleton& skeleton, const std::vector<Anim::VQS>& worldPose)
{
    if (!m_showSkeleton || m_boneLineVAO == 0)
        return;

    std::vector<float> vertices;
    vertices.reserve(skeleton.bones.size() * 6);

    for (size_t i = 0; i < skeleton.bones.size(); ++i)
    {
        int parentIndex = skeleton.bones[i].parentIndex;
        if (parentIndex < 0 || parentIndex >= static_cast<int>(worldPose.size()) ||
            i >= worldPose.size())
            continue;

        const Anim::Vec3& parentPos = worldPose[parentIndex].v;
        const Anim::Vec3& childPos  = worldPose[i].v;
        vertices.insert(vertices.end(),
                        {parentPos.x, parentPos.y, parentPos.z, childPos.x, childPos.y, childPos.z});
    }

    m_boneLineVertexCount = static_cast<int>(vertices.size() / 3);
    if (m_boneLineVertexCount == 0)
        return;

    glBindBuffer(GL_ARRAY_BUFFER, m_boneLineVBO);
    glBufferData(GL_ARRAY_BUFFER,
                static_cast<GLsizeiptr>(vertices.size() * sizeof(float)),
                vertices.data(),
                GL_DYNAMIC_DRAW);

    glDisable(GL_DEPTH_TEST);  // overlay draws on top, debug-only like the light gizmos
    glDisable(GL_CULL_FACE);

    m_boneLineShader.Bind();
    m_boneLineShader.SetMat4("uModel", modelMatrix);
    m_boneLineShader.SetMat4("uView", camera.GetView());
    m_boneLineShader.SetMat4("uProj", camera.GetProj());
    m_boneLineShader.SetVec3("uColor", glm::vec3(1.0f, 0.85f, 0.2f));

    glBindVertexArray(m_boneLineVAO);
    glDrawArrays(GL_LINES, 0, m_boneLineVertexCount);
    glBindVertexArray(0);

    m_boneLineShader.Unbind();
    glEnable(GL_DEPTH_TEST);
}
