#include "pch.h"
#include <algorithm>
#include <ImGuizmo.h>
#include <glm/ext/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>

#include "graphics/Renderer.h"

void Renderer::DrawScenePanel()
{
    if (!m_ready)
        return;

    ImGui::Begin("Scene");

    ImGui::Text("Scene: %s", m_activeScene.name.c_str());
    {
        std::vector<const char*> items;
        items.reserve(m_sceneFiles.size());
        for (const auto& f : m_sceneFiles) items.push_back(f.c_str());

        if (items.empty())
        {
            ImGui::TextDisabled("No .json files found in %s", kScenesFolder);
        }
        else
        {
            ImGui::SetNextItemWidth(-1.0f);
            ImGui::Combo("##scene_pick", &m_sceneSelectedIdx,
                         items.data(), static_cast<int>(items.size()));
        }
    }

    const bool canSwitchScene = m_sceneSelectedIdx >= 0 &&
                                m_sceneSelectedIdx < static_cast<int>(m_sceneFiles.size());

    if (!canSwitchScene) ImGui::BeginDisabled();
    if (ImGui::Button("Load Scene"))
    {
        std::string path = std::string(kScenesFolder) + m_sceneFiles[m_sceneSelectedIdx];
        m_sceneLoadFailed = !SwitchScene(path);
    }
    if (!canSwitchScene) ImGui::EndDisabled();
    ImGui::SameLine();
    if (ImGui::Button("Refresh##scenes"))
        ScanScenesFolder();
    if (m_sceneLoadFailed)
        ImGui::TextColored(ImVec4(1.0f, 0.4f, 0.4f, 1.0f), "Failed to load scene, see console");

    ImGui::Separator();

    ImGui::Text("Lights");
    for (int i = 0; i < m_lightCount; ++i)
    {
        ImGui::PushID(i);
        bool isSelected = (m_selection.kind == SelectionKind::Light && m_selection.index == i);
        char label[16];
        snprintf(label, sizeof(label), "Light %d", i);
        if (ImGui::Selectable(label, isSelected))
            m_selection = isSelected ? Selection{} : Selection{SelectionKind::Light, i};
        ImGui::PopID();
    }

    ImGui::Spacing();
    ImGui::Text("Objects");
    for (size_t i = 0; i < m_activeScene.objects.size(); ++i)
    {
        ImGui::PushID(static_cast<int>(i));
        bool isSelected = (m_selection.kind == SelectionKind::Object &&
                           m_selection.index == static_cast<int>(i));
        if (ImGui::Selectable(m_activeScene.objects[i].name.c_str(), isSelected))
            m_selection = isSelected ? Selection{}
                                    : Selection{SelectionKind::Object, static_cast<int>(i)};
        ImGui::PopID();
    }

    ImGui::End();
}

void Renderer::DrawInspectorPanel()
{
    if (!m_ready)
        return;

    ImGui::Begin("Inspector");

    if (m_selection.kind == SelectionKind::Light && m_selection.index >= 0 &&
        m_selection.index < m_lightCount)
    {
        int i = m_selection.index;
        ImGui::Text("Light %d", i);
        ImGui::Checkbox("Enabled", &m_lightEnabled[i]);
        ImGui::DragFloat("Intensity", &m_lightIntensity[i], 0.05f, 0.0f, 50.0f);
        ImGui::DragFloat3("Position", &m_lights[i].position.x, 0.05f);
        ImGui::ColorEdit3("Color", &m_lights[i].color.x);
        ImGui::DragFloat("Range", &m_lights[i].range, 0.05f, 0.1f, 20.0f);
    }
    else if (m_selection.kind == SelectionKind::Object && m_selection.index >= 0 &&
             m_selection.index < static_cast<int>(m_activeScene.objects.size()))
    {
        SceneObject& obj = m_activeScene.objects[m_selection.index];
        ImGui::Text("%s", obj.name.c_str());
        ImGui::Checkbox("Visible", &obj.visible);

        ImGui::Text("Gizmo:");
        ImGui::SameLine();
        if (ImGui::RadioButton("Move", m_gizmoOperation == ImGuizmo::TRANSLATE))
            m_gizmoOperation = ImGuizmo::TRANSLATE;
        ImGui::SameLine();
        if (ImGui::RadioButton("Rotate", m_gizmoOperation == ImGuizmo::ROTATE))
            m_gizmoOperation = ImGuizmo::ROTATE;
        ImGui::SameLine();
        if (ImGui::RadioButton("Scale", m_gizmoOperation == ImGuizmo::SCALE))
            m_gizmoOperation = ImGuizmo::SCALE;

        ImGui::DragFloat3("Position", &obj.position.x, 0.05f);
        ImGui::DragFloat3("Rotation (deg)", &obj.rotationEulerDegrees.x, 0.5f);
        ImGui::DragFloat3("Scale", &obj.scale.x, 0.01f, 0.001f, 100.0f, "%.3f",
                         ImGuiSliderFlags_AlwaysClamp);
        ImGui::Separator();
        ImGui::Text("Material");
        const std::vector<Geometry::SubmeshRange>* submeshes = ResolveSubmeshes(obj.meshRef);
        bool hasTexturedSubmesh =
            submeshes && std::any_of(submeshes->begin(), submeshes->end(),
                                     [](const auto& p) { return !p.albedoTexture.empty(); });
        if (obj.role == ObjectRole::Cornell)
            ImGui::TextDisabled("Kd: per-wall, baked into geometry");
        else if (hasTexturedSubmesh)
            ImGui::TextDisabled("Kd: from imported texture (untextured parts use Kd below)");
        else
            ImGui::ColorEdit3("Kd (albedo)", &obj.material.kd.x);
        ImGui::ColorEdit3("Ks (F0)", &obj.material.ks.x);
        ImGui::DragFloat("Alpha (roughness)", &obj.material.alpha, 1.0f, 1.0f, 256.0f);
    }
    else
    {
        ImGui::TextDisabled("Nothing selected");
    }

    ImGui::End();
}

void Renderer::DrawRenderSettingsPanel()
{
    if (!m_ready)
        return;

    ImGui::Begin("Render Settings");

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
                               "Specular IBL only",
                               "AO Raw",
                               "AO Blur H",
                               "AO Blur V",
                               "Cel Outline Mask"};

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

    if (ImGui::CollapsingHeader("Cel Shading", ImGuiTreeNodeFlags_DefaultOpen))
    {
        ImGui::Checkbox("Cel Shading Enabled", &m_celEnabled);

        ImGui::BeginDisabled(!m_celEnabled);

        ImGui::Checkbox("Toon Shading", &m_toonEnabled);
        ImGui::BeginDisabled(!m_toonEnabled);
        ImGui::SliderInt("Toon Bands", &m_toonBands, 1, 8);
        ImGui::EndDisabled();

        ImGui::Spacing();
        ImGui::Text("Outline");
        ImGui::SliderFloat("Thickness",         &m_outlineThickness,  0.5f, 4.0f);
        ImGui::SliderFloat("Depth Threshold",   &m_depthThreshold,    0.001f, 0.5f, "%.3f");
        ImGui::SliderFloat("Normal Threshold",  &m_normalThreshold,   0.01f,  1.0f, "%.3f");
        ImGui::ColorEdit3("Outline Color",      &m_outlineColor.x);

        ImGui::EndDisabled();
    }

    ImGui::Separator();

    if (ImGui::CollapsingHeader("Ambient Occlusion", ImGuiTreeNodeFlags_DefaultOpen))
    {
        ImGui::Checkbox("Enabled##AO", &m_aoEnabled);
        ImGui::SliderFloat("Strength",    &m_aoStrength,   0.0f,  1.0f);

        ImGui::SliderInt("Samples (n)",   &m_aoSamples,    10,    20);
        ImGui::SliderFloat("Range (R)",   &m_aoRadius,     0.1f,  3.0f);
        ImGui::SliderFloat("Scale (s)",   &m_aoScale,      0.0f,  5.0f);
        ImGui::SliderFloat("Contrast (k)",&m_aoContrast,   0.1f,  5.0f);
        ImGui::SliderFloat("Depth bias",  &m_aoDelta,      0.0f,  0.01f, "%.4f");

        ImGui::Spacing();
        ImGui::Text("Bilateral Blur");
        ImGui::SliderInt("Blur radius",   &m_aoBlurRadius, 1,     16);
        ImGui::SliderFloat("Depth sigma", &m_aoDepthSigma, 0.001f, 0.1f, "%.3f");
    }

    ImGui::Separator();

    ImGui::Text("Lights");
    ImGui::Checkbox("Show Light Gizmos", &m_showLightGizmos);
    ImGui::Checkbox("Use Light Volumes (direct lighting)", &m_useLightVolumes);
    if (ImGui::IsItemHovered())
        ImGui::SetTooltip(
            "Direct lighting technique for PBS mode:\n"
            "Off = fullscreen \"many lights\" loop (default)\n"
            "On  = additive light-volume geometry per light\n"
            "Mutually exclusive - ignored in IBL mode.");
    ImGui::SliderInt("Light Count", &m_lightCount, 1, kMaxLights);

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
    ImGui::Text("Tone Mapping");
    ImGui::DragFloat("Exposure", &m_exposure, 0.05f, 0.001f, 10000.0f, "%.3f");
    if (m_hdriTex.ID() == 0)
        ImGui::DragFloat("Ambient", &m_ambient, 0.001f, 0.0f, 1.0f);

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
        if (LoadHDRI(path))
            m_lightingMode = LightingMode::IBL;
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

    ImGui::End();
}

void Renderer::DrawDebugUI()
{
    if (!m_ready)
        return;

    DrawScenePanel();
    DrawInspectorPanel();
    DrawRenderSettingsPanel();
    DrawAnimationPanel(m_animator, m_skeleton, m_animationClips, m_animSelectedClipIndex,
                       m_animSelectedBoneIndex);

    {
        glm::mat4 skeletalModel =
            (m_skeletalObjectIndex >= 0)
                ? ComputeModelMatrix(m_activeScene.objects[m_skeletalObjectIndex])
                : glm::mat4(1.0f);
        HandleBoneHover(skeletalModel, m_skeleton, m_animator.worldPose);
    }

    // ---- Gizmo ---------------------------------------------------------------
    // Draw directly into the foreground draw list — no overlay window needed.
    // ImGuizmo handles its own mouse hit-testing through ImGui IO, so
    // WantCaptureMouse stays false when the mouse isn't over a gizmo handle.
    // Lights are translate-only (no rotation/scale concept); objects get the
    // full Move/Rotate/Scale toggle set in the Inspector.
    if (m_selection.kind == SelectionKind::Light && m_selection.index >= 0 &&
        m_selection.index < m_lightCount)
    {
        ImGuiIO& io = ImGui::GetIO();
        ImGuizmo::SetDrawlist(ImGui::GetForegroundDrawList());
        ImGuizmo::SetRect(0, 0, io.DisplaySize.x, io.DisplaySize.y);

        glm::mat4 model = glm::translate(glm::mat4(1.0f), m_lights[m_selection.index].position);
        ImGuizmo::Manipulate(
            glm::value_ptr(m_cachedView),
            glm::value_ptr(m_cachedProj),
            ImGuizmo::TRANSLATE,
            ImGuizmo::WORLD,
            glm::value_ptr(model));

        if (ImGuizmo::IsUsing())
            m_lights[m_selection.index].position = glm::vec3(model[3]);
    }
    else if (m_selection.kind == SelectionKind::Object && m_selection.index >= 0 &&
             m_selection.index < static_cast<int>(m_activeScene.objects.size()))
    {
        ImGuiIO& io = ImGui::GetIO();
        ImGuizmo::SetDrawlist(ImGui::GetForegroundDrawList());
        ImGuizmo::SetRect(0, 0, io.DisplaySize.x, io.DisplaySize.y);

        SceneObject& obj = m_activeScene.objects[m_selection.index];
        glm::mat4 model = ComputeModelMatrix(obj);

        ImGuizmo::Manipulate(
            glm::value_ptr(m_cachedView),
            glm::value_ptr(m_cachedProj),
            m_gizmoOperation,
            ImGuizmo::WORLD,
            glm::value_ptr(model));

        if (ImGuizmo::IsUsing())
        {
            float t[3], r[3], s[3];
            ImGuizmo::DecomposeMatrixToComponents(glm::value_ptr(model), t, r, s);
            obj.position = glm::make_vec3(t);
            obj.rotationEulerDegrees = glm::make_vec3(r);
            obj.scale = glm::make_vec3(s);
        }
    }

    // ---- Click-to-select / click-away-to-deselect ---------------------------
    // On a left-click that ImGui and ImGuizmo are not consuming: try lights first
    // (screen-space nearest-icon pick), then fall back to a world-space raycast
    // against scene object geometry. A click that hits neither clears selection.
    {
        ImGuiIO& io = ImGui::GetIO();
        if (ImGui::IsMouseClicked(ImGuiMouseButton_Left)
            && !io.WantCaptureMouse
            && !ImGuizmo::IsOver()
            && !ImGuizmo::IsUsing())
        {
            ImVec2 mp = ImGui::GetMousePos();
            int lightHit = -1;

            if (m_showLightGizmos)
            {
                constexpr float kHitRadiusPx = 24.0f;
                float bestDistSq = kHitRadiusPx * kHitRadiusPx;

                int count = std::clamp(m_lightCount, 0, kMaxLights);
                for (int i = 0; i < count; ++i)
                {
                    if (!m_lightEnabled[i])
                        continue;

                    // Project world position → NDC → screen pixels.
                    glm::vec4 clip = m_cachedProj * m_cachedView
                                     * glm::vec4(m_lights[i].position, 1.0f);
                    if (clip.w <= 0.0f)
                        continue;   // behind the camera

                    glm::vec3 ndc = glm::vec3(clip) / clip.w;
                    float sx = (ndc.x * 0.5f + 0.5f) * io.DisplaySize.x;
                    float sy = (1.0f - (ndc.y * 0.5f + 0.5f)) * io.DisplaySize.y;

                    float dx = mp.x - sx;
                    float dy = mp.y - sy;
                    float dSq = dx * dx + dy * dy;
                    if (dSq < bestDistSq)
                    {
                        bestDistSq = dSq;
                        lightHit = i;
                    }
                }
            }

            if (lightHit >= 0)
            {
                m_selection = {SelectionKind::Light, lightHit};
            }
            else
            {
                float ndcX = (mp.x / io.DisplaySize.x) * 2.0f - 1.0f;
                float ndcY = 1.0f - (mp.y / io.DisplaySize.y) * 2.0f;

                glm::mat4 invVP = glm::inverse(m_cachedProj * m_cachedView);
                glm::vec4 nearP = invVP * glm::vec4(ndcX, ndcY, -1.0f, 1.0f);
                glm::vec4 farP = invVP * glm::vec4(ndcX, ndcY, 1.0f, 1.0f);
                nearP /= nearP.w;
                farP /= farP.w;

                glm::vec3 rayOrigin = glm::vec3(nearP);
                glm::vec3 rayDir = glm::normalize(glm::vec3(farP - nearP));

                int objHit = -1;
                m_selection = RaycastScene(m_activeScene, m_raycastMeshes, rayOrigin, rayDir, objHit)
                                  ? Selection{SelectionKind::Object, objHit}
                                  : Selection{};
            }
        }
    }
}

void Renderer::HandleBoneHover(const glm::mat4& modelMatrix, const Anim::Skeleton& skeleton,
                               const std::vector<Anim::VQS>& worldPose) const
{
    if (!m_showSkeleton)
        return;

    ImGuiIO& io = ImGui::GetIO();
    if (io.WantCaptureMouse)
        return;

    ImVec2 mp = ImGui::GetMousePos();

    constexpr float kHitRadiusPx = 10.0f;
    float bestDistSq = kHitRadiusPx * kHitRadiusPx;
    int hoveredBone = -1;

    for (size_t i = 0; i < skeleton.bones.size() && i < worldPose.size(); ++i)
    {
        const Anim::Vec3& jointPos = worldPose[i].v;
        glm::vec4 worldPos4 =
            modelMatrix * glm::vec4(jointPos.x, jointPos.y, jointPos.z, 1.0f);
        glm::vec4 clip = m_cachedProj * m_cachedView * worldPos4;
        if (clip.w <= 0.0f)
            continue;  // behind the camera

        glm::vec3 ndc = glm::vec3(clip) / clip.w;
        float sx = (ndc.x * 0.5f + 0.5f) * io.DisplaySize.x;
        float sy = (1.0f - (ndc.y * 0.5f + 0.5f)) * io.DisplaySize.y;

        float dx = mp.x - sx;
        float dy = mp.y - sy;
        float dSq = dx * dx + dy * dy;
        if (dSq < bestDistSq)
        {
            bestDistSq = dSq;
            hoveredBone = static_cast<int>(i);
        }
    }

    if (hoveredBone >= 0)
        ImGui::SetTooltip("%s", skeleton.bones[hoveredBone].name.c_str());
}

void Renderer::DrawAnimationPanel(Anim::Animator& animator, const Anim::Skeleton& skeleton,
                                  const std::vector<Anim::AnimationClip>& clips,
                                  int& selectedClipIndex, int& selectedBoneIndex)
{
    if (!m_ready)
        return;

    ImGui::Begin("Animation");

    if (clips.empty() || skeleton.bones.empty())
    {
        ImGui::TextDisabled("No animation data loaded.");
        ImGui::End();
        return;
    }

    selectedClipIndex = std::clamp(selectedClipIndex, 0, static_cast<int>(clips.size()) - 1);
    selectedBoneIndex =
        std::clamp(selectedBoneIndex, 0, static_cast<int>(skeleton.bones.size()) - 1);

    // Clip picker
    {
        std::vector<const char*> items;
        items.reserve(clips.size());
        for (const auto& clip : clips) items.push_back(clip.name.c_str());

        if (ImGui::Combo(
                "Clip", &selectedClipIndex, items.data(), static_cast<int>(items.size())))
            Anim::SetClip(animator, skeleton, clips[selectedClipIndex]);
    }

    ImGui::Checkbox("Show Mesh", &m_showMesh);
    ImGui::SameLine();
    ImGui::Checkbox("Show Skeleton", &m_showSkeleton);

    // Interpolation mode
    {
        static const char* kModeItems[] = {"Lerp", "Slerp", "ELerp", "iSlerp", "iVQS"};
        int mode = static_cast<int>(animator.mode);
        if (ImGui::Combo("Interpolation", &mode, kModeItems, IM_ARRAYSIZE(kModeItems)))
            animator.mode = static_cast<Anim::InterpolationMode>(mode);

        if (animator.mode == Anim::InterpolationMode::ISlerp ||
            animator.mode == Anim::InterpolationMode::IVQS)
            ImGui::SliderInt("Segment steps", &animator.incrementalSteps, 2, 64);
    }

    ImGui::Separator();

    // Playback
    if (ImGui::Button(animator.playing ? "Pause" : "Play"))
        animator.playing = !animator.playing;
    ImGui::SameLine();
    ImGui::Checkbox("Loop", &animator.looping);
    ImGui::SameLine();
    ImGui::Text("%.2fs / %.2fs", Anim::TimeSeconds(animator), Anim::DurationSeconds(animator));

    // Bone picker for timeline
    {
        std::vector<const char*> items;
        items.reserve(skeleton.bones.size());
        for (const auto& bone : skeleton.bones) items.push_back(bone.name.c_str());

        ImGui::Combo("Bone", &selectedBoneIndex, items.data(), static_cast<int>(items.size()));
    }

    // Keyframe timeline
    const std::vector<Anim::Keyframe> kEmptyKeys;
    const std::vector<Anim::Keyframe>& keys =
        (selectedBoneIndex < static_cast<int>(animator.boneKeyframes.size()))
            ? animator.boneKeyframes[selectedBoneIndex]
            : kEmptyKeys;
    float duration = animator.clip ? animator.clip->duration : 0.0f;

    ImVec2 canvasPos = ImGui::GetCursorScreenPos();
    ImVec2 canvasSize(ImGui::GetContentRegionAvail().x, 40.0f);
    ImDrawList* drawList = ImGui::GetWindowDrawList();

    drawList->AddRectFilled(
        canvasPos, ImVec2(canvasPos.x + canvasSize.x, canvasPos.y + canvasSize.y),
        IM_COL32(30, 30, 30, 255));

    if (duration > 1e-4f)
    {
        for (const Anim::Keyframe& kf : keys)
        {
            float u = std::clamp(kf.time / duration, 0.0f, 1.0f);
            float x = canvasPos.x + u * canvasSize.x;
            drawList->AddLine(ImVec2(x, canvasPos.y),
                              ImVec2(x, canvasPos.y + canvasSize.y),
                              IM_COL32(210, 190, 80, 255),
                              2.0f);
        }

        float playheadU = std::clamp(animator.timeTicks / duration, 0.0f, 1.0f);
        float playheadX = canvasPos.x + playheadU * canvasSize.x;
        drawList->AddLine(ImVec2(playheadX, canvasPos.y),
                          ImVec2(playheadX, canvasPos.y + canvasSize.y),
                          IM_COL32(255, 70, 70, 255),
                          2.0f);
    }

    ImGui::InvisibleButton("##keyframe_timeline", canvasSize);
    if (ImGui::IsItemActive() && duration > 1e-4f)
    {
        animator.playing = false;
        float u =
            std::clamp((ImGui::GetIO().MousePos.x - canvasPos.x) / canvasSize.x, 0.0f, 1.0f);
        Anim::SeekTo(animator, u * duration);
    }

    ImGui::TextDisabled("%d keyframes on this bone", static_cast<int>(keys.size()));

    ImGui::End();
}
