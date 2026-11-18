#include "pch.h"
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdio>
#include <utility>
#include <vector>
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
        for (const auto& f : m_sceneFiles)
            items.push_back(f.c_str());

        if (items.empty())
        {
            ImGui::TextDisabled("No .json files found in %s", kScenesFolder);
        }
        else
        {
            ImGui::SetNextItemWidth(-1.0f);
            ImGui::Combo("##scene_pick",
                         &m_sceneSelectedIdx,
                         items.data(),
                         static_cast<int>(items.size()));
        }
    }

    const bool canSwitchScene =
        m_sceneSelectedIdx >= 0 && m_sceneSelectedIdx < static_cast<int>(m_sceneFiles.size());

    if (!canSwitchScene)
        ImGui::BeginDisabled();
    if (ImGui::Button("Load Scene"))
    {
        std::string path = std::string(kScenesFolder) + m_sceneFiles[m_sceneSelectedIdx];
        m_sceneLoadFailed = !SwitchScene(path);
    }
    if (!canSwitchScene)
        ImGui::EndDisabled();
    ImGui::SameLine();
    if (ImGui::Button("Refresh##scenes"))
        ScanScenesFolder();
    if (m_sceneLoadFailed)
        ImGui::TextColored(ImVec4(1.0f, 0.4f, 0.4f, 1.0f), "Failed to load scene, see console");
    if (!m_pathMotionStatus.empty())
    {
        ImVec4 color =
            m_pathMotionActive ? ImVec4(0.5f, 0.9f, 0.5f, 1.0f) : ImVec4(1.0f, 0.4f, 0.4f, 1.0f);
        ImGui::PushTextWrapPos();
        ImGui::TextColored(color, "%s", m_pathMotionStatus.c_str());
        ImGui::PopTextWrapPos();
    }

    ImGui::Separator();

    ImGui::SeparatorText("Lights");
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
    ImGui::SeparatorText("Objects");
    for (size_t i = 0; i < m_activeScene.objects.size(); ++i)
    {
        ImGui::PushID(static_cast<int>(i));
        bool isSelected =
            (m_selection.kind == SelectionKind::Object && m_selection.index == static_cast<int>(i));
        if (ImGui::Selectable(m_activeScene.objects[i].name.c_str(), isSelected))
            m_selection =
                isSelected ? Selection{} : Selection{SelectionKind::Object, static_cast<int>(i)};
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

        if (m_pathMotionActive && m_selection.index == m_skeletalObjectIndex)
            ImGui::TextDisabled("Path owns placement; TRS below is a local offset");
        ImGui::DragFloat3("Position", &obj.position.x, 0.05f);
        ImGui::DragFloat3("Rotation (deg)", &obj.rotationEulerDegrees.x, 0.5f);
        ImGui::DragFloat3("Scale",
                          &obj.scale.x,
                          0.01f,
                          0.001f,
                          100.0f,
                          "%.3f",
                          ImGuiSliderFlags_AlwaysClamp);
        ImGui::Separator();
        ImGui::Text("Material");
        const std::vector<Geometry::SubmeshRange>* submeshes = ResolveSubmeshes(obj.meshRef);
        bool hasTexturedSubmesh =
            submeshes && std::any_of(submeshes->begin(),
                                     submeshes->end(),
                                     [](const auto& p) { return !p.albedoTexture.empty(); });
        if (obj.role == ObjectRole::Cornell)
            ImGui::TextDisabled("Kd: per-wall, baked into geometry");
        else if (hasTexturedSubmesh)
            ImGui::TextDisabled("Kd: from imported texture (untextured parts use Kd below)");
        else
            ImGui::ColorEdit3("Kd (albedo)", &obj.material.kd.x);
        ImGui::ColorEdit3("Ks (F0)", &obj.material.ks.x);
        ImGui::DragFloat("Alpha (roughness)", &obj.material.alpha, 1.0f, 1.0f, 256.0f);
        ImGui::DragFloat("Metallic", &obj.material.metallic, 0.01f, 0.0f, 1.0f);

        ImGui::Spacing();
        ImGui::Text("Texture Strength");
        ImGui::SliderFloat("Albedo##Strength", &obj.material.albedoStrength, 0.0f, 1.0f);
        ImGui::SliderFloat("Specular##Strength", &obj.material.specularStrength, 0.0f, 1.0f);
        ImGui::SliderFloat("Roughness##Strength", &obj.material.roughnessStrength, 0.0f, 1.0f);
        ImGui::SliderFloat("Metallic##Strength", &obj.material.metallicStrength, 0.0f, 1.0f);
        ImGui::SliderFloat("Normal##Strength", &obj.material.normalStrength, 0.0f, 1.0f);
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
                               "Cel Outline Mask",
                               "Metallic",
                               "IBL Shadow Occlusion"};

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
        ImGui::SliderFloat("Thickness", &m_outlineThickness, 0.5f, 4.0f);
        ImGui::SliderFloat("Depth Threshold", &m_depthThreshold, 0.001f, 0.5f, "%.3f");
        ImGui::SliderFloat("Normal Threshold", &m_normalThreshold, 0.01f, 1.0f, "%.3f");
        ImGui::ColorEdit3("Outline Color", &m_outlineColor.x);

        ImGui::EndDisabled();
    }

    ImGui::Separator();

    if (ImGui::CollapsingHeader("Ambient Occlusion", ImGuiTreeNodeFlags_DefaultOpen))
    {
        ImGui::Checkbox("Enabled##AO", &m_aoEnabled);
        ImGui::SliderFloat("Strength", &m_aoStrength, 0.0f, 1.0f);

        ImGui::SliderInt("Samples (n)", &m_aoSamples, 10, 20);
        ImGui::SliderFloat("Range (R)", &m_aoRadius, 0.1f, 3.0f);
        ImGui::SliderFloat("Scale (s)", &m_aoScale, 0.0f, 5.0f);
        ImGui::SliderFloat("Contrast (k)", &m_aoContrast, 0.1f, 5.0f);
        ImGui::SliderFloat("Depth bias", &m_aoDelta, 0.0f, 0.01f, "%.4f");

        ImGui::Spacing();
        ImGui::Text("Bilateral Blur");
        ImGui::SliderInt("Blur radius", &m_aoBlurRadius, 1, 16);
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
        ImGui::SliderFloat("Shadow Strength", &m_shadowStrength, 0.0f, 1.0f);
        if (ImGui::IsItemHovered())
            ImGui::SetTooltip(
                "Global multiplier on every shadow in the scene - direct-light shadows\n"
                "(PBS and IBL) and the IBL ambient-occlusion term alike.\n"
                "0 = no shadowing anywhere, 1 = shadow factors at full strength.");

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

        if (m_hasHDRISun)
        {
            ImGui::DragFloat("Sun Shadow Bias", &m_sunShadowBias, 0.0005f, 0.0f, 0.05f, "%.4f");
            if (ImGui::IsItemHovered())
                ImGui::SetTooltip(
                    "Bias for the directional shadow approximating the HDRI's dominant\n"
                    "light direction - occludes IBL ambient independently of the point lights.");
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
        for (const auto& f : m_hdriFiles)
            items.push_back(f.c_str());

        if (items.empty())
        {
            ImGui::TextDisabled("No .hdr files found in %s", kHDRIFolder);
        }
        else
        {
            ImGui::SetNextItemWidth(-1.0f);
            ImGui::Combo("##hdri_pick",
                         &m_hdriSelectedIdx,
                         items.data(),
                         static_cast<int>(items.size()));
        }
    }

    const bool canLoad =
        m_hdriSelectedIdx >= 0 && m_hdriSelectedIdx < static_cast<int>(m_hdriFiles.size());

    if (!canLoad)
        ImGui::BeginDisabled();
    if (ImGui::Button("Load HDRI"))
    {
        std::string path = std::string(kHDRIFolder) + m_hdriFiles[m_hdriSelectedIdx];
        if (LoadHDRI(path))
            m_lightingMode = LightingMode::IBL;
    }
    if (!canLoad)
        ImGui::EndDisabled();
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

void Renderer::DrawProfilerPanel()
{
    if (!m_ready)
        return;

    ImGui::Begin("Profiler");

    ImGuiIO& io = ImGui::GetIO();
    m_fpsHistory[m_fpsHistoryOffset] = io.Framerate;
    m_fpsHistoryOffset = (m_fpsHistoryOffset + 1) % kFpsHistoryCount;

    ImGui::Text("%.1f FPS (%.3f ms/frame)", io.Framerate, 1000.0f / io.Framerate);
    ImGui::PlotLines("##fps",
                     m_fpsHistory,
                     kFpsHistoryCount,
                     m_fpsHistoryOffset,
                     nullptr,
                     0.0f,
                     144.0f,
                     ImVec2(0, 60));

    ImGui::End();
}

static const Fluid::GpuTimer::Stage* FindStage(const std::vector<Fluid::GpuTimer::Stage>& stages, const char* name)
{
    for (const Fluid::GpuTimer::Stage& stage : stages)
        if (stage.valid && stage.name == name)
            return &stage;
    return nullptr;
}

static void StageCell(const Fluid::GpuTimer::Stage* stage)
{
    ImGui::TableNextColumn();
    if (stage)
        ImGui::Text("%.3f", stage->ms);
    else
        ImGui::TextDisabled("-");
}

static double Megabytes(size_t bytes)
{
    return static_cast<double>(bytes) / (1024.0 * 1024.0);
}

void Renderer::DrawFluidPanel()
{
    if (!m_ready)
        return;

    ImGui::Begin("Fluid");
    if (!m_fluid.IsDeviceReady())
    {
        ImGui::TextDisabled("CUDA unavailable: %s", m_fluid.Status().c_str());
        ImGui::End();
        return;
    }

    const Fluid::CudaDeviceInfo& device = m_fluid.Device();
    ImGui::Text("%s, sm_%d%d, %d SMs, %.1f GB",
                device.name.c_str(),
                device.ccMajor,
                device.ccMinor,
                device.smCount,
                static_cast<double>(device.totalMemoryBytes) / (1024.0 * 1024.0 * 1024.0));

    if (!m_fluid.IsLoaded())
    {
        ImGui::TextDisabled("%s",
                            m_fluid.Status().empty() ? "No fluid binding in this scene"
                                                     : m_fluid.Status().c_str());
        ImGui::End();
        return;
    }

    if (ImGui::Button(m_fluid.playing ? "Pause" : "Play"))
        m_fluid.playing = !m_fluid.playing;
    ImGui::SameLine();
    ImGui::BeginDisabled(m_fluid.playing);
    if (ImGui::Button("Step"))
        m_fluid.stepRequested = true;
    ImGui::EndDisabled();
    ImGui::SameLine();
    if (ImGui::Button("Reset"))
    {
        if (m_fluid.playbackMode)
            m_fluid.SetPlaybackFrame(0);
        else
            m_fluid.Reset();
    }

    ImGui::SeparatorText("Bake");
    if (m_fluid.IsBaking())
    {
        char progress[64];
        std::snprintf(progress,
                      sizeof(progress),
                      "%d / %d frames",
                      m_fluid.BakedFrames(),
                      m_fluid.BakeTarget());
        ImGui::ProgressBar(static_cast<float>(m_fluid.BakedFrames()) /
                               static_cast<float>(m_fluid.BakeTarget()),
                           ImVec2(-1.0f, 0.0f),
                           progress);
        if (ImGui::Button("Stop bake"))
            m_fluid.StopBake();
    }
    else
    {
        ImGui::SliderInt("Frames", &m_fluidBakeFrames, 60, 3600);
        const double estimate = 6.0 * static_cast<double>(m_fluid.MaxParticles()) * m_fluidBakeFrames;
        ImGui::TextDisabled("%.1f s of simulation, up to %.0f MB",
                            m_fluidBakeFrames / 60.0,
                            estimate / (1024.0 * 1024.0));
        if (ImGui::Button("Bake"))
            m_fluid.StartBake(m_fluidBakeFrames);
        ImGui::SameLine();
        ImGui::BeginDisabled(m_fluid.BakedFrames() == 0);
        if (ImGui::Button("Clear bake"))
            m_fluid.ClearBake();
        ImGui::SameLine();
        if (ImGui::Button("Save"))
            m_fluid.SaveBake();
        ImGui::EndDisabled();
        ImGui::SameLine();
        ImGui::BeginDisabled(!m_fluid.HasSavedBake());
        if (ImGui::Button("Load saved"))
            m_fluid.LoadBake();
        ImGui::EndDisabled();
        ImGui::TextDisabled("%s%s",
                            m_fluid.BakeFile().c_str(),
                            m_fluid.HasSavedBake() ? "" : " (nothing saved yet)");
    }
    if (!m_fluid.BakeStatus().empty())
        ImGui::TextDisabled("%s", m_fluid.BakeStatus().c_str());
    if (m_fluid.BakedFrames() > 0 && !m_fluid.IsBaking())
    {
        ImGui::Checkbox("Play baked", &m_fluid.playbackMode);
        ImGui::SameLine();
        ImGui::Checkbox("Loop", &m_fluid.loopPlayback);
        ImGui::BeginDisabled(!m_fluid.playbackMode);
        int frame = m_fluid.PlaybackFrame();
        if (ImGui::SliderInt("Frame", &frame, 0, m_fluid.BakedFrames() - 1))
            m_fluid.SetPlaybackFrame(frame);
        ImGui::EndDisabled();
        ImGui::Text("%d frames baked, %.1f MB", m_fluid.BakedFrames(), Megabytes(m_fluid.BakeBytes()));
    }

    ImGui::SeparatorText("Simulation");
    int backend = static_cast<int>(m_fluid.backend);
    ImGui::TextUnformatted("Backend");
    ImGui::SameLine();
    ImGui::BeginDisabled(!m_fluid.DenseAvailable());
    ImGui::RadioButton("Dense", &backend, static_cast<int>(Fluid::Backend::Dense));
    ImGui::EndDisabled();
    ImGui::SameLine();
    ImGui::RadioButton("Sparse", &backend, static_cast<int>(Fluid::Backend::Sparse));
    m_fluid.backend = static_cast<Fluid::Backend>(backend);
    if (!m_fluid.DenseAvailable())
    {
        ImGui::SameLine();
        ImGui::TextDisabled("(domain too large for dense)");
    }

    const Fluid::GridInfo& grid = m_fluid.Grid();
    ImGui::Text("%d particles, domain %d x %d x %d cells, cell %.4f m",
                m_fluid.playbackMode ? m_fluid.RenderCount() : m_fluid.ParticleCount(),
                grid.dims.x,
                grid.dims.y,
                grid.dims.z,
                grid.h);
    if (m_fluid.EmitterCount() > 0)
        ImGui::Text("%d emitters, particle cap %d", m_fluid.EmitterCount(), m_fluid.MaxParticles());

    const Fluid::StepStats& stats = m_fluid.LastStats();
    ImGui::Text("%d substeps, max speed %.2f m/s", stats.substeps, stats.maxSpeed);
    ImGui::Text("CG %d iterations this frame (last solve %d), residual %.1e%s",
                stats.cgIterationsTotal,
                stats.cg.iterations,
                stats.cg.relativeResidual,
                stats.cg.converged || stats.substeps == 0 ? "" : ", not converged");

    ImGui::SeparatorText("Sparse topology");
    const Fluid::SparseTopology& topology = m_fluid.Topology();
    ImGui::Checkbox("Incremental rebuild", &m_fluid.incrementalTopology);
    ImGui::SameLine();
    ImGui::TextDisabled("last: %s", topology.LastWasFull() ? "full" : "incremental");
    ImGui::Text("Root level %d, %d nodes (%d bricks, %d interior) in %d slots",
                topology.RootLevel(),
                topology.NodeCount(),
                topology.LeafCount(),
                topology.InteriorCount(),
                topology.NodeSlots());
    ImGui::Text("Particles with uncovered influence corners: %d", m_fluid.Uncovered());
    const Fluid::BrickAtlas& atlas = m_fluid.Atlas();
    const int3 atlasDims = atlas.TexelDims();
    ImGui::Text("Atlas: %d bricks, %d x %d x %d texels, %.1f MB",
                atlas.Capacity(),
                atlasDims.x,
                atlasDims.y,
                atlasDims.z,
                static_cast<double>(atlas.MemoryBytes()) / (1024.0 * 1024.0));
    const Fluid::SubcellLists& subcells = m_fluid.Subcells();
    ImGui::Text("Subcells: %d, %d entries (%.2f per particle), %.1f MB",
                subcells.SubcellCount(),
                subcells.EntryCount(),
                m_fluid.ParticleCount() > 0
                    ? static_cast<double>(subcells.EntryCount()) / m_fluid.ParticleCount()
                    : 0.0,
                static_cast<double>(subcells.MemoryBytes()) / (1024.0 * 1024.0));
    ImGui::Checkbox("Show nodes", &m_showFluidBricks);
    ImGui::SameLine();
    ImGui::BeginDisabled(!m_showFluidBricks);
    ImGui::Checkbox("Bricks", &m_showFluidLeaves);
    ImGui::SameLine();
    ImGui::Checkbox("Interior", &m_showFluidInterior);
    ImGui::EndDisabled();

    ImGui::SeparatorText("Rendering");
    int view = static_cast<int>(m_fluidView);
    ImGui::RadioButton("Surface", &view, static_cast<int>(FluidView::Surface));
    ImGui::SameLine();
    ImGui::RadioButton("Particles", &view, static_cast<int>(FluidView::Particles));
    m_fluidView = static_cast<FluidView>(view);
    ImGui::BeginDisabled(m_fluidView != FluidView::Surface);
    ImGui::SliderFloat("Absorption (1/m)", &m_fluidAbsorption, 0.0f, 50.0f);
    ImGui::SliderFloat("Refraction", &m_fluidRefraction, 0.0f, 0.1f);
    ImGui::EndDisabled();
    if (m_fluidView == FluidView::Surface && !m_fluid.SurfaceReady())
        ImGui::TextDisabled("Level set not available this frame");

    ImGui::SeparatorText("Solver");
    Fluid::StepSettings& settings = m_fluid.settings;
    ImGui::SliderFloat("FLIP ratio", &settings.flipRatio, 0.0f, 1.0f);
    ImGui::SliderFloat("CG tolerance",
                       &settings.cgTolerance,
                       1e-7f,
                       1e-2f,
                       "%.1e",
                       ImGuiSliderFlags_Logarithmic);
    ImGui::SliderInt("CG max iterations", &settings.cgMaxIterations, 1, 2000);

    ImGui::SeparatorText("Benchmarks");
    const ImGuiTableFlags tableFlags = ImGuiTableFlags_RowBg | ImGuiTableFlags_BordersInnerV;
    const std::vector<Fluid::GpuTimer::Stage>& stages = m_fluid.Timer().Stages();

    if (ImGui::CollapsingHeader("Per-stage GPU times (Table 3)", ImGuiTreeNodeFlags_DefaultOpen))
    {
        const float total = m_fluid.Timer().TotalMs();
        if (ImGui::BeginTable("##fluidStages", 3, tableFlags))
        {
            ImGui::TableSetupColumn("Stage (last substep)");
            ImGui::TableSetupColumn("ms");
            ImGui::TableSetupColumn("%");
            ImGui::TableHeadersRow();
            for (const Fluid::GpuTimer::Stage& stage : stages)
            {
                ImGui::TableNextRow();
                ImGui::TableNextColumn();
                ImGui::TextUnformatted(stage.name.c_str());
                StageCell(stage.valid ? &stage : nullptr);
                ImGui::TableNextColumn();
                if (stage.valid && total > 0.0f)
                    ImGui::Text("%.1f", 100.0f * stage.ms / total);
                else
                    ImGui::TextDisabled("-");
            }
            ImGui::TableNextRow();
            ImGui::TableNextColumn();
            ImGui::TextUnformatted("Total");
            ImGui::TableNextColumn();
            ImGui::Text("%.3f", total);
            ImGui::TableNextColumn();
            ImGui::EndTable();
        }
    }

    if (ImGui::CollapsingHeader("Topology rebuild, full vs incremental (Table 1)"))
    {
        const Fluid::GpuTimer::Stage* full = FindStage(stages, "Topology (full)");
        const Fluid::GpuTimer::Stage* incremental = FindStage(stages, "Topology (incremental)");
        if (ImGui::BeginTable("##fluidTopology", 3, tableFlags))
        {
            ImGui::TableSetupColumn("Path");
            ImGui::TableSetupColumn("ms");
            ImGui::TableSetupColumn("Speedup");
            ImGui::TableHeadersRow();
            ImGui::TableNextRow();
            ImGui::TableNextColumn();
            ImGui::TextUnformatted("Full rebuild");
            StageCell(full);
            ImGui::TableNextColumn();
            ImGui::TextDisabled("1.0x");
            ImGui::TableNextRow();
            ImGui::TableNextColumn();
            ImGui::TextUnformatted("Incremental");
            StageCell(incremental);
            ImGui::TableNextColumn();
            if (full && incremental && incremental->ms > 0.0f)
                ImGui::Text("%.1fx", full->ms / incremental->ms);
            else
                ImGui::TextDisabled("-");
            ImGui::EndTable();
        }
        ImGui::TextDisabled("Untick Incremental rebuild for a few frames to time the full path.");
    }

    if (ImGui::CollapsingHeader("Pressure solve, dense vs sparse (Table 2)"))
    {
        const Fluid::BackendSnapshot* snapshots[2] = {&m_fluid.Snapshot(Fluid::Backend::Dense),
                                                      &m_fluid.Snapshot(Fluid::Backend::Sparse)};
        const char* solveStages[2] = {"Pressure solve", "Sparse CG"};
        if (ImGui::BeginTable("##fluidSolve", 3, tableFlags))
        {
            ImGui::TableSetupColumn("##metric");
            ImGui::TableSetupColumn("Dense");
            ImGui::TableSetupColumn("Sparse");
            ImGui::TableHeadersRow();

            ImGui::TableNextRow();
            ImGui::TableNextColumn();
            ImGui::TextUnformatted("Substeps per frame");
            for (const Fluid::BackendSnapshot* snapshot : snapshots)
            {
                ImGui::TableNextColumn();
                if (snapshot->valid)
                    ImGui::Text("%d", snapshot->stats.substeps);
                else
                    ImGui::TextDisabled("-");
            }

            ImGui::TableNextRow();
            ImGui::TableNextColumn();
            ImGui::TextUnformatted("CG iterations per frame");
            for (const Fluid::BackendSnapshot* snapshot : snapshots)
            {
                ImGui::TableNextColumn();
                if (snapshot->valid)
                    ImGui::Text("%d", snapshot->stats.cgIterationsTotal);
                else
                    ImGui::TextDisabled("-");
            }

            ImGui::TableNextRow();
            ImGui::TableNextColumn();
            ImGui::TextUnformatted("Solve ms (last substep)");
            for (int b = 0; b < 2; ++b)
                StageCell(snapshots[b]->valid ? FindStage(snapshots[b]->stages, solveStages[b]) : nullptr);

            ImGui::TableNextRow();
            ImGui::TableNextColumn();
            ImGui::TextUnformatted("ms per iteration");
            for (int b = 0; b < 2; ++b)
            {
                ImGui::TableNextColumn();
                const Fluid::GpuTimer::Stage* stage =
                    snapshots[b]->valid ? FindStage(snapshots[b]->stages, solveStages[b]) : nullptr;
                const int iterations = snapshots[b]->stats.cg.iterations;
                if (stage && iterations > 0)
                    ImGui::Text("%.4f", stage->ms / iterations);
                else
                    ImGui::TextDisabled("-");
            }
            ImGui::EndTable();
        }
        ImGui::TextDisabled("Switch backends to fill both columns. Dense includes divergence and");
        ImGui::TextDisabled("the velocity update; sparse is the CG alone.");
    }

    if (ImGui::CollapsingHeader("Memory (Table 4)"))
    {
        const Fluid::MemoryReport memory = m_fluid.Memory();
        const size_t sparse = memory.topology + memory.atlas + memory.subcells;
        if (ImGui::BeginTable("##fluidMemory", 2, tableFlags))
        {
            ImGui::TableSetupColumn("Buffer");
            ImGui::TableSetupColumn("MB");
            ImGui::TableHeadersRow();
            const std::pair<const char*, size_t> rows[] = {{"Particles", memory.particles},
                                                           {"Sparse topology", memory.topology},
                                                           {"Brick atlas (9 channels)", memory.atlas},
                                                           {"Subcell lists", memory.subcells},
                                                           {"Sparse total", sparse}};
            for (const auto& row : rows)
            {
                ImGui::TableNextRow();
                ImGui::TableNextColumn();
                ImGui::TextUnformatted(row.first);
                ImGui::TableNextColumn();
                ImGui::Text("%.1f", Megabytes(row.second));
            }
            ImGui::TableNextRow();
            ImGui::TableNextColumn();
            ImGui::TextUnformatted("Dense grid and solver");
            ImGui::TableNextColumn();
            if (memory.dense > 0)
                ImGui::Text("%.1f", Megabytes(memory.dense));
            else
                ImGui::TextDisabled("not allocated");
            ImGui::EndTable();
        }
        ImGui::Text("Domain %.1fM cells", static_cast<double>(memory.cells) / 1e6);
        if (memory.dense > 0 && sparse > 0)
        {
            ImGui::SameLine();
            ImGui::Text(", dense / sparse = %.2fx", static_cast<double>(memory.dense) / sparse);
        }
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
    DrawProfilerPanel();
    DrawPathMotionPanel();
    DrawIKPanel();
    DrawFluidPanel();
    DrawAnimationPanel(m_animator,
                       m_skeleton,
                       m_animationClips,
                       m_animSelectedClipIndex,
                       m_animSelectedBoneIndex);

    HandleBoneHover(GetObjectModelMatrix(m_skeletalObjectIndex), m_skeleton, m_animator.worldPose);
    DrawPathPointLabels();

    // Light/object and path-point selection are mutually exclusive, so whichever was chosen last
    // owns the single gizmo.
    if (m_selection.kind != SelectionKind::None)
        m_pathSelectedPoint = -1;

    // Gizmo: drawn into the foreground draw list, no overlay window needed. Lights are
    // translate-only; objects get the full Move/Rotate/Scale toggle set in the Inspector.
    const auto* pathPoints =
        m_pathMotionActive && !m_reachActive
            ? &m_activeScene.objects[m_skeletalObjectIndex].pathMotion.controlPointsXZ
            : nullptr;
    if (pathPoints && m_pathPointGizmo && m_pathSelectedPoint >= 0 &&
        m_pathSelectedPoint < static_cast<int>(pathPoints->size()))
    {
        ImGuiIO& io = ImGui::GetIO();
        ImGuizmo::SetDrawlist(ImGui::GetForegroundDrawList());
        ImGuizmo::SetRect(0, 0, io.DisplaySize.x, io.DisplaySize.y);

        glm::vec2 p = (*pathPoints)[m_pathSelectedPoint];
        glm::mat4 model = glm::translate(glm::mat4(1.0f), glm::vec3(p.x, 0.0f, p.y));
        ImGuizmo::Manipulate(
            glm::value_ptr(m_cachedView),
            glm::value_ptr(m_cachedProj),
            static_cast<ImGuizmo::OPERATION>(ImGuizmo::TRANSLATE_X | ImGuizmo::TRANSLATE_Z),
            ImGuizmo::WORLD,
            glm::value_ptr(model));

        if (ImGuizmo::IsUsing())
        {
            m_animator.playing = false;
            glm::vec2 moved(model[3].x, model[3].z);
            if (moved != p)
            {
                std::vector<glm::vec2> candidate = *pathPoints;
                candidate[m_pathSelectedPoint] = moved;
                std::string error;
                m_pathEditError = TryApplyPathPoints(candidate, error) ? std::string() : error;
            }
        }
    }
    else if (m_selection.kind == SelectionKind::Light && m_selection.index >= 0 &&
             m_selection.index < m_lightCount)
    {
        ImGuiIO& io = ImGui::GetIO();
        ImGuizmo::SetDrawlist(ImGui::GetForegroundDrawList());
        ImGuizmo::SetRect(0, 0, io.DisplaySize.x, io.DisplaySize.y);

        glm::mat4 model = glm::translate(glm::mat4(1.0f), m_lights[m_selection.index].position);
        ImGuizmo::Manipulate(glm::value_ptr(m_cachedView),
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
        glm::mat4 pathPose = GetPathPoseMatrix(m_selection.index);
        glm::mat4 model = pathPose * ComputeModelMatrix(obj);

        // The IK target only has a position, so it always gets the translate gizmo.
        ImGuizmo::Manipulate(
            glm::value_ptr(m_cachedView),
            glm::value_ptr(m_cachedProj),
            m_selection.index == m_ikTargetIndex ? ImGuizmo::TRANSLATE : m_gizmoOperation,
            ImGuizmo::WORLD,
            glm::value_ptr(model));

        if (ImGuizmo::IsUsing())
        {
            // Path-bound objects store TRS as an offset under the path pose, so strip it before
            // writing back.
            glm::mat4 local = glm::inverse(pathPose) * model;
            float t[3], r[3], s[3];
            ImGuizmo::DecomposeMatrixToComponents(glm::value_ptr(local), t, r, s);
            obj.position = glm::make_vec3(t);
            obj.rotationEulerDegrees = glm::make_vec3(r);
            obj.scale = glm::make_vec3(s);
        }
    }

    // Click-to-select: on an unconsumed left-click, try lights (screen-space nearest-icon pick)
    // first, then a world-space raycast; a miss clears selection.
    {
        ImGuiIO& io = ImGui::GetIO();
        // Shift+click belongs to IK target placement (UpdateIKTargetInput).
        if (ImGui::IsMouseClicked(ImGuiMouseButton_Left) && !io.WantCaptureMouse && !io.KeyShift &&
            !ImGuizmo::IsOver() && !ImGuizmo::IsUsing())
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
                    glm::vec4 clip =
                        m_cachedProj * m_cachedView * glm::vec4(m_lights[i].position, 1.0f);
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
                // Unproject the cursor at the near and far planes to build a world-space pick ray.
                glm::vec3 rayOrigin, rayDir;
                int objHit = -1;
                m_selection =
                    MouseRay(rayOrigin, rayDir) &&
                            RaycastScene(m_activeScene, m_raycastMeshes, rayOrigin, rayDir, objHit)
                        ? Selection{SelectionKind::Object, objHit}
                        : Selection{};
            }
        }
    }
}

void Renderer::HandleBoneHover(const glm::mat4& modelMatrix,
                               const Anim::Skeleton& skeleton,
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
        // worldPose is skeleton space; the model matrix places the joint in the world.
        glm::vec4 worldPos4 = modelMatrix * glm::vec4(jointPos.x, jointPos.y, jointPos.z, 1.0f);
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

void Renderer::DrawAnimationPanel(Anim::Animator& animator,
                                  const Anim::Skeleton& skeleton,
                                  const std::vector<Anim::AnimationClip>& clips,
                                  int& selectedClipIndex,
                                  int& selectedBoneIndex)
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

    // Model picker - swaps only the mesh; skeleton/clips stay as loaded.
    if (m_skeletalMeshNames.size() > 1)
    {
        std::vector<const char*> items;
        items.reserve(m_skeletalMeshNames.size());
        for (const auto& n : m_skeletalMeshNames)
            items.push_back(n.c_str());

        int newIdx = m_skeletalMeshSelectedIdx;
        if (ImGui::Combo("Model", &newIdx, items.data(), static_cast<int>(items.size())))
        {
            int prevIdx = m_skeletalMeshSelectedIdx;
            m_skeletalMeshSelectedIdx = newIdx;
            if (!SwapSkeletalMesh(m_skeletalMeshFiles[newIdx]))
                m_skeletalMeshSelectedIdx = prevIdx;
        }

        ImGui::Separator();
    }

    // Clip picker
    {
        std::vector<const char*> items;
        items.reserve(clips.size());
        for (const auto& clip : clips)
            items.push_back(clip.name.c_str());

        // Reach mode picks the walk and idle clips itself; a manual switch would leave the
        // character sliding in the wrong clip, so the picker is disabled there.
        ImGui::BeginDisabled(m_reachActive);
        if (ImGui::Combo("Clip", &selectedClipIndex, items.data(), static_cast<int>(items.size())))
        {
            // Locomotion-to-locomotion switches on the path cross-fade (phase synced). Any other
            // clip cross-fades from its first frame while the path plays, and cuts when paused.
            int locomotionTarget = -1;
            if (m_pathMotionActive && m_pathLocomotionIndex >= 0)
            {
                const auto& locomotion =
                    m_activeScene.objects[m_skeletalObjectIndex].pathMotion.locomotion;
                for (size_t i = 0; i < locomotion.size(); ++i)
                    if (locomotion[i].clip == clips[selectedClipIndex].name)
                        locomotionTarget = static_cast<int>(i);
            }
            if (locomotionTarget >= 0)
            {
                SelectPathLocomotion(locomotionTarget);
            }
            else if (m_pathMotionActive && animator.playing)
            {
                Anim::BeginBlendTo(
                    animator,
                    skeleton,
                    clips[selectedClipIndex],
                    m_activeScene.objects[m_skeletalObjectIndex].pathMotion.locomotionBlendSeconds,
                    false);
                m_timelineOwnsPose = false;
                SyncPathLocomotion();
            }
            else
            {
                Anim::SetClip(animator, skeleton, clips[selectedClipIndex]);
                SyncPathLocomotion();
            }
        }
        ImGui::EndDisabled();
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
    if (m_pathMotionActive && m_pathPaceCoupled && m_pathLocomotionIndex >= 0)
        ImGui::TextDisabled(
            m_timelineOwnsPose
                ? "Timeline scrub owns the pose; Play or a path scrub returns it to the path"
                : "Clip time follows path distance; scrubbing below pauses the path");

    // Bone picker for timeline
    {
        std::vector<const char*> items;
        items.reserve(skeleton.bones.size());
        for (const auto& bone : skeleton.bones)
            items.push_back(bone.name.c_str());

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

    drawList->AddRectFilled(canvasPos,
                            ImVec2(canvasPos.x + canvasSize.x, canvasPos.y + canvasSize.y),
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
    if (!m_reachActive && ImGui::IsItemActive() && duration > 1e-4f)
    {
        animator.playing = false;
        m_timelineOwnsPose = true;
        Anim::EndBlend(animator);
        float u = std::clamp((ImGui::GetIO().MousePos.x - canvasPos.x) / canvasSize.x, 0.0f, 1.0f);
        Anim::SeekTo(animator, u * duration);
    }

    ImGui::TextDisabled("%d keyframes on this bone", static_cast<int>(keys.size()));

    ImGui::End();
}

// Labels go through ImGui's foreground list instead of GL so the numbered markers stay readable at
// any distance.
void Renderer::DrawPathPointLabels()
{
    if (!m_pathMotionActive || !m_showPathPoints || (m_reachActive && !m_reachPathValid))
        return;

    ImGuiIO& io = ImGui::GetIO();
    ImDrawList* drawList = ImGui::GetForegroundDrawList();
    glm::mat4 viewProj = m_cachedProj * m_cachedView;

    const auto& points = m_pathFollower.GetPath().GetControlPoints();
    for (size_t i = 0; i < points.size(); ++i)
    {
        glm::vec4 clip = viewProj * glm::vec4(points[i].x, points[i].y + 0.4f, points[i].z, 1.0f);
        if (clip.w <= 1e-4f)
            continue;
        glm::vec2 ndc = glm::vec2(clip) / clip.w;
        ImVec2 screen((ndc.x * 0.5f + 0.5f) * io.DisplaySize.x,
                      (1.0f - (ndc.y * 0.5f + 0.5f)) * io.DisplaySize.y);

        char label[8];
        snprintf(label, sizeof(label), "P%zu", i);
        ImVec2 size = ImGui::CalcTextSize(label);
        ImVec2 pos(screen.x - 0.5f * size.x, screen.y - size.y);
        drawList->AddRectFilled(ImVec2(pos.x - 3, pos.y - 1),
                                ImVec2(pos.x + size.x + 3, pos.y + size.y + 1),
                                IM_COL32(0, 0, 0, 160),
                                3.0f);
        bool selected = static_cast<int>(i) == m_pathSelectedPoint;
        drawList->AddText(pos,
                          selected ? IM_COL32(255, 255, 80, 255) : IM_COL32(255, 160, 60, 255),
                          label);
    }
}

namespace
{
    // One lap of value(tau) with the two ease boundaries and a playhead; values are drawn against
    // [0, maxValue].
    template <typename ValueFn>
    void DrawLapPlot(const char* id,
                     ValueFn value,
                     double maxValue,
                     double easeInEnd,
                     double easeOutStart,
                     double playheadTau,
                     ImU32 color)
    {
        ImVec2 origin = ImGui::GetCursorScreenPos();
        ImVec2 size(ImGui::GetContentRegionAvail().x, 80.0f);
        ImGui::InvisibleButton(id, size);
        ImDrawList* drawList = ImGui::GetWindowDrawList();
        drawList->AddRectFilled(origin,
                                ImVec2(origin.x + size.x, origin.y + size.y),
                                IM_COL32(30, 30, 30, 255));

        auto toScreen = [&](double tau, double v)
        {
            double h = (maxValue > 0.0) ? std::clamp(v / maxValue, 0.0, 1.0) : 0.0;
            return ImVec2(origin.x + static_cast<float>(tau) * size.x,
                          origin.y + size.y - 2.0f - static_cast<float>(h) * (size.y - 4.0f));
        };

        for (double boundary : {easeInEnd, easeOutStart})
        {
            float x = origin.x + static_cast<float>(boundary) * size.x;
            drawList->AddLine(ImVec2(x, origin.y),
                              ImVec2(x, origin.y + size.y),
                              IM_COL32(110, 110, 110, 255));
        }

        constexpr int kSamples = 128;
        std::array<ImVec2, kSamples + 1> points;
        for (int i = 0; i <= kSamples; ++i)
        {
            double tau = static_cast<double>(i) / kSamples;
            points[i] = toScreen(tau, value(tau));
        }
        drawList->AddPolyline(points.data(), kSamples + 1, color, 0, 1.5f);

        ImVec2 head = toScreen(playheadTau, value(playheadTau));
        drawList->AddLine(ImVec2(head.x, origin.y),
                          ImVec2(head.x, origin.y + size.y),
                          IM_COL32(255, 70, 70, 255),
                          1.5f);
        drawList->AddCircleFilled(head, 3.5f, IM_COL32(255, 70, 70, 255));
    }

    const char* BuildStatusName(Anim::ArcLengthBuildStatus status)
    {
        switch (status)
        {
            case Anim::ArcLengthBuildStatus::Success:
                return "ok";
            case Anim::ArcLengthBuildStatus::NotBuilt:
                return "not built";
            case Anim::ArcLengthBuildStatus::InvalidOptions:
                return "invalid options";
            case Anim::ArcLengthBuildStatus::InvalidPath:
                return "invalid path";
            case Anim::ArcLengthBuildStatus::DepthLimit:
                return "depth limit";
            case Anim::ArcLengthBuildStatus::SampleLimit:
                return "sample limit";
            case Anim::ArcLengthBuildStatus::NumericFailure:
                return "numeric failure";
        }
        return "?";
    }

    const char* HeadingSourceName(Anim::PathHeadingSource source)
    {
        switch (source)
        {
            case Anim::PathHeadingSource::LookAhead:
                return "look-ahead target";
            case Anim::PathHeadingSource::Tangent:
                return "tangent fallback";
            case Anim::PathHeadingSource::Previous:
                return "previous heading fallback";
        }
        return "?";
    }
}  // namespace

void Renderer::SelectPathLocomotion(int locomotionIndex)
{
    const PathMotionBinding& binding = m_activeScene.objects[m_skeletalObjectIndex].pathMotion;
    const auto& locomotion = binding.locomotion;
    for (size_t i = 0; i < m_animationClips.size(); ++i)
    {
        if (m_animationClips[i].name != locomotion[locomotionIndex].clip)
            continue;
        m_animSelectedClipIndex = static_cast<int>(i);
        // The blend clock stops while paused, so a paused switch cuts rather than hiding the new
        // clip until Play.
        Anim::BeginBlendTo(m_animator,
                           m_skeleton,
                           m_animationClips[i],
                           m_animator.playing ? binding.locomotionBlendSeconds : 0.0f);
        m_timelineOwnsPose = false;
        SyncPathLocomotion();
        return;
    }
}

void Renderer::DrawPathMotionPanel()
{
    if (!m_ready)
        return;

    ImGui::Begin("Path Motion");

    if (!m_pathMotionActive)
    {
        ImGui::PushTextWrapPos();
        ImGui::TextDisabled("%s",
                            m_pathMotionStatus.empty()
                                ? "No path binding in this scene; load path_motion.json."
                                : m_pathMotionStatus.c_str());
        ImGui::PopTextWrapPos();
        ImGui::End();
        return;
    }

    SceneObject& obj = m_activeScene.objects[m_skeletalObjectIndex];
    PathMotionBinding& binding = obj.pathMotion;
    const Anim::PathFollowerState& state = m_pathFollower.GetState();
    const Anim::PathFollowerSettings& settings = m_pathFollower.GetSettings();
    const Anim::ArcLengthTable& tables = m_pathFollower.GetTables();
    const Anim::SpeedProfile& profile = m_pathFollower.GetProfile();
    const double length = tables.GetTotalLength();
    const double lapSeconds = m_pathFollower.GetLapDurationSeconds();

    if (ImGui::BeginTable("##path_motion_cols", 2, ImGuiTableFlags_Resizable))
    {
        ImGui::TableNextColumn();

        ImGui::SeparatorText("Playback");
        if (ImGui::Button(m_animator.playing ? "Pause" : "Play"))
            m_animator.playing = !m_animator.playing;
        // Reach mode drives the follower itself; resetting or scrubbing it would teleport the
        // character, so those controls (and the clip picker below) are disabled there.
        ImGui::BeginDisabled(m_reachActive);
        ImGui::SameLine();
        if (ImGui::Button("Reset"))
        {
            m_pathFollower.Reset();
            m_timelineOwnsPose = false;
        }
        ImGui::SameLine();
        ImGui::Text("Lap %llu", static_cast<unsigned long long>(state.lapCount));

        // Scrubbing pauses playback so the slider, not the clock, is the single authority while
        // dragging.
        float progress = static_cast<float>(state.normalizedDistance);
        bool scrubbed = ImGui::SliderFloat("Lap progress",
                                           &progress,
                                           0.0f,
                                           1.0f,
                                           "%.3f",
                                           ImGuiSliderFlags_AlwaysClamp);
        if (ImGui::IsItemActive())
            m_animator.playing = false;
        if (scrubbed)
        {
            m_pathFollower.Seek(progress, state.lapCount);
            m_timelineOwnsPose = false;
        }
        ImGui::EndDisabled();

        ImGui::SeparatorText("Locomotion");
        {
            const char* preview = (m_pathLocomotionIndex >= 0)
                                      ? binding.locomotion[m_pathLocomotionIndex].clip.c_str()
                                      : "(not a locomotion clip)";
            ImGui::BeginDisabled(m_reachActive);
            if (ImGui::BeginCombo("Clip##path", preview))
            {
                for (int i = 0; i < static_cast<int>(binding.locomotion.size()); ++i)
                    if (ImGui::Selectable(binding.locomotion[i].clip.c_str(),
                                          i == m_pathLocomotionIndex))
                        SelectPathLocomotion(i);
                ImGui::EndCombo();
            }
            ImGui::EndDisabled();
        }
        ImGui::SliderFloat("Blend time (s)",
                           &binding.locomotionBlendSeconds,
                           0.0f,
                           1.5f,
                           "%.2f",
                           ImGuiSliderFlags_AlwaysClamp);
        if (ImGui::IsItemHovered())
            ImGui::SetTooltip("Cross-fade length when switching clips. 0 cuts immediately.");
        if (m_animator.blendFromClip && m_animator.clip)
            ImGui::Text("Blending %s -> %s  %.0f%%",
                        m_animator.blendFromClip->name.c_str(),
                        m_animator.clip->name.c_str(),
                        Anim::BlendWeight(m_animator) * 100.0f);
        ImGui::Checkbox("Pace coupled to distance", &m_pathPaceCoupled);
        if (ImGui::IsItemHovered())
            ImGui::SetTooltip(
                "Off: the clip plays on its own clock while the character travels, "
                "to compare foot sliding.");

        if (m_pathLocomotionIndex >= 0)
        {
            LocomotionCalibration& calib = binding.locomotion[m_pathLocomotionIndex];
            ImGui::SliderFloat("Cruise speed (m/s)",
                               &calib.cruiseSpeed,
                               0.0f,
                               10.0f,
                               "%.2f",
                               ImGuiSliderFlags_AlwaysClamp);
            ImGui::SliderFloat("Stride (m/cycle)",
                               &calib.strideMeters,
                               0.3f,
                               6.0f,
                               "%.2f",
                               ImGuiSliderFlags_AlwaysClamp);
            // Cycles per second times clip length: 1.0 means the clip plays at its authored speed.
            double clipRate =
                state.speed * Anim::DurationSeconds(m_animator) / settings.strideMeters;
            ImGui::Text("Clip playback rate x%.2f (1.00 = authored)", clipRate);
        }
        else if (m_reachActive)
        {
            ImGui::TextDisabled("Standing: pace sync resumes on the next walk");
        }
        else
        {
            ImGui::TextColored(ImVec4(1.0f, 0.8f, 0.3f, 1.0f),
                               "Pace sync off: current clip has no locomotion calibration");
        }

        ImGui::SeparatorText("Speed profile");
        ImGui::SliderFloat("Ease-in end",
                           &binding.easeInEnd,
                           0.01f,
                           binding.easeOutStart - 0.01f,
                           "%.2f",
                           ImGuiSliderFlags_AlwaysClamp);
        ImGui::SliderFloat("Ease-out start",
                           &binding.easeOutStart,
                           binding.easeInEnd + 0.01f,
                           0.99f,
                           "%.2f",
                           ImGuiSliderFlags_AlwaysClamp);
        ImGui::SliderFloat("Look-ahead (m)",
                           &binding.lookAheadMeters,
                           0.1f,
                           3.0f,
                           "%.2f",
                           ImGuiSliderFlags_AlwaysClamp);
        if (m_pathFollower.GetEffectiveLookAheadMeters() < settings.lookAheadMeters)
            ImGui::TextDisabled("Look-ahead capped to %.2f m (10%% of path)",
                                m_pathFollower.GetEffectiveLookAheadMeters());
        if (!m_pathSettingsError.empty())
            ImGui::TextColored(ImVec4(1.0f, 0.4f, 0.4f, 1.0f),
                               "Rejected: %s",
                               m_pathSettingsError.c_str());

        // Approach paths are planned, not edited.
        if (!m_reachActive)
            DrawPathPointEditor();

        ImGui::SeparatorText("Overlays");
        ImGui::Checkbox("Curve", &m_showPathCurve);
        ImGui::SameLine();
        ImGui::Checkbox("Polygon", &m_showPathPolygon);
        ImGui::SameLine();
        ImGui::Checkbox("Points", &m_showPathPoints);
        ImGui::Checkbox("COI", &m_showPathCOI);
        ImGui::SameLine();
        ImGui::Checkbox("Distance ticks", &m_showPathTicks);
        if (m_showPathTicks && ImGui::SliderFloat("Tick spacing (m)",
                                                  &m_pathTickSpacing,
                                                  0.1f,
                                                  2.0f,
                                                  "%.2f",
                                                  ImGuiSliderFlags_AlwaysClamp))
            m_pathOverlayDirty = true;
        if (ImGui::Button("Frame Path"))
            RequestPathCameraFrame();

        ImGui::TableNextColumn();

        const double easeIn = profile.GetEaseInEnd();
        const double easeOut = profile.GetEaseOutStart();
        const bool moving = std::isfinite(lapSeconds) && lapSeconds > 0.0;

        ImGui::SeparatorText("Distance-time");
        if (moving)
            ImGui::Text("s = %.2f / %.2f m, t = %.2f / %.2f s",
                        state.normalizedDistance * length,
                        length,
                        state.normalizedTime * lapSeconds,
                        lapSeconds);
        else
            ImGui::Text("s = %.2f / %.2f m, stationary (cruise speed 0)",
                        state.normalizedDistance * length,
                        length);
        DrawLapPlot(
            "##distance_plot",
            [&](double tau) { return profile.Evaluate(tau).normalizedDistance * length; },
            length,
            easeIn,
            easeOut,
            state.normalizedTime,
            IM_COL32(60, 200, 255, 255));

        ImGui::SeparatorText("Speed-time");
        ImGui::Text("v = %.2f m/s, cruise %.2f m/s", state.speed, settings.cruiseSpeed);
        DrawLapPlot(
            "##speed_plot",
            [&](double tau)
            { return moving ? profile.Evaluate(tau).normalizedSpeed * length / lapSeconds : 0.0; },
            settings.cruiseSpeed * 1.1,
            easeIn,
            easeOut,
            state.normalizedTime,
            IM_COL32(120, 255, 120, 255));
        ImGui::TextDisabled("x: one lap; grey: ease-in end / ease-out start; red: now");

        ImGui::SeparatorText("Current sample");
        Anim::PathParameter loc =
            tables.ParameterAtWrappedNormalizedLength(state.normalizedDistance);
        ImGui::Text("Segment %zu, t = %.4f, u = %.4f",
                    loc.segmentIndex,
                    loc.parameter,
                    state.normalizedDistance);
        ImGui::Text("Travelled %.2f m, gait cycles %.2f, phase %.3f",
                    state.unwrappedDistance,
                    state.gaitCycles,
                    state.gaitPhase);
        ImGui::Text("Heading: %s", HeadingSourceName(state.headingSource));

        if (ImGui::CollapsingHeader("Arc-length tables"))
        {
            const Anim::ArcLengthBuildOptions& options = tables.GetOptions();
            const Anim::ArcLengthBuildReport& report = tables.GetLastBuildReport();
            const auto& segments = tables.GetSegments();
            ImGui::Text("Total %.4f m, %zu segments, %zu global samples",
                        length,
                        segments.size(),
                        tables.GetGlobalSamples().size());
            ImGui::Text("Tolerance %.0e m, max interval %.4f, max depth %u",
                        options.chordTolerance,
                        options.maxParameterInterval,
                        options.maxDepth);
            ImGui::Text("Build %s: deepest %u, max discrepancy %.2e m, max width %.4f",
                        BuildStatusName(report.status),
                        report.maxDepthReached,
                        report.maxAcceptedDiscrepancy,
                        report.maxAcceptedInterval);

            if (ImGui::BeginTable(
                    "##segments",
                    3,
                    ImGuiTableFlags_Borders | ImGuiTableFlags_RowBg | ImGuiTableFlags_ScrollY,
                    ImVec2(0.0f, 160.0f)))
            {
                ImGui::TableSetupScrollFreeze(0, 1);
                ImGui::TableSetupColumn("Segment");
                ImGui::TableSetupColumn("Length (m)");
                ImGui::TableSetupColumn("Samples");
                ImGui::TableHeadersRow();
                for (size_t i = 0; i < segments.size(); ++i)
                {
                    ImGui::TableNextRow();
                    if (i == loc.segmentIndex)
                        ImGui::TableSetBgColor(ImGuiTableBgTarget_RowBg1,
                                               IM_COL32(80, 60, 20, 255));
                    ImGui::TableNextColumn();
                    ImGui::Text("P%zu-P%zu", i, (i + 1) % segments.size());
                    ImGui::TableNextColumn();
                    ImGui::Text("%.4f", segments[i].length);
                    ImGui::TableNextColumn();
                    ImGui::Text("%zu", segments[i].samples.size());
                }
                ImGui::EndTable();
            }
        }

        ImGui::EndTable();
    }

    ImGui::End();
}

void Renderer::DrawPathPointEditor()
{
    // Points are only ever replaced through TryApplyPathPoints, which rejects invalid shapes and
    // keeps the last valid path.
    const std::vector<glm::vec2>& points =
        m_activeScene.objects[m_skeletalObjectIndex].pathMotion.controlPointsXZ;
    auto apply = [&](const std::vector<glm::vec2>& candidate)
    {
        std::string error;
        bool ok = TryApplyPathPoints(candidate, error);
        m_pathEditError = ok ? std::string() : error;
        return ok;
    };

    ImGui::SeparatorText("Control points");
    ImGui::TextDisabled("%zu points (min %zu). Edits are session-only; Copy JSON to keep them.",
                        points.size(),
                        Anim::CubicPath::kMinControlPoints);

    for (int i = 0; i < static_cast<int>(points.size()); ++i)
    {
        if (i % 6 != 0)
            ImGui::SameLine();
        char label[8];
        snprintf(label, sizeof(label), "P%d", i);
        if (ImGui::Selectable(label, m_pathSelectedPoint == i, 0, ImVec2(36.0f, 0.0f)))
        {
            m_pathSelectedPoint = (m_pathSelectedPoint == i) ? -1 : i;
            m_selection = Selection{};
        }
    }

    if (m_pathSelectedPoint >= static_cast<int>(points.size()))
        m_pathSelectedPoint = -1;

    if (m_pathSelectedPoint >= 0)
    {
        const int sel = m_pathSelectedPoint;
        glm::vec2 p = points[sel];
        if (ImGui::DragFloat2("X / Z (m)", &p.x, 0.02f, -kGroundHalfSize, kGroundHalfSize, "%.2f"))
        {
            std::vector<glm::vec2> candidate = points;
            candidate[sel] = p;
            apply(candidate);
        }
        // Playback stays paused after an edit; the user resumes it explicitly.
        if (ImGui::IsItemActive())
            m_animator.playing = false;

        ImGui::Checkbox("Floor gizmo", &m_pathPointGizmo);
        ImGui::SameLine();
        if (ImGui::Button("Insert after"))
        {
            // New point sits on the current curve, halfway along the segment it splits.
            Anim::Vec3 mid = m_pathFollower.GetPath().Evaluate(static_cast<size_t>(sel), 0.5f);
            std::vector<glm::vec2> candidate = points;
            candidate.insert(candidate.begin() + sel + 1, glm::vec2(mid.x, mid.z));
            if (apply(candidate))
                m_pathSelectedPoint = sel + 1;
        }
        ImGui::SameLine();
        bool atMinimum = points.size() <= Anim::CubicPath::kMinControlPoints;
        if (atMinimum)
            ImGui::BeginDisabled();
        if (ImGui::Button("Delete"))
        {
            std::vector<glm::vec2> candidate = points;
            candidate.erase(candidate.begin() + sel);
            if (apply(candidate))
                m_pathSelectedPoint = std::min(sel, static_cast<int>(candidate.size()) - 1);
        }
        if (atMinimum)
            ImGui::EndDisabled();
    }

    if (ImGui::Button("Reset path") && apply(m_pathDefaultPointsXZ))
        m_pathSelectedPoint = -1;
    ImGui::SameLine();
    if (ImGui::Button("Copy JSON"))
    {
        std::string json = "\"controlPoints\": [\n";
        char entry[64];
        for (size_t i = 0; i < points.size(); ++i)
        {
            snprintf(entry,
                     sizeof(entry),
                     "  [%.3f, %.3f]%s\n",
                     points[i].x,
                     points[i].y,
                     (i + 1 < points.size()) ? "," : "");
            json += entry;
        }
        json += "]";
        ImGui::SetClipboardText(json.c_str());
    }

    if (!m_pathEditError.empty())
    {
        ImGui::PushTextWrapPos();
        ImGui::TextColored(ImVec4(1.0f, 0.4f, 0.4f, 1.0f),
                           "Edit rejected, previous path kept: %s",
                           m_pathEditError.c_str());
        ImGui::PopTextWrapPos();
    }
}

void Renderer::DrawIKPanel()
{
    if (!m_ready)
        return;

    ImGui::Begin("IK Reach");
    if (!m_reachActive)
    {
        ImGui::TextDisabled("%s",
                            m_reachStatus.empty() ? "No valid ikReach binding in this scene"
                                                  : m_reachStatus.c_str());
        ImGui::End();
        return;
    }

    SceneObject& owner = m_activeScene.objects[m_skeletalObjectIndex];
    glm::vec3& target = m_activeScene.objects[m_ikTargetIndex].position;

    // Re-enabling after an error (e.g. a rejected path) clears the message and tries again.
    if (ImGui::Checkbox("Enabled", &m_reachEnabled) && m_reachEnabled)
        m_reachStatus.clear();
    ImGui::SameLine();
    // Switching to manual keeps any reach in progress, pinned to where the target is now.
    if (ImGui::Checkbox("Auto", &m_reachAuto) && !m_reachAuto)
    {
        m_reachHasCommand = m_reach.GetState() != Anim::ReachState::Idle;
        m_reachCommandTarget = target;
    }
    ImGui::SameLine();
    ImGui::BeginDisabled(m_reachAuto);
    if (ImGui::Button("Go"))
    {
        m_reachHasCommand = true;
        m_reachCommandTarget = target;
        m_reachEnabled = true;
        m_reachStatus.clear();
    }
    ImGui::SameLine();
    if (ImGui::Button("Stop"))
        m_reachHasCommand = false;
    ImGui::EndDisabled();

    ImGui::SeparatorText("Target");
    ImGui::DragFloat3("Position", &target.x, 0.02f);
    ImGui::SameLine();
    if (ImGui::SmallButton("Select"))
        m_selection = {SelectionKind::Object, m_ikTargetIndex};
    ImGui::TextDisabled("Shift+click floor, arrows move, PgUp/PgDn height");

    ImGui::SeparatorText("Status");
    ImGui::Text("State %s, IK weight %.2f",
                Anim::ToString(m_reach.GetState()),
                m_reachOutput.weight);
    ImGui::Text("Shoulder to target %.2f m, arm reach %.2f m, %s",
                m_reachDistance,
                m_reachLength,
                m_reachInReach ? "in reach" : "out of reach");
    if (m_ikResult.tier > 0)
        ImGui::Text("Tier %d, %s, %s, %d iterations, error %.4f m",
                    m_ikResult.tier,
                    m_ikResult.reachable ? "reachable" : "chain too short",
                    m_ikResult.converged ? "converged" : "not converged",
                    m_ikResult.iterations,
                    m_ikResult.error);
    else
        ImGui::TextDisabled("Solver idle");
    if (!m_reachStatus.empty())
        ImGui::TextColored(ImVec4(1.0f, 0.4f, 0.4f, 1.0f), "%s", m_reachStatus.c_str());

    ImGui::SeparatorText("Joints");
    if (ImGui::BeginTable("##ik_joints",
                          4,
                          ImGuiTableFlags_RowBg | ImGuiTableFlags_SizingStretchProp))
    {
        ImGui::TableSetupColumn("Bone");
        ImGui::TableSetupColumn("Limit");
        ImGui::TableSetupColumn("Angle");
        ImGui::TableSetupColumn("Range");
        ImGui::TableHeadersRow();
        for (const IKJointReadout& joint : m_ikJointReadouts)
        {
            float angle = glm::degrees(joint.angle);
            float lo = glm::degrees(joint.minAngle);
            float hi = glm::degrees(joint.maxAngle);
            // Orange within half a degree of a limit, i.e. where the constraint is clamping.
            bool atLimit = angle <= lo + 0.5f || angle >= hi - 0.5f;
            if (joint.type == Anim::JointLimitType::Cone)
                atLimit = angle >= hi - 0.5f;

            ImGui::TableNextRow();
            ImGui::TableNextColumn();
            const std::string& name = m_skeleton.bones[joint.bone].name;
            size_t colon = name.find(':');
            ImGui::TextUnformatted(colon == std::string::npos ? name.c_str()
                                                              : name.c_str() + colon + 1);
            ImGui::TableNextColumn();
            ImGui::TextUnformatted(joint.type == Anim::JointLimitType::Cone ? "Cone" : "Hinge");
            ImGui::TableNextColumn();
            if (atLimit)
                ImGui::TextColored(ImVec4(1.0f, 0.6f, 0.2f, 1.0f), "%.1f", angle);
            else
                ImGui::Text("%.1f", angle);
            ImGui::TableNextColumn();
            ImGui::Text("%.0f to %.0f", lo, hi);
        }
        ImGui::EndTable();
    }

    ImGui::SeparatorText("Solver");
    ImGui::Checkbox("Constraints", &m_ikUseConstraints);
    ImGui::SameLine();
    ImGui::Checkbox("Pole", &m_ikUsePole);
    ImGui::SameLine();
    ImGui::Checkbox("Spine tier", &m_ikUseSpine);
    ImGui::SliderFloat("Tolerance (m)",
                       &m_ikTolerance,
                       1e-4f,
                       1e-2f,
                       "%.4f",
                       ImGuiSliderFlags_Logarithmic | ImGuiSliderFlags_AlwaysClamp);
    ImGui::SliderInt("Max iterations",
                     &m_ikMaxIterations,
                     1,
                     64,
                     "%d",
                     ImGuiSliderFlags_AlwaysClamp);

    ImGui::SeparatorText("Reach");
    ImGui::SliderFloat("Reach time (s)",
                       &owner.ikReach.reachSeconds,
                       0.2f,
                       3.0f,
                       "%.2f",
                       ImGuiSliderFlags_AlwaysClamp);
    ImGui::SliderFloat("Standoff (m)",
                       &owner.ikReach.standoffMeters,
                       0.15f,
                       0.8f,
                       "%.2f",
                       ImGuiSliderFlags_AlwaysClamp);
    ImGui::Checkbox("IK overlays", &m_showReachOverlay);

    ImGui::End();
}
