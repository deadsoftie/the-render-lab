# The Render Lab

A real-time OpenGL renderer built as a personal sandbox for experimenting with rendering techniques. Implements deferred shading, physically based shading, image based lighting with HDRI environments, moment shadow mapping, and an interactive ImGui debug UI for live parameter tuning.

## Features

### Rendering
- **Deferred Shading** — 4-attachment GBuffer (world position, world normal, albedo, specular + roughness) with a fullscreen PBS light pass
- **Forward Shading** — Retained as a sanity-check toggle
- **Reinhard tone mapping** with exposure control and gamma correction

### Physically Based Shading (PBS)
- GGX micro-facet BRDF — `D_GGX`, `G_Smith` (Smith height-correlated), `F_Schlick`
- Phong `alpha` (1–256) maps to GGX roughness via `sqrt(2 / (alpha + 2))`
- Direct point lights with windowed inverse-square attenuation

### Image Based Lighting (IBL)
- Load any equirectangular `.hdr` environment map at runtime
- **GPU irradiance baking** — compute shader integrates the HDRI hemisphere into a 512×256 diffuse irradiance map on load (32,400-sample Riemann sum)
- **Diffuse IBL** — irradiance map lookup by surface normal
- **Specular IBL** — GGX NDF importance sampling: half-vector `H` sampled from NDF, light direction derived as `L = reflect(-V, H)`, weighted by `G·F·LdotH / (NdotV·NdotH)` after PDF cancellation
- **HDRI skydome** — background pixels reconstructed from inverse view-projection, sampled from the environment map
- Configurable HDRI rotation and IBL sample count (1–100)

### Shadows
- **PCF soft shadows** — 20-sample cubemap filter kernel
- **4-Moment Shadow Mapping (MSM)** — Hamburger MSM with separable Gaussian blur via compute shaders, enabled by default
- Per-light toggle, bias, radius, and blur controls

### Scene
- **Cornell box** with per-wall albedo controls
- Ground plane, cubes (non-uniform scale, correct normal transform), and sphere
- **8 IBL probe spheres** — a row demonstrating the full roughness/F0 parameter space:
  - Left 4: white dielectric (F0 = 0.04), roughness from matte → smooth
  - Right 4: fixed roughness, F0 from plastic → semi-metal → gold → chrome mirror
- **Assimp model import** — arbitrary `.fbx`/`.obj` with position, rotation, scale, and auto-fit controls
- **Orbital camera** — right-drag to orbit, middle-drag to pan, scroll to zoom
- **Light gizmos** — billboarded icon sprites at each light position

### Debug Views
| Index | View |
|-------|------|
| 0 | Final shaded output |
| 1 | World position |
| 2 | World normals |
| 3 | Albedo (Kd) |
| 4 | Specular + roughness |
| 5 | Eye vector |
| 6 | Light globes |
| 7 | Attenuation falloff |
| 8 | MSM depth |
| 9 | Irradiance map (IBL only) |

---

## Prerequisites

- **CMake 3.20+**
- **Ninja** (recommended generator)
- **GCC 11+ or Clang 14+** (C++20 required)
- **vcpkg** — bundled as a submodule; no separate install needed
- **Linux:** OpenGL + GLFW system headers

```bash
# Fedora
sudo dnf install cmake ninja-build gcc-c++ \
  mesa-libGL-devel libX11-devel libXrandr-devel \
  libXinerama-devel libXcursor-devel libXi-devel pkg-config
```

---

## Building

Clone with submodules:

```bash
git clone --recurse-submodules <repo-url>
cd the-render-lab
```

Bootstrap vcpkg (first time only):

```bash
./vcpkg/bootstrap-vcpkg.sh -disableMetrics
```

Configure and build:

```bash
cmake --preset linux-debug
cmake --build --preset linux-debug
```

| Preset | Executable |
|---|---|
| `linux-debug` | `out/build/linux-debug/the-render-lab` |
| `linux-release` | `out/build/linux-release/the-render-lab` |

vcpkg installs all dependencies automatically on first configure.

---

## Running

```bash
./run.sh
```

`run.sh` handles GPU selection on Linux (NVIDIA PRIME offload) and picks the release binary if available, falling back to debug.

---

## Controls

| Action | Input |
|--------|-------|
| Orbit camera | Right mouse drag |
| Pan camera | Middle mouse drag |
| Zoom | Scroll wheel |
| Quit | Escape |

---

## Project Structure

```
assets/
  shaders/
    brdf.glsl               GGX BRDF helpers (D_GGX, G_Smith, F_Schlick, EvalBRDF)
    shadows.glsl            Shared shadow helpers (MSM, PCF kernel, attenuation)
    gbuffer.vert/frag       GBuffer geometry pass
    deferred_light.frag     PBS deferred light pass
    deferred_light.vert     Fullscreen quad vertex shader
    deferred_ibl.frag       IBL light pass (diffuse + specular + skydome)
    irradiance_bake.comp    Compute shader — bakes irradiance map from HDRI
    blur_h.comp / blur_v.comp   Separable Gaussian blur for MSM
    local_light.vert/frag   Additive local light volume pass
    basic_lit.vert/frag     Forward path (sanity check)
    shadow_depth.vert/frag  PCF shadow depth capture
    msm_moment_depth.frag   MSM moment capture
  hdris/                    Place .hdr environment maps here
  models/                   Place .fbx / .obj files here
  textures/                 Light gizmo icon

src/
  App.h/.cpp                Window creation, GLFW/ImGui init, main loop
  graphics/
    Renderer.h/.cpp         Full render pipeline and ImGui debug UI
    GBuffer.h/.cpp          4-attachment MRT framebuffer
    ShadowMap.h/.cpp        PCF cubemap shadow map
    MomentShadowMap.h/.cpp  4-moment cubemap with compute-shader blur
    Shader.h/.cpp           Compile, link, #include resolution, uniform setters
    Mesh.h/.cpp             VAO/VBO/EBO wrapper
    Geometry.h/.cpp         Procedural geometry (Cornell box, sphere, cube, ground)
    Texture.h/.cpp          2D/HDR texture loading via stb_image
  scene/
    Camera.h/.cpp           Perspective camera, view/projection matrices
    CameraController.h/.cpp Orbital camera mouse input
  input/
    Input.h/.cpp            Stateless per-frame key/mouse queries

CMakeLists.txt              Build definition
CMakePresets.json           Debug/release presets for Linux and Windows
run.sh                      Launch wrapper (sets NVIDIA PRIME offload on Linux)
vcpkg.json                  Dependency manifest
```

---

## Rendering Pipeline

### Deferred Path (default)

1. **Shadow Pass** — PCF cubemap depth or MSM moment capture (+ Gaussian blur) per light
2. **GBuffer Pass** — Scene geometry written into 4 MRT attachments; normal matrix supplied from CPU (`transpose(inverse(model))`) to correctly handle non-uniform scale
3. **Fullscreen Light Pass** — Reads GBuffer; in PBS mode evaluates direct lights with GGX BRDF; in IBL mode adds HDRI diffuse + GGX importance-sampled specular and renders the skydome in empty pixels
4. **Local Light Pass** — Skipped in IBL mode; in PBS mode renders additive sphere volume contributions per light

### GBuffer Layout

| Attachment | Format | Contents |
|------------|--------|----------|
| 0 | RGBA16F | World position (w = 1 if geometry present) |
| 1 | RGBA16F | World normal |
| 2 | RGBA8 | Diffuse albedo (Kd) |
| 3 | RGBA16F | Specular F0 (rgb) + roughness alpha (a) |

---

## Dependencies

Managed via **vcpkg** in manifest mode (`vcpkg.json`):

| Library | Purpose |
|---------|---------|
| `glfw3` | Window and OpenGL 4.3 context |
| `glad` | OpenGL function loader |
| `glm` | Math (vectors, matrices, transforms) |
| `stb` | HDR and LDR image loading |
| `assimp` | 3D model import (FBX, OBJ, etc.) |
| `imgui` (+ glfw + opengl3 bindings) | Immediate-mode debug UI |
