# The Render Lab

A real-time OpenGL renderer built as a personal sandbox for experimenting with rendering techniques. Currently implements deferred shading, moment shadow mapping (MSM), per-light volumes, and an interactive ImGui debug UI for live parameter tuning.

## Features

- **Deferred Shading** — 4-attachment GBuffer (world position, world normal, diffuse, specular + shininess) with a fullscreen Blinn-Phong light pass
- **Forward Shading** — Retained as a sanity-check toggle
- **Omnidirectional Shadow Mapping** — Cubemap depth shadows with PCF soft shadows
- **4-Moment Shadow Mapping (MSM)** — Hamburger MSM with per-face 7-tap Gaussian blur via compute shaders, enabled by default
- **Per-Light Sphere Volumes** — Local lights rendered as instanced sphere meshes in a separate pass
- **Light Gizmos** — Billboarded icon sprites marking light positions in the scene
- **Model Import** — Arbitrary mesh import via assimp (defaults to `assets/models/monkey/monkey.fbx`)
- **Procedural Geometry** — Cornell box, cubes, spheres, ground plane
- **Orbital Camera** — Right-drag to orbit, middle-drag to pan, scroll to zoom
- **9 Debug Views** — Final output, world position, normals, albedo, specular, eye vector, light globes, brightness falloff, MSM depth

## Prerequisites

- **Windows 10/11 x64**
- **Visual Studio 2022** with the "Desktop development with C++" workload (v145 toolset, C++20)
- **vcpkg** — bundled as a submodule; no separate install needed

## Building

Clone with submodules:

```
git clone --recurse-submodules <repo-url>
cd the-render-lab
```

Install dependencies (first time only, or after changing `vcpkg.json`):

```
vcpkg/vcpkg install
```

Build from the command line:

```
msbuild the-render-lab.sln /p:Configuration=Debug /p:Platform=x64
msbuild the-render-lab.sln /p:Configuration=Release /p:Platform=x64
```

Or open `the-render-lab.sln` in Visual Studio 2022 and build normally.

**Output:**
| Configuration | Executable |
|---|---|
| Debug | `bin/Debug-windows-x86_64/the-render-lab.exe` |
| Release | `bin/Release-windows-x86_64/the-render-lab.exe` |

## Running

The executable must be run from the **repository root** so it can find assets at their relative paths:

```
bin/Debug-windows-x86_64/the-render-lab.exe
```

## Controls

| Action       | Input             |
| ------------ | ----------------- |
| Orbit camera | Right mouse drag  |
| Pan camera   | Middle mouse drag |
| Zoom         | Scroll wheel      |

## Project Structure

```
the-render-lab/
├── assets/
│   ├── shaders/          # GLSL shaders (deferred, forward, shadow, MSM, compute blur)
│   ├── textures/         # Light gizmo icon textures
│   └── models/           # 3D model files (monkey.fbx, porsche-spyder/)
├── src/
│   ├── App.h/cpp         # Window creation, main loop
│   ├── graphics/
│   │   ├── Renderer      # Core render pipeline and ImGui debug UI
│   │   ├── GBuffer       # 4-attachment MRT framebuffer
│   │   ├── ShadowMap     # Cubemap PCF shadow maps
│   │   ├── MomentShadowMap  # MSM cubemaps with compute-shader blur
│   │   ├── Shader        # Shader compilation, uniforms, compute dispatch
│   │   ├── Mesh          # VAO/VBO/EBO management
│   │   ├── Geometry      # Procedural mesh generation
│   │   └── Texture       # STB-based image loading
│   ├── scene/
│   │   ├── Camera        # Perspective camera, view/projection matrices
│   │   └── CameraController  # Orbital camera input
│   └── input/
│       └── Input         # Stateless per-frame key/mouse queries
├── build/                # Visual Studio project files
└── vcpkg.json            # Dependency manifest
```

## Rendering Pipeline

### Deferred Path (default)

1. **GBuffer Pass** — Renders scene geometry into 4 MRT attachments
2. **Shadow Pass** — PCF cubemap depth pass or MSM moment pass (+ Gaussian blur) per light
3. **Fullscreen Light Pass** — Reads GBuffer, accumulates Blinn-Phong shading for all lights
4. **Local Light Pass** — Renders sphere volumes per light with per-light shadow lookups

### GBuffer Layout

| Attachment | Format  | Contents                                 |
| ---------- | ------- | ---------------------------------------- |
| 0          | RGBA16F | World position (w=1 if geometry present) |
| 1          | RGBA16F | World normal                             |
| 2          | RGBA8   | Diffuse (Kd)                             |
| 3          | RGBA16F | Specular (rgb) + shininess (a)           |

### Moment Shadow Mapping

Uses the Hamburger 4-MSM technique. Each light renders a cubemap of 4 moment statistics (z, z², z³, z⁴). A separable 7-tap Gaussian blur is applied per cubemap face via compute shaders (ping-pong through a 2D intermediate). The blurred moments are sampled during the light pass to reconstruct a soft shadow factor.

## Dependencies

Managed via **vcpkg** in manifest mode (`vcpkg.json`):

| Library                                 | Purpose                              |
| --------------------------------------- | ------------------------------------ |
| glfw3                                   | Window and OpenGL context creation   |
| glad                                    | OpenGL function loader               |
| glm                                     | Math (vectors, matrices, transforms) |
| stb                                     | Image loading for textures           |
| assimp                                  | 3D model import (FBX, OBJ, etc.)     |
| imgui (+ glfw-binding, opengl3-binding) | Immediate-mode debug UI              |
