# Tech Debt — Performance

Items identified during performance audit. Ordered by impact.

---

## 1. Replace IBL Importance Sampling with Split-Sum LUT
**Impact:** High — eliminates the biggest per-pixel cost  
**Effort:** High

`deferred_ibl.frag` runs a GGX importance sampling loop up to 100 times per pixel per frame.
At 1280×720 with 24 samples that's ~22M `textureLod` calls per frame, each with mip-level
PDF computation (`log2`, `sqrt`).

**Fix:** Precompute two textures offline:
- **DFG LUT** (2D, NdotV × roughness): stores `∫ F(cosθ) G(cosθ) / 4·NdotV dω`
- **Pre-filtered env map** (cubemap mip chain): stores irradiance at each roughness level

Runtime IBL becomes two texture lookups + multiply. Standard split-sum approximation
(Karis 2013 / UE4 approach). Baking can reuse the existing Hammersley + GGX code,
run once on HDRI load.

**Files:** `deferred_ibl.frag`, `Renderer.cpp` (add `BakeSpecularLUT()`, `BakePrefilteredEnv()`)

---

## 2. Geometry Shader Cubemap for Shadow Passes
**Impact:** Medium — cuts 30 draw calls to 5  
**Effort:** High

`ShadowPass()` and `MSMShadowPass()` call `DrawSceneGeometry()` 6× per light (one per
cubemap face) × 5 lights = 30 full-scene draws per frame. Each iteration changes
viewport, binds a different framebuffer attachment, and re-uploads model matrices.

**Fix:** Single draw call per light using a geometry shader that emits `gl_Layer` to
route triangles to all 6 faces at once. Requires:
- New `shadow_depth_cube.geom` geometry shader
- Pack 6 view-projection matrices into a UBO
- `glFramebufferTexture` (attach entire cubemap, not face-by-face)
- Same fix applies to MSM moment pass

**Files:** `Renderer.cpp` (`ShadowPass`, `MSMShadowPass`), new `shadow_depth_cube.geom`

---

## 3. View-Space Depth in GBuffer (for Cel Outline)
**Impact:** Low–Medium — removes 8 matrix multiplies per pixel in outline pass  
**Effort:** Medium

`cel_outline.frag` calls `-(uView * vec4(worldPos, 1.0)).z` once per neighbour sample
(8 times per pixel). The view matrix is uniform but the per-sample multiply still
executes in the fragment shader.

**Fix:** Store view-space depth directly in GBuffer during the GBuffer pass — pack it
into the unused `w` component of the Normal attachment (currently `RGBA16F`, `w` is
wasted). `cel_outline.frag` then reads a single float per sample instead of a mat4 multiply.

Alternatively: half-resolution outline pass + bilinear upscale, which also halves the
texture fetch count.

**Files:** `gbuffer.frag` (write depth to `gNormal.w`), `cel_outline.frag` (read `gNormal.w`)
`GBuffer.h` (document the new packing), `ao.frag` (already uses world pos, unaffected)

---

## 4. Cache Shadow Cubemaps for Static Lights
**Impact:** Medium — skips up to 30 scene redraws per frame when scene is static
**Effort:** Low

`ShadowPass()` / `MSMShadowPass()` re-render every cubemap face every frame, even when
neither the light nor any scene geometry has moved. In this project the Cornell box
and all lights are static for the entire session.

**Fix:** Dirty-flag per light. Set on light position/range edit, on geometry transform
change, or on `m_showIBLProbes` toggle. Skip the render pass for clean lights. Forcing
a full rebuild on shader reload is acceptable.

**Files:** `Renderer.cpp` (`ShadowPass`, `MSMShadowPass`, light/geometry mutators),
`Renderer.h` (`m_shadowDirty[kMaxLights]`)

---

## 5. Instance or Batch Scene Submission
**Impact:** Low — reduces CPU-side draw overhead
**Effort:** Low

`DrawSceneGeometry()` issues one `Draw` + one `SetMat4("uModel", ...)` upload per
primitive. Cornell walls, ground, 3 cubes, sphere, up to 8 probe spheres = 14 draws
per pass. Called once in GBuffer, once per shadow face per light in shadow passes.
Probe spheres are all the same mesh at different transforms — textbook instancing case.

**Fix:** `glDrawElementsInstanced` for probe spheres with a per-instance model matrix
in a VBO or SSBO. Secondary: collapse the three cubes into one instanced draw.

**Files:** `Renderer.cpp` (`DrawSceneGeometry`), `Mesh.h/.cpp` (add `DrawInstanced`)

---

## 6. Frustum Cull Before Shadow Face Submit
**Impact:** Low — skips submits that contribute nothing
**Effort:** Low

Every primitive is re-submitted for all 6 cubemap faces of every light, even when
the primitive sits entirely outside that face's 90° frustum. For Cornell the back
wall never contributes to the +Z face from a light in front, etc.

**Fix:** Per-face frustum vs. per-primitive AABB test before the `Draw` call.
Complements item #2 — if GS layered rendering lands, this moves into the geometry
shader as per-triangle culling.

**Files:** `Renderer.cpp` (`ShadowPass`, `MSMShadowPass`), new AABB helper.

---

## Already Done (not debt)
- MSM on by default (`m_useMSM = true`) — PCF fallback is rarely hit
- Hammersley dirty flag — only rebuilds when `m_iblSamples` changes
- PCF trimmed from 20 → 12 samples
- AO blur radius default reduced 8 → 5
