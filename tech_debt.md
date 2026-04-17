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

## Already Done (not debt)
- MSM on by default (`m_useMSM = true`) — PCF fallback is rarely hit
- Hammersley dirty flag — only rebuilds when `m_iblSamples` changes
- PCF trimmed from 20 → 12 samples
- AO blur radius default reduced 8 → 5
