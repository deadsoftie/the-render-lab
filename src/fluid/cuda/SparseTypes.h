#pragma once
#include <cuda_runtime.h>

// Sparse grid layout shared by the CUDA kernels and the GL shaders (fluid_bricks.vert and
// fluid_surface.frag repeat these numbers).
namespace Fluid
{
    // Every tree level has 8 children per axis (512 in total); a leaf owns one brick of 8 voxels
    // per axis.
    constexpr int kLog2Dim = 3;
    constexpr int kBrickDim = 1 << kLog2Dim;
    constexpr int kChildCount = kBrickDim * kBrickDim * kBrickDim;
    constexpr int kMaxLevels = 6;
    // Voxel coordinates are shifted so the domain's voxel 0 sits one brick inside a block aligned
    // to the level 4 node size. Without the extra brick, the one voxel influence padding at the
    // domain min crosses a top-level boundary and forces the root up to level 5.
    constexpr int kCoordOffset = (1 << (kLog2Dim * 5)) + kBrickDim;
    constexpr int kNullNode = -1;
    // Removed node slots stay in the pool with this level until a later rebuild reuses them.
    constexpr int kDeadLevel = -1;
    // Atlas: each brick is 10 texels per axis (8 voxels plus a 1 voxel apron on each side), packed
    // 16 by 16 bricks per layer of 10 texels. A 3D array axis is limited to 16384 texels, which
    // caps the layer count.
    constexpr int kApron = 1;
    constexpr int kBrickTexels = kBrickDim + 2 * kApron;
    constexpr int kAtlasBricksXY = 16;
    constexpr int kAtlasBricksPerLayer = kAtlasBricksXY * kAtlasBricksXY;
    constexpr int kAtlasMaxLayers = 16384 / kBrickTexels;
    // Subcells (paper section 6): 4 voxels per axis, 8 per brick, the unit of work for the gather
    // and the solver.
    constexpr int kLog2SubcellDim = 2;
    constexpr int kSubcellDim = 1 << kLog2SubcellDim;
    constexpr int kSubcellsPerAxis = kBrickDim / kSubcellDim;
    constexpr int kSubcellsPerBrick = kSubcellsPerAxis * kSubcellsPerAxis * kSubcellsPerAxis;

    struct VoxelSpace
    {
        float3 origin{0.0f, 0.0f, 0.0f};
        float h = 1.0f;
    };

    struct VoxelBox
    {
        int3 lo{0, 0, 0};
        int3 hi{0, 0, 0};
    };

    // Pool 0 entry (paper section 3). Interior nodes use childList, an index into the pool of
    // 512 entry child lists; leaves use brick, their atlas slot. The shaders read this layout as
    // 7 ints.
    struct Node
    {
        int level = 0;
        int3 coord{0, 0, 0};
        int parent = kNullNode;
        int childList = kNullNode;
        int brick = kNullNode;
    };
}
