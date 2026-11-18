#pragma once
#include <cuda_runtime.h>

namespace Fluid
{
    constexpr int kLog2Dim = 3;
    constexpr int kBrickDim = 1 << kLog2Dim;
    constexpr int kChildCount = kBrickDim * kBrickDim * kBrickDim;
    constexpr int kMaxLevels = 6;
    constexpr int kCoordOffset = (1 << (kLog2Dim * 5)) + kBrickDim;
    constexpr int kNullNode = -1;
    constexpr int kDeadLevel = -1;
    constexpr int kApron = 1;
    constexpr int kBrickTexels = kBrickDim + 2 * kApron;
    constexpr int kAtlasBricksXY = 16;
    constexpr int kAtlasBricksPerLayer = kAtlasBricksXY * kAtlasBricksXY;
    constexpr int kAtlasMaxLayers = 16384 / kBrickTexels;
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

    struct Node
    {
        int level = 0;
        int3 coord{0, 0, 0};
        int parent = kNullNode;
        int childList = kNullNode;
        int brick = kNullNode;
    };
}
