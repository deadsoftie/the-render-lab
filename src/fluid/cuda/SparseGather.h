#pragma once
#include "fluid/cuda/BrickAtlas.h"
#include "fluid/cuda/FluidTypes.h"
#include "fluid/cuda/GpuTimer.h"
#include "fluid/cuda/SparseTopology.h"
#include "fluid/cuda/SubcellLists.h"

namespace Fluid
{
    constexpr float kLevelSetRadiusScale = 0.5f;

    inline VoxelBox DomainVoxels(const GridInfo& grid)
    {
        VoxelBox box;
        box.lo = make_int3(kCoordOffset, kCoordOffset, kCoordOffset);
        box.hi = make_int3(kCoordOffset + grid.dims.x, kCoordOffset + grid.dims.y, kCoordOffset + grid.dims.z);
        return box;
    }

    bool SparseParticleToGrid(const SubcellLists& lists,
                              BrickAtlas& atlas,
                              const SparseTopology& topology,
                              const ShapeList& obstacles,
                              const VoxelBox& domain,
                              float levelSetRadius,
                              GpuTimer* timer = nullptr);
}
