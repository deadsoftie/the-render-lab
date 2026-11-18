#pragma once
#include "fluid/cuda/BrickAtlas.h"
#include "fluid/cuda/FluidTypes.h"
#include "fluid/cuda/GpuTimer.h"
#include "fluid/cuda/SparseTopology.h"
#include "fluid/cuda/SubcellLists.h"

namespace Fluid
{
    constexpr float kLevelSetRadiusScale = 0.5f;

    bool SparseParticleToGrid(const SubcellLists& lists,
                              BrickAtlas& atlas,
                              const SparseTopology& topology,
                              const ShapeList& obstacles,
                              float levelSetRadius,
                              GpuTimer* timer = nullptr);
}
