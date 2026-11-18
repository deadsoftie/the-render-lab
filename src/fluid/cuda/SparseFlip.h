#pragma once
#include "fluid/cuda/BrickAtlas.h"
#include "fluid/cuda/DeviceBuffer.h"
#include "fluid/cuda/FluidTypes.h"
#include "fluid/cuda/GpuTimer.h"
#include "fluid/cuda/SparseProject.h"
#include "fluid/cuda/SparseTopology.h"
#include "fluid/cuda/SubcellLists.h"

namespace Fluid
{
    // The paper's sparse FLIP step (Alg. 1), split like DenseFlip so the caller can run each stage
    // per substep: Rebuild (topology), Transfer (atlas, subcells, P2G), Project, Advect.
    class SparseFlip
    {
       public:
        void Init(const GridInfo& grid, const ShapeList& obstacles);
        void Release();

        bool Rebuild(const float4* positions, int count, bool incremental, GpuTimer* timer = nullptr);
        bool Transfer(const float4* positions, const float4* velocities, int count, GpuTimer* timer = nullptr);
        CgResult Project(float dt, float3 gravity, float tolerance, int maxIterations, GpuTimer* timer = nullptr);
        bool Advect(float4* positions, float4* velocities, int count, float dt, float flipRatio, GpuTimer* timer = nullptr);

        int CountUncovered(const float4* positions, int count);

        const GridInfo& Grid() const { return m_grid; }
        const SparseTopology& Topology() const { return m_topology; }
        const BrickAtlas& Atlas() const { return m_atlas; }
        const SubcellLists& Subcells() const { return m_subcells; }

       private:
        bool ExtendWalls(AtlasChannel channel);

        GridInfo m_grid;
        ShapeList m_obstacles;
        SparseTopology m_topology;
        BrickAtlas m_atlas;
        SubcellLists m_subcells;
        SparseCg m_cg;
    };
}
