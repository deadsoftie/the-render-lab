#pragma once
#include "fluid/cuda/DenseCg.h"
#include "fluid/cuda/DeviceBuffer.h"
#include "fluid/cuda/FluidTypes.h"
#include "fluid/cuda/GpuTimer.h"

namespace Fluid
{
    // FLIP on a full MAC grid: the reference the sparse stages are tested against and the dense
    // side of the paper's comparisons. Each substep is Transfer (cell sort, gather, forces),
    // Project (divergence, CG, pressure gradient) and Advect (FLIP and PIC blend, RK2, collision).
    // UOld, VOld and WOld keep the raw gather before forces, for the FLIP difference.
    class DenseFlip
    {
       public:
        bool Init(const GridInfo& grid, const ShapeList& obstacles, int particleCount);
        void Release();

        StepStats Step(DeviceBuffer<float4>& positions,
                       DeviceBuffer<float4>& velocities,
                       int count,
                       float frameSeconds,
                       const StepSettings& settings,
                       GpuTimer* timer = nullptr);

        float MaxSpeed(const DeviceBuffer<float4>& velocities, int count);
        bool Transfer(DeviceBuffer<float4>& positions,
                      DeviceBuffer<float4>& velocities,
                      int count,
                      float dt,
                      float3 gravity,
                      GpuTimer* timer = nullptr);
        CgResult Project(float tolerance, int maxIterations, GpuTimer* timer = nullptr);
        bool Advect(DeviceBuffer<float4>& positions,
                    DeviceBuffer<float4>& velocities,
                    int count,
                    float dt,
                    float flipRatio,
                    GpuTimer* timer = nullptr);

        const GridInfo& Grid() const { return m_grid; }
        const DeviceBuffer<float>& U() const { return m_u; }
        const DeviceBuffer<float>& V() const { return m_v; }
        const DeviceBuffer<float>& W() const { return m_w; }
        const DeviceBuffer<float>& UOld() const { return m_uOld; }
        const DeviceBuffer<float>& VOld() const { return m_vOld; }
        const DeviceBuffer<float>& WOld() const { return m_wOld; }
        const DeviceBuffer<float>& Pressure() const { return m_pressure; }
        const DeviceBuffer<unsigned char>& Markers() const { return m_markers; }
        size_t MemoryBytes() const;

       private:
        bool ReserveParticles(int count);

        GridInfo m_grid;
        ShapeList m_obstacles;
        int m_keyBits = 1;

        DeviceBuffer<float> m_u;
        DeviceBuffer<float> m_v;
        DeviceBuffer<float> m_w;
        DeviceBuffer<float> m_uOld;
        DeviceBuffer<float> m_vOld;
        DeviceBuffer<float> m_wOld;
        DeviceBuffer<unsigned char> m_markers;
        DeviceBuffer<float> m_rhs;
        DeviceBuffer<float> m_pressure;

        DeviceBuffer<unsigned int> m_cellStart;
        DeviceBuffer<unsigned int> m_cellEnd;
        DeviceBuffer<unsigned int> m_keys;
        DeviceBuffer<unsigned int> m_sortedKeys;
        DeviceBuffer<unsigned int> m_indices;
        DeviceBuffer<unsigned int> m_sortedIndices;
        DeviceBuffer<unsigned char> m_sortTemp;
        DeviceBuffer<float4> m_scratchPositions;
        DeviceBuffer<float4> m_scratchVelocities;
        DeviceBuffer<unsigned int> m_maxSpeed;

        DenseCg m_cg;
    };
}
