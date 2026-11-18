#include "fluid/cuda/SparseGather.h"

#include "fluid/cuda/CudaCheck.h"
#include "fluid/cuda/GridMath.cuh"
#include "fluid/cuda/Shapes.cuh"
#include "fluid/cuda/SparseGrid.cuh"

namespace Fluid
{
    namespace
    {
        constexpr int kSubcellThreads = kSubcellDim * kSubcellDim * kSubcellDim;

        __global__ void GatherKernel(VoxelSpace space,
                                     ShapeList obstacles,
                                     float levelSetRadius,
                                     const Node* nodes,
                                     const int* brickNodes,
                                     const unsigned int* counts,
                                     const unsigned int* offsets,
                                     const float4* listPositions,
                                     const float4* listVelocities,
                                     cudaSurfaceObject_t velocitySurface,
                                     cudaSurfaceObject_t markerSurface,
                                     cudaSurfaceObject_t levelSetSurface)
        {
            __shared__ float4 sharedPositions[kSubcellThreads];
            __shared__ float4 sharedVelocities[kSubcellThreads];

            const int subcell = blockIdx.x;
            const int brick = subcell / kSubcellsPerBrick;
            const int node = brickNodes[brick];
            if (node == kNullNode)
                return;

            const int s = subcell % kSubcellsPerBrick;
            const int3 subOffset = make_int3((s % kSubcellsPerAxis) * kSubcellDim,
                                             ((s / kSubcellsPerAxis) % kSubcellsPerAxis) * kSubcellDim,
                                             (s / (kSubcellsPerAxis * kSubcellsPerAxis)) * kSubcellDim);
            const int3 t = make_int3(threadIdx.x, threadIdx.y, threadIdx.z);
            const int3 local = make_int3(subOffset.x + t.x, subOffset.y + t.y, subOffset.z + t.z);
            const int3 leaf = nodes[node].coord;
            const int3 voxel = BrickVoxel(leaf, local);
            const int3 subMinVoxel = BrickVoxel(leaf, subOffset);
            const float3 subMin = space.origin + make_float3(subMinVoxel.x - kCoordOffset,
                                                             subMinVoxel.y - kCoordOffset,
                                                             subMinVoxel.z - kCoordOffset) *
                                                     space.h;
            const float invH = 1.0f / space.h;
            const float3 center = VoxelCenter(space, voxel);
            const float kernelRadius = 2.0f * levelSetRadius;
            const float invKernelRadius2 = 1.0f / (kernelRadius * kernelRadius);

            const float3 fu = make_float3(t.x, t.y + 0.5f, t.z + 0.5f);
            const float3 fv = make_float3(t.x + 0.5f, t.y, t.z + 0.5f);
            const float3 fw = make_float3(t.x + 0.5f, t.y + 0.5f, t.z);
            const float3 fc = make_float3(t.x + 0.5f, t.y + 0.5f, t.z + 0.5f);

            float su = 0.0f, sv = 0.0f, sw = 0.0f;
            float wu = 0.0f, wv = 0.0f, ww = 0.0f;
            float density = 0.0f;
            float levelWeight = 0.0f;
            float3 levelCenter = make_float3(0.0f, 0.0f, 0.0f);
            bool occupied = false;

            const unsigned int count = counts[subcell];
            const unsigned int first = offsets[subcell];
            const int lane = t.x + kSubcellDim * (t.y + kSubcellDim * t.z);
            for (unsigned int base = 0; base < count; base += kSubcellThreads)
            {
                const unsigned int entry = base + lane;
                if (entry < count)
                {
                    sharedPositions[lane] = listPositions[first + entry];
                    sharedVelocities[lane] = listVelocities[first + entry];
                }
                __syncthreads();

                const unsigned int chunk = min(static_cast<unsigned int>(kSubcellThreads), count - base);
                for (unsigned int k = 0; k < chunk; ++k)
                {
                    const float3 x = XYZ(sharedPositions[k]);
                    const float3 v = XYZ(sharedVelocities[k]);
                    const float3 g = (x - subMin) * invH;

                    const float weightU = Hat(g.x - fu.x) * Hat(g.y - fu.y) * Hat(g.z - fu.z);
                    const float weightV = Hat(g.x - fv.x) * Hat(g.y - fv.y) * Hat(g.z - fv.z);
                    const float weightW = Hat(g.x - fw.x) * Hat(g.y - fw.y) * Hat(g.z - fw.z);
                    su += weightU * v.x;
                    sv += weightV * v.y;
                    sw += weightW * v.z;
                    wu += weightU;
                    wv += weightV;
                    ww += weightW;
                    density += Hat(g.x - fc.x) * Hat(g.y - fc.y) * Hat(g.z - fc.z);

                    occupied |= floorf(g.x) == t.x && floorf(g.y) == t.y && floorf(g.z) == t.z;

                    const float3 d = x - center;
                    const float s2 = Dot(d, d) * invKernelRadius2;
                    if (s2 < 1.0f)
                    {
                        const float a = 1.0f - s2;
                        const float w = a * a * a;
                        levelWeight += w;
                        levelCenter = levelCenter + x * w;
                    }
                }
                __syncthreads();
            }

            const float4 velocity = make_float4(wu > 0.0f ? su / wu : 0.0f,
                                                wv > 0.0f ? sv / wv : 0.0f,
                                                ww > 0.0f ? sw / ww : 0.0f,
                                                density);
            unsigned char marker = kCellAir;
            if (FirstContaining(obstacles, center) >= 0)
                marker = kCellSolid;
            else if (occupied)
                marker = kCellFluid;
            float levelSet = space.h;
            if (levelWeight > 0.0f)
                levelSet = fminf(Length(center - levelCenter * (1.0f / levelWeight)) - levelSetRadius, space.h);

            const int3 texel = AtlasTexel(brick, local);
            surf3Dwrite(velocity, velocitySurface, texel.x * static_cast<int>(sizeof(float4)), texel.y, texel.z);
            surf3Dwrite(marker, markerSurface, texel.x * static_cast<int>(sizeof(unsigned char)), texel.y, texel.z);
            surf3Dwrite(levelSet, levelSetSurface, texel.x * static_cast<int>(sizeof(float)), texel.y, texel.z);
        }
    }

    bool SparseParticleToGrid(const SubcellLists& lists,
                              BrickAtlas& atlas,
                              const SparseTopology& topology,
                              const ShapeList& obstacles,
                              float levelSetRadius,
                              GpuTimer* timer)
    {
        if (lists.SubcellCount() == 0 || atlas.Capacity() * kSubcellsPerBrick < lists.SubcellCount())
            return false;

        if (timer)
            timer->Begin("Sparse P2G");
        GatherKernel<<<lists.SubcellCount(), dim3(kSubcellDim, kSubcellDim, kSubcellDim)>>>(
            topology.Space(),
            obstacles,
            levelSetRadius,
            topology.Nodes(),
            atlas.BrickNodes(),
            lists.Counts(),
            lists.Offsets(),
            lists.Positions(),
            lists.Velocities(),
            atlas.Surface(AtlasChannel::Velocity),
            atlas.Surface(AtlasChannel::Marker),
            atlas.Surface(AtlasChannel::LevelSet));
        bool ok = CUDA_CHECK(cudaGetLastError());
        if (timer)
            timer->End();

        if (timer)
            timer->Begin("Apron");
        ok = ok && atlas.UpdateApron(AtlasChannel::Velocity, topology) &&
             atlas.UpdateApron(AtlasChannel::Marker, topology) &&
             atlas.UpdateApron(AtlasChannel::LevelSet, topology, topology.Space().h);
        if (timer)
            timer->End();
        return ok;
    }
}
