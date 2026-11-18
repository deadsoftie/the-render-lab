#pragma once
#include <cstdint>

#include "fluid/cuda/SparseTypes.h"
#include "fluid/cuda/VecMath.cuh"

namespace Fluid
{
    constexpr uint64_t kCoordMask = (uint64_t{1} << 20) - 1;

    // Bits to shift a voxel coordinate by to get the coordinate of its level-`level` node
    // (a leaf, level 0, covers 8 voxels per axis).
    __host__ __device__ inline int LevelShift(int level)
    {
        return kLog2Dim * (level + 1);
    }

    __host__ __device__ inline int3 ShiftDown(int3 v, int shift)
    {
        return make_int3(v.x >> shift, v.y >> shift, v.z >> shift);
    }

    __host__ __device__ inline int3 ShiftUp(int3 v, int shift)
    {
        return make_int3(v.x << shift, v.y << shift, v.z << shift);
    }

    __host__ __device__ inline int3 ToVoxel(const VoxelSpace& space, float3 p)
    {
        float3 g = (p - space.origin) * (1.0f / space.h);
        return make_int3(static_cast<int>(floorf(g.x)) + kCoordOffset,
                         static_cast<int>(floorf(g.y)) + kCoordOffset,
                         static_cast<int>(floorf(g.z)) + kCoordOffset);
    }

    __host__ __device__ inline int3 NodeCoord(int3 voxel, int level)
    {
        return ShiftDown(voxel, LevelShift(level));
    }

    __host__ __device__ inline int ChildSlot(int3 childCoord)
    {
        const int mask = kBrickDim - 1;
        return (childCoord.x & mask) + kBrickDim * ((childCoord.y & mask) + kBrickDim * (childCoord.z & mask));
    }

    __host__ __device__ inline uint64_t PackKey(int level, int3 coord)
    {
        return (static_cast<uint64_t>(level) << 60) |
               ((static_cast<uint64_t>(coord.x) & kCoordMask) << 40) |
               ((static_cast<uint64_t>(coord.y) & kCoordMask) << 20) |
               (static_cast<uint64_t>(coord.z) & kCoordMask);
    }

    __host__ __device__ inline int KeyLevel(uint64_t key)
    {
        return static_cast<int>(key >> 60);
    }

    __host__ __device__ inline int3 KeyCoord(uint64_t key)
    {
        return make_int3(static_cast<int>((key >> 40) & kCoordMask),
                         static_cast<int>((key >> 20) & kCoordMask),
                         static_cast<int>(key & kCoordMask));
    }

    __host__ __device__ inline float3 VoxelCenter(const VoxelSpace& space, int3 voxel)
    {
        return space.origin + make_float3(voxel.x - kCoordOffset + 0.5f,
                                          voxel.y - kCoordOffset + 0.5f,
                                          voxel.z - kCoordOffset + 0.5f) *
                                  space.h;
    }

    __host__ __device__ inline int3 BrickVoxel(int3 leafCoord, int3 local)
    {
        return make_int3((leafCoord.x << kLog2Dim) + local.x,
                         (leafCoord.y << kLog2Dim) + local.y,
                         (leafCoord.z << kLog2Dim) + local.z);
    }

    __host__ __device__ inline int3 LocalVoxel(int3 voxel)
    {
        const int mask = kBrickDim - 1;
        return make_int3(voxel.x & mask, voxel.y & mask, voxel.z & mask);
    }

    // Texel of a brick-local voxel. local may be -1 or 8 to address the apron.
    __host__ __device__ inline int3 AtlasTexel(int brick, int3 local)
    {
        return make_int3((brick % kAtlasBricksXY) * kBrickTexels + local.x + kApron,
                         ((brick / kAtlasBricksXY) % kAtlasBricksXY) * kBrickTexels + local.y + kApron,
                         (brick / kAtlasBricksPerLayer) * kBrickTexels + local.z + kApron);
    }

    __host__ __device__ inline int SubcellIndex(int brick, int3 subcellCoord)
    {
        const int mask = kSubcellsPerAxis - 1;
        return brick * kSubcellsPerBrick + (subcellCoord.x & mask) +
               kSubcellsPerAxis * ((subcellCoord.y & mask) + kSubcellsPerAxis * (subcellCoord.z & mask));
    }

    __host__ __device__ inline float3 Corner(float3 p, float extent, int corner)
    {
        return make_float3(p.x + ((corner & 1) ? extent : -extent),
                           p.y + ((corner & 2) ? extent : -extent),
                           p.z + ((corner & 4) ? extent : -extent));
    }

    // Walks from the root (node 0) towards targetLevel. Returns kNullNode at the first missing
    // child, with deepestLevel set to the last level that exists. The caller must make sure the
    // voxel lies inside the root; child slots wrap otherwise. With marks set, every node on the
    // path is flagged as still in use (incremental rebuild, paper section 5.2).
    __host__ __device__ inline int FindNode(const Node* nodes,
                                            const int* childLists,
                                            int rootLevel,
                                            int3 voxel,
                                            int targetLevel,
                                            int& deepestLevel,
                                            unsigned char* marks = nullptr)
    {
        int current = 0;
        deepestLevel = rootLevel;
        for (int level = rootLevel - 1; level >= targetLevel; --level)
        {
            int child = childLists[nodes[current].childList * kChildCount +
                                   ChildSlot(NodeCoord(voxel, level))];
            if (child == kNullNode)
                return kNullNode;
            current = child;
            deepestLevel = level;
            if (marks)
                marks[current] = 1;
        }
        return current;
    }
}
