#pragma once
#include <cstdint>

#include "fluid/cuda/SparseTypes.h"
#include "fluid/cuda/VecMath.cuh"

namespace Fluid
{
    constexpr uint64_t kCoordMask = (uint64_t{1} << 20) - 1;

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

    __host__ __device__ inline float3 Corner(float3 p, float extent, int corner)
    {
        return make_float3(p.x + ((corner & 1) ? extent : -extent),
                           p.y + ((corner & 2) ? extent : -extent),
                           p.z + ((corner & 4) ? extent : -extent));
    }

    __host__ __device__ inline int FindNode(const Node* nodes,
                                            const int* childLists,
                                            int rootLevel,
                                            int3 voxel,
                                            int targetLevel,
                                            int& deepestLevel)
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
        }
        return current;
    }
}
