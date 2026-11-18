#include "fluid/cuda/SparseFlip.h"

#include "fluid/cuda/Collision.cuh"
#include "fluid/cuda/CudaCheck.h"
#include "fluid/cuda/GridMath.cuh"
#include "fluid/cuda/SparseGather.h"
#include "fluid/cuda/SparseGrid.cuh"

namespace Fluid
{
    namespace
    {
        constexpr int kBlockSize = 256;

        const dim3 kSubcellBlock(kSubcellDim, kSubcellDim, kSubcellDim);

        struct Tree
        {
            const Node* nodes;
            const int* childLists;
            int rootLevel;
        };

        void BeginStage(GpuTimer* timer, const char* name)
        {
            if (timer)
                timer->Begin(name);
        }

        void EndStage(GpuTimer* timer)
        {
            if (timer)
                timer->End();
        }

        __device__ float4 LoadTexel(cudaSurfaceObject_t surface, int3 texel)
        {
            return surf3Dread<float4>(surface, texel.x * static_cast<int>(sizeof(float4)), texel.y, texel.z);
        }

        __device__ void StoreTexel(cudaSurfaceObject_t surface, int3 texel, float4 value)
        {
            surf3Dwrite(value, surface, texel.x * static_cast<int>(sizeof(float4)), texel.y, texel.z);
        }

        __device__ float Component(float4 v, int axis)
        {
            return axis == 0 ? v.x : (axis == 1 ? v.y : v.z);
        }

        __device__ int OutsideDepth(int cell, int dims)
        {
            return cell < 0 ? -cell : (cell >= dims ? cell - dims + 1 : 0);
        }

        __device__ bool Locate(const Tree& tree, const GridInfo& grid, float3 p, int& brick, float3& local)
        {
            const float3 g = (p - grid.origin) * (1.0f / grid.h);
            const int3 voxel = make_int3(static_cast<int>(floorf(g.x)) + kCoordOffset,
                                         static_cast<int>(floorf(g.y)) + kCoordOffset,
                                         static_cast<int>(floorf(g.z)) + kCoordOffset);
            const int3 root = NodeCoord(voxel, tree.rootLevel);
            const int3 rootCoord = tree.nodes[0].coord;
            if (root.x != rootCoord.x || root.y != rootCoord.y || root.z != rootCoord.z)
                return false;

            int deepest = 0;
            const int leaf = FindNode(tree.nodes, tree.childLists, tree.rootLevel, voxel, 0, deepest);
            if (leaf == kNullNode)
                return false;
            const Node node = tree.nodes[leaf];
            const int3 origin = BrickVoxel(node.coord, make_int3(0, 0, 0));
            local = make_float3(g.x - static_cast<float>(origin.x - kCoordOffset),
                                g.y - static_cast<float>(origin.y - kCoordOffset),
                                g.z - static_cast<float>(origin.z - kCoordOffset));
            brick = node.brick;
            return true;
        }

        __device__ float3 SampleAtlas(cudaTextureObject_t texture, int brick, float3 local)
        {
            const int3 base = AtlasTexel(brick, make_int3(0, 0, 0));
            const float x = base.x + local.x;
            const float y = base.y + local.y;
            const float z = base.z + local.z;
            return make_float3(tex3D<float4>(texture, x + 0.5f, y, z).x,
                               tex3D<float4>(texture, x, y + 0.5f, z).y,
                               tex3D<float4>(texture, x, y, z + 0.5f).z);
        }

        __global__ void WallKernel(const int* brickNodes, const Node* nodes, cudaSurfaceObject_t velocity, int3 dims)
        {
            const int subcell = blockIdx.x;
            const int brick = subcell / kSubcellsPerBrick;
            const int node = brickNodes[brick];
            if (node == kNullNode)
                return;

            const int s = subcell % kSubcellsPerBrick;
            const int3 local = make_int3((s % kSubcellsPerAxis) * kSubcellDim + threadIdx.x,
                                         ((s / kSubcellsPerAxis) % kSubcellsPerAxis) * kSubcellDim + threadIdx.y,
                                         (s / (kSubcellsPerAxis * kSubcellsPerAxis)) * kSubcellDim + threadIdx.z);
            const int3 voxel = BrickVoxel(nodes[node].coord, local);
            const int3 cell = make_int3(voxel.x - kCoordOffset, voxel.y - kCoordOffset, voxel.z - kCoordOffset);
            const int3 depth = make_int3(
                OutsideDepth(cell.x, dims.x), OutsideDepth(cell.y, dims.y), OutsideDepth(cell.z, dims.z));
            if (depth.x + depth.y + depth.z == 0 || depth.x > 1 || depth.y > 1 || depth.z > 1)
                return;

            const int3 texel = AtlasTexel(brick, local);
            const int3 source = make_int3(texel.x + ClampIndex(cell.x, 0, dims.x - 1) - cell.x,
                                          texel.y + ClampIndex(cell.y, 0, dims.y - 1) - cell.y,
                                          texel.z + ClampIndex(cell.z, 0, dims.z - 1) - cell.z);
            float4 v = LoadTexel(velocity, texel);
            float* components[3] = {&v.x, &v.y, &v.z};
            for (int axis = 0; axis < 3; ++axis)
            {
                const int c = Axis(cell, axis);
                const int n = Axis(dims, axis);
                if (c == n)
                    *components[axis] = 0.0f;
                else if (c >= 0 && c < n)
                    *components[axis] = Component(LoadTexel(velocity, source), axis);
            }
            StoreTexel(velocity, texel, v);
        }

        __global__ void AdvectKernel(Tree tree,
                                     GridInfo grid,
                                     ShapeList obstacles,
                                     cudaTextureObject_t velocity,
                                     cudaTextureObject_t velocityOld,
                                     float dt,
                                     float flipRatio,
                                     int count,
                                     float4* positions,
                                     float4* velocities)
        {
            int p = blockIdx.x * blockDim.x + threadIdx.x;
            if (p >= count)
                return;

            const float4 x4 = positions[p];
            float3 x = XYZ(x4);
            float3 vel = XYZ(velocities[p]);
            int brick = 0;
            float3 local;
            if (Locate(tree, grid, x, brick, local))
            {
                const float3 vNew = SampleAtlas(velocity, brick, local);
                const float3 vOld = SampleAtlas(velocityOld, brick, local);
                const float3 flip = vel + vNew - vOld;
                vel = flip * flipRatio + vNew * (1.0f - flipRatio);

                float3 vMid = vNew;
                if (Locate(tree, grid, x + vNew * (0.5f * dt), brick, local))
                    vMid = SampleAtlas(velocity, brick, local);
                x = x + vMid * dt;
            }
            else
            {
                x = x + vel * dt;
            }
            CollideParticle(grid, obstacles, x, vel);

            positions[p] = make_float4(x.x, x.y, x.z, x4.w);
            velocities[p] = make_float4(vel.x, vel.y, vel.z, 0.0f);
        }
    }

    void SparseFlip::Init(const GridInfo& grid, const ShapeList& obstacles)
    {
        m_grid = grid;
        m_obstacles = obstacles;
    }

    void SparseFlip::Release()
    {
        m_cg.Release();
        m_subcells.Release();
        m_atlas.Release();
        m_topology.Release();
    }

    bool SparseFlip::Rebuild(const float4* positions, int count, bool incremental, GpuTimer* timer)
    {
        VoxelSpace space;
        space.origin = m_grid.origin;
        space.h = m_grid.h;
        return incremental ? m_topology.Update(positions, count, space, m_grid.h, timer)
                           : m_topology.FullRebuild(positions, count, space, m_grid.h, timer);
    }

    int SparseFlip::CountUncovered(const float4* positions, int count)
    {
        return m_topology.CountUncovered(positions, count, m_grid.h);
    }

    bool SparseFlip::Transfer(const float4* positions, const float4* velocities, int count, GpuTimer* timer)
    {
        if (!m_atlas.Reserve(m_topology.BrickCapacity()) || !m_atlas.BuildBrickTable(m_topology))
            return false;
        BeginStage(timer, "Atlas clear");
        const bool cleared = m_atlas.ClearActive();
        EndStage(timer);
        return cleared && m_subcells.Build(positions, velocities, count, m_topology, m_grid.h, timer) &&
               SparseParticleToGrid(m_subcells,
                                    m_atlas,
                                    m_topology,
                                    m_obstacles,
                                    DomainVoxels(m_grid),
                                    kLevelSetRadiusScale * m_grid.h,
                                    timer);
    }

    CgResult SparseFlip::Project(float dt, float3 gravity, float tolerance, int maxIterations, GpuTimer* timer)
    {
        return SparseProject(m_atlas, m_topology, m_cg, gravity, dt, tolerance, maxIterations, timer);
    }

    bool SparseFlip::ExtendWalls(AtlasChannel channel)
    {
        const int subcellCount = m_topology.BrickCapacity() * kSubcellsPerBrick;
        if (subcellCount == 0)
            return true;
        WallKernel<<<subcellCount, kSubcellBlock>>>(
            m_atlas.BrickNodes(), m_topology.Nodes(), m_atlas.Surface(channel), m_grid.dims);
        return CUDA_CHECK(cudaGetLastError()) && m_atlas.UpdateApron(channel, m_topology);
    }

    bool SparseFlip::Advect(float4* positions, float4* velocities, int count, float dt, float flipRatio, GpuTimer* timer)
    {
        if (count <= 0)
            return true;

        BeginStage(timer, "Sparse advect");
        bool ok = ExtendWalls(AtlasChannel::Velocity) && ExtendWalls(AtlasChannel::VelocityOld);
        if (ok)
        {
            const Tree tree{m_topology.Nodes(), m_topology.ChildLists(), m_topology.RootLevel()};
            AdvectKernel<<<(count + kBlockSize - 1) / kBlockSize, kBlockSize>>>(tree,
                                                                                m_grid,
                                                                                m_obstacles,
                                                                                m_atlas.Texture(AtlasChannel::Velocity),
                                                                                m_atlas.Texture(AtlasChannel::VelocityOld),
                                                                                dt,
                                                                                flipRatio,
                                                                                count,
                                                                                positions,
                                                                                velocities);
            ok = CUDA_CHECK(cudaGetLastError());
        }
        EndStage(timer);
        return ok;
    }
}
