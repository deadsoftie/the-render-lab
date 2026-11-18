#pragma once
#include <cuda_runtime.h>

namespace Fluid
{
    constexpr unsigned char kCellAir = 0;
    constexpr unsigned char kCellFluid = 1;
    constexpr unsigned char kCellSolid = 2;
    constexpr int kMaxShapes = 8;

    enum class ShapeType : int
    {
        Box = 0,
        Sphere = 1
    };

    struct Shape
    {
        ShapeType type = ShapeType::Box;
        float3 lo{0.0f, 0.0f, 0.0f};
        float3 hi{0.0f, 0.0f, 0.0f};
        float3 center{0.0f, 0.0f, 0.0f};
        float radius = 0.0f;
        float3 velocity{0.0f, 0.0f, 0.0f};
    };

    struct ShapeList
    {
        int count = 0;
        Shape items[kMaxShapes];
    };

    struct GridInfo
    {
        int3 dims{0, 0, 0};
        float3 origin{0.0f, 0.0f, 0.0f};
        float h = 0.0f;
    };

    __host__ __device__ inline int CellCount(const GridInfo& grid)
    {
        return grid.dims.x * grid.dims.y * grid.dims.z;
    }

    struct StepSettings
    {
        float flipRatio = 0.95f;
        float cgTolerance = 1e-4f;
        int cgMaxIterations = 200;
        float3 gravity{0.0f, -9.81f, 0.0f};
    };

    struct CgResult
    {
        int iterations = 0;
        float relativeResidual = 0.0f;
        bool converged = false;
    };

    struct StepStats
    {
        int substeps = 0;
        float maxSpeed = 0.0f;
        int cgIterationsTotal = 0;
        CgResult cg;
    };
}
