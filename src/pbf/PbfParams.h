#pragma once

namespace Pbf
{
    constexpr unsigned int kParamsBinding = 0;
    constexpr unsigned int kShapesBinding = 1;
    constexpr unsigned int kPositionsBinding = 0;
    constexpr unsigned int kPredictedBinding = 1;
    constexpr unsigned int kVelocitiesBinding = 2;
    constexpr unsigned int kSortedPositionsBinding = 3;
    constexpr unsigned int kSortedPredictedBinding = 4;
    constexpr unsigned int kSortedVelocitiesBinding = 5;
    constexpr unsigned int kDeltaBinding = 3;
    constexpr int kMaxShapes = 8;
    constexpr unsigned int kBodiesBinding = 3;
    constexpr unsigned int kBodyImpulseBinding = 15;
    constexpr int kMaxBodies = 16;
    constexpr int kImpulseComponents = 6;
    constexpr float kImpulseScale = 1.0e6f;

    struct alignas(16) ParamsGpu
    {
        float domainMin[4] = {0, 0, 0, 0};
        float domainMax[4] = {0, 0, 0, 0};
        float gravity[4] = {0, -9.81f, 0, 0};
        int gridDims[4] = {1, 1, 1, 1};
        float rho0 = 1000.0f;
        float invRho0 = 1.0f / 1000.0f;
        float epsilon = 0.0f;
        float sCorrK = 0.1f;
        float sCorrDq = 0.0f;
        float sCorrN = 4.0f;
        float xsphC = 0.01f;
        float vorticityEps = 0.0f;
        float poly6 = 0.0f;
        float spikyGrad = 0.0f;
        float wDq = 1.0f;
        float mass = 1.0f;
        int particleCount = 0;
        int iterations = 4;
        float lambdaScale = 1.0f;
        int pad1 = 0;
    };

    static_assert(sizeof(ParamsGpu) == 128, "ParamsGpu must match the std140 PbfParams block");

    struct alignas(16) ShapesGpu
    {
        int count[4] = {0, 0, 0, 0};
        float a[kMaxShapes][4] = {};
        float b[kMaxShapes][4] = {};
    };

    static_assert(sizeof(ShapesGpu) == 16 + 2 * kMaxShapes * 16, "ShapesGpu must match the std140 PbfShapes block");

    struct alignas(16) BodiesGpu
    {
        int info[4] = {0, 0, 0, 0};
        float params[4] = {kImpulseScale, 0.0f, 0.0f, 0.0f};
        float com[kMaxBodies][4] = {};
        float cObj[kMaxBodies][4] = {};
        float axisX[kMaxBodies][4] = {};
        float axisY[kMaxBodies][4] = {};
        float axisZ[kMaxBodies][4] = {};
        float halfExtents[kMaxBodies][4] = {};
        float velocity[kMaxBodies][4] = {};
        float omega[kMaxBodies][4] = {};
    };

    static_assert(sizeof(BodiesGpu) == 32 + 8 * kMaxBodies * 16, "BodiesGpu must match the std140 PbfBodies block");

    struct Settings
    {
        int iterations = 4;
        int substeps = 2;
        float epsilon = 0.05f;
        float sCorrK = 0.1f;
        float sCorrN = 4.0f;
        float sCorrDqFactor = 0.2f;
        float xsphC = 0.01f;
        float vorticityEps = 0.0f;
        float gravity[3] = {0.0f, -9.81f, 0.0f};
    };
}
