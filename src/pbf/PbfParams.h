#pragma once

namespace Pbf
{
    constexpr unsigned int kParamsBinding = 0;
    constexpr unsigned int kPositionsBinding = 0;
    constexpr unsigned int kPredictedBinding = 1;
    constexpr unsigned int kVelocitiesBinding = 2;
    constexpr unsigned int kSortedPositionsBinding = 3;
    constexpr unsigned int kSortedPredictedBinding = 4;
    constexpr unsigned int kSortedVelocitiesBinding = 5;

    struct alignas(16) ParamsGpu
    {
        float domainMin[4] = {0, 0, 0, 0};
        float domainMax[4] = {0, 0, 0, 0};
        float gravity[4] = {0, -9.81f, 0, 0};
        int gridDims[4] = {1, 1, 1, 1};
        float rho0 = 1000.0f;
        float invRho0 = 1.0f / 1000.0f;
        float epsilon = 100.0f;
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
        int pad0 = 0;
        int pad1 = 0;
    };

    static_assert(sizeof(ParamsGpu) == 128, "ParamsGpu must match the std140 PbfParams block");

    struct Settings
    {
        int iterations = 4;
        int substeps = 2;
        float epsilon = 100.0f;
        float sCorrK = 0.1f;
        float sCorrN = 4.0f;
        float sCorrDqFactor = 0.2f;
        float xsphC = 0.01f;
        float vorticityEps = 0.0f;
        float gravity[3] = {0.0f, -9.81f, 0.0f};
    };
}
