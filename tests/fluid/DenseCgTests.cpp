#include "FluidTests.h"
#include "TestFramework.h"
#include "fluid/cuda/DenseCg.h"
#include "fluid/cuda/DeviceBuffer.h"
#include "fluid/cuda/GridMath.cuh"

#include <algorithm>
#include <cmath>
#include <vector>

using namespace Fluid;

namespace
{
    std::vector<double> ApplyA(const GridInfo& grid,
                               const std::vector<unsigned char>& markers,
                               const std::vector<double>& x)
    {
        std::vector<double> out(x.size(), 0.0);
        for (int c = 0; c < CellCount(grid); ++c)
        {
            if (markers[c] != kCellFluid)
                continue;
            int3 ijk = CellCoord(grid, c);
            int nonSolid = 0;
            double neighbors = 0.0;
            for (int axis = 0; axis < 3; ++axis)
            {
                for (int side = -1; side <= 1; side += 2)
                {
                    int i = ijk.x + (axis == 0 ? side : 0);
                    int j = ijk.y + (axis == 1 ? side : 0);
                    int k = ijk.z + (axis == 2 ? side : 0);
                    if (!InGrid(grid, i, j, k))
                        continue;
                    int n = CellIndex(grid, i, j, k);
                    if (markers[n] == kCellSolid)
                        continue;
                    ++nonSolid;
                    if (markers[n] == kCellFluid)
                        neighbors += x[n];
                }
            }
            out[c] = nonSolid * x[c] - neighbors;
        }
        return out;
    }

    double DotProduct(const std::vector<double>& a, const std::vector<double>& b)
    {
        double sum = 0.0;
        for (size_t i = 0; i < a.size(); ++i)
            sum += a[i] * b[i];
        return sum;
    }

    std::vector<double> CpuSolve(const GridInfo& grid,
                                 const std::vector<unsigned char>& markers,
                                 const std::vector<double>& b)
    {
        std::vector<double> x(b.size(), 0.0);
        std::vector<double> r = b;
        std::vector<double> d = r;
        double rr = DotProduct(r, r);
        const double threshold = 1e-24 * rr;
        for (int it = 0; it < 10000 && rr > threshold; ++it)
        {
            std::vector<double> q = ApplyA(grid, markers, d);
            double alpha = rr / DotProduct(d, q);
            for (size_t i = 0; i < x.size(); ++i)
            {
                x[i] += alpha * d[i];
                r[i] -= alpha * q[i];
            }
            double rrNew = DotProduct(r, r);
            double beta = rrNew / rr;
            rr = rrNew;
            for (size_t i = 0; i < d.size(); ++i)
                d[i] = r[i] + beta * d[i];
        }
        return x;
    }
}

void RunDenseCgTests()
{
    GridInfo grid;
    grid.dims = make_int3(8, 8, 8);
    grid.origin = make_float3(0.0f, 0.0f, 0.0f);
    grid.h = 1.0f;
    const int cells = CellCount(grid);

    std::vector<unsigned char> markers(cells, kCellFluid);
    std::vector<float> b(cells, 0.0f);
    std::vector<double> bDouble(cells, 0.0);
    unsigned int seed = 12345u;
    for (int c = 0; c < cells; ++c)
    {
        int3 ijk = CellCoord(grid, c);
        if (ijk.y >= 6)
            markers[c] = kCellAir;
        else if (ijk.x == 3 && ijk.z == 3 && ijk.y < 4)
            markers[c] = kCellSolid;
        if (markers[c] != kCellFluid)
            continue;
        seed = seed * 1664525u + 1013904223u;
        b[c] = static_cast<float>(seed >> 8) / 16777216.0f * 2.0f - 1.0f;
        bDouble[c] = b[c];
    }

    DeviceBuffer<unsigned char> deviceMarkers;
    DeviceBuffer<float> deviceB;
    DeviceBuffer<float> deviceX(cells);
    CHECK(deviceMarkers.Upload(markers));
    CHECK(deviceB.Upload(b));

    DenseCg cg;
    CgResult result =
        cg.Solve(grid, deviceMarkers.Data(), deviceB.Data(), deviceX.Data(), 1e-5f, 1000);
    CHECK(result.converged);
    CHECK(result.iterations > 0 && result.iterations <= 1000);
    CHECK(result.relativeResidual <= 1e-5f);

    std::vector<float> x;
    CHECK(deviceX.Download(x));
    std::vector<double> xDouble(x.begin(), x.end());
    std::vector<double> reference = CpuSolve(grid, markers, bDouble);

    double maxError = 0.0;
    double maxReference = 0.0;
    for (int c = 0; c < cells; ++c)
    {
        maxError = std::max(maxError, std::abs(xDouble[c] - reference[c]));
        maxReference = std::max(maxReference, std::abs(reference[c]));
        if (markers[c] != kCellFluid)
            CHECK(x[c] == 0.0f);
    }
    CHECK(maxReference > 0.0);
    CHECK(maxError <= 1e-3 * maxReference);

    std::vector<double> ax = ApplyA(grid, markers, xDouble);
    double residual = 0.0;
    for (int c = 0; c < cells; ++c)
        residual += (bDouble[c] - ax[c]) * (bDouble[c] - ax[c]);
    CHECK(std::sqrt(residual / DotProduct(bDouble, bDouble)) <= 1e-3);

    std::vector<float> zeros(cells, 0.0f);
    CHECK(deviceB.Upload(zeros));
    CgResult trivial = cg.Solve(grid, deviceMarkers.Data(), deviceB.Data(), deviceX.Data(), 1e-5f, 100);
    CHECK(trivial.converged && trivial.iterations == 0);
}
