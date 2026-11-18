#include "fluid/cuda/DenseCg.h"

#include <cmath>

#include "fluid/cuda/CudaCheck.h"
#include "fluid/cuda/GridMath.cuh"
#include "fluid/cuda/Reduce.cuh"

namespace Fluid
{
    namespace
    {
        constexpr int kBlockSize = 256;
        constexpr int kCheckInterval = 10;
        constexpr int kDq = 0;
        constexpr int kRrOld = 1;
        constexpr int kRrNew = 2;
        constexpr int kScalarCount = 3;

        int Blocks(int n)
        {
            return (n + kBlockSize - 1) / kBlockSize;
        }

        __global__ void InitKernel(int cells,
                                   const unsigned char* markers,
                                   const float* b,
                                   float* x,
                                   float* r,
                                   float* d,
                                   double* scalars)
        {
            int c = blockIdx.x * blockDim.x + threadIdx.x;
            double rr = 0.0;
            if (c < cells)
            {
                float value = markers[c] == kCellFluid ? b[c] : 0.0f;
                x[c] = 0.0f;
                r[c] = value;
                d[c] = value;
                rr = static_cast<double>(value) * value;
            }
            BlockSumAtomic<kBlockSize>(rr, scalars + kRrOld);
        }

        __global__ void SpMVDotKernel(GridInfo grid,
                                      const unsigned char* markers,
                                      const float* d,
                                      float* q,
                                      double* scalars)
        {
            int c = blockIdx.x * blockDim.x + threadIdx.x;
            double dq = 0.0;
            if (c < CellCount(grid))
            {
                float result = 0.0f;
                if (markers[c] == kCellFluid)
                {
                    int3 ijk = CellCoord(grid, c);
                    int nonSolid = 0;
                    float neighbors = 0.0f;
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
                            unsigned char marker = markers[n];
                            if (marker == kCellSolid)
                                continue;
                            ++nonSolid;
                            if (marker == kCellFluid)
                                neighbors += d[n];
                        }
                    }
                    result = nonSolid * d[c] - neighbors;
                    dq = static_cast<double>(d[c]) * result;
                }
                q[c] = result;
            }
            BlockSumAtomic<kBlockSize>(dq, scalars + kDq);
        }

        __global__ void UpdateXRKernel(int cells,
                                       const float* d,
                                       const float* q,
                                       float* x,
                                       float* r,
                                       double* scalars)
        {
            int c = blockIdx.x * blockDim.x + threadIdx.x;
            double dq = scalars[kDq];
            float alpha = dq > 0.0 ? static_cast<float>(scalars[kRrOld] / dq) : 0.0f;
            double rr = 0.0;
            if (c < cells)
            {
                x[c] += alpha * d[c];
                float value = r[c] - alpha * q[c];
                r[c] = value;
                rr = static_cast<double>(value) * value;
            }
            BlockSumAtomic<kBlockSize>(rr, scalars + kRrNew);
        }

        __global__ void UpdateDKernel(int cells, const float* r, float* d, const double* scalars)
        {
            int c = blockIdx.x * blockDim.x + threadIdx.x;
            if (c >= cells)
                return;
            double rrOld = scalars[kRrOld];
            float beta = rrOld > 0.0 ? static_cast<float>(scalars[kRrNew] / rrOld) : 0.0f;
            d[c] = r[c] + beta * d[c];
        }

        __global__ void RollKernel(double* scalars)
        {
            scalars[kRrOld] = scalars[kRrNew];
            scalars[kRrNew] = 0.0;
            scalars[kDq] = 0.0;
        }
    }

    CgResult DenseCg::Solve(const GridInfo& grid,
                            const unsigned char* markers,
                            const float* b,
                            float* x,
                            float tolerance,
                            int maxIterations,
                            cudaStream_t stream)
    {
        CgResult result;
        const int cells = CellCount(grid);
        if (cells <= 0 || !Reserve(cells) || !m_scalars.Clear(stream))
            return result;

        InitKernel<<<Blocks(cells), kBlockSize, 0, stream>>>(
            cells, markers, b, x, m_r.Data(), m_d.Data(), m_scalars.Data());

        double rr0 = 0.0;
        if (!CUDA_CHECK(cudaGetLastError()) || !ReadScalar(kRrOld, rr0, stream))
            return result;
        if (rr0 <= 0.0)
        {
            result.converged = true;
            return result;
        }

        const double threshold = static_cast<double>(tolerance) * tolerance * rr0;
        double rr = rr0;
        int iteration = 0;
        while (iteration < maxIterations)
        {
            SpMVDotKernel<<<Blocks(cells), kBlockSize, 0, stream>>>(
                grid, markers, m_d.Data(), m_q.Data(), m_scalars.Data());
            UpdateXRKernel<<<Blocks(cells), kBlockSize, 0, stream>>>(
                cells, m_d.Data(), m_q.Data(), x, m_r.Data(), m_scalars.Data());
            UpdateDKernel<<<Blocks(cells), kBlockSize, 0, stream>>>(
                cells, m_r.Data(), m_d.Data(), m_scalars.Data());
            RollKernel<<<1, 1, 0, stream>>>(m_scalars.Data());
            ++iteration;

            if (iteration % kCheckInterval == 0 || iteration == maxIterations)
            {
                if (!CUDA_CHECK(cudaGetLastError()) || !ReadScalar(kRrOld, rr, stream))
                    break;
                if (rr <= threshold)
                {
                    result.converged = true;
                    break;
                }
            }
        }

        result.iterations = iteration;
        result.relativeResidual = static_cast<float>(std::sqrt(rr / rr0));
        return result;
    }

    void DenseCg::Release()
    {
        m_r.Free();
        m_d.Free();
        m_q.Free();
        m_scalars.Free();
    }

    bool DenseCg::Reserve(int cells)
    {
        const size_t count = static_cast<size_t>(cells);
        return m_r.Resize(count) && m_d.Resize(count) && m_q.Resize(count) &&
               m_scalars.Resize(kScalarCount);
    }

    bool DenseCg::ReadScalar(int index, double& value, cudaStream_t stream) const
    {
        return CUDA_CHECK(cudaMemcpyAsync(&value,
                                          m_scalars.Data() + index,
                                          sizeof(double),
                                          cudaMemcpyDeviceToHost,
                                          stream)) &&
               CUDA_CHECK(cudaStreamSynchronize(stream));
    }
}
