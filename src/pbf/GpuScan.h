#pragma once
#include <memory>
#include <string>
#include <vector>

#include "graphics/resources/GpuBuffer.h"
#include "graphics/resources/Shader.h"

namespace Pbf
{
    constexpr unsigned int kScanInBinding = 11;
    constexpr unsigned int kScanOutBinding = 12;
    constexpr unsigned int kScanSumsBinding = 13;
    constexpr unsigned int kScanBlockSize = 512;

    class GpuScan
    {
       public:
        bool Init(const std::string& shaderDir);
        bool IsReady() const { return m_ready; }
        void Run(const GpuBuffer& input, GpuBuffer& output, unsigned int count);
        void Destroy();

       private:
        struct Level
        {
            std::unique_ptr<GpuBuffer> sums;
            std::unique_ptr<GpuBuffer> scanned;
        };

        void RunLevel(const GpuBuffer& input, GpuBuffer& output, unsigned int count, size_t level);

        Shader m_scan;
        Shader m_add;
        std::vector<Level> m_levels;
        bool m_ready = false;
    };
}
