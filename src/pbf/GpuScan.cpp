#include "pch.h"
#include "pbf/GpuScan.h"

#include <glad/glad.h>

namespace Pbf
{
    bool GpuScan::Init(const std::string& shaderDir)
    {
        m_ready = m_scan.LoadComputeFromFile(shaderDir + "pbf_scan.comp") &&
                  m_add.LoadComputeFromFile(shaderDir + "pbf_scan_add.comp");
        return m_ready;
    }

    void GpuScan::Run(const GpuBuffer& input, GpuBuffer& output, unsigned int count)
    {
        if (!m_ready || count == 0)
            return;
        RunLevel(input, output, count, 0);
    }

    void GpuScan::RunLevel(const GpuBuffer& input, GpuBuffer& output, unsigned int count, size_t level)
    {
        unsigned int groups = (count + kScanBlockSize - 1) / kScanBlockSize;
        if (m_levels.size() <= level)
            m_levels.resize(level + 1);
        Level& lv = m_levels[level];
        if (!lv.sums)
        {
            lv.sums = std::make_unique<GpuBuffer>();
            lv.scanned = std::make_unique<GpuBuffer>();
        }
        lv.sums->Resize(static_cast<size_t>(groups) * sizeof(unsigned int));
        lv.scanned->Resize(static_cast<size_t>(groups) * sizeof(unsigned int));

        m_scan.Bind();
        m_scan.SetInt("uCount", static_cast<int>(count));
        input.BindBase(GL_SHADER_STORAGE_BUFFER, kScanInBinding);
        output.BindBase(GL_SHADER_STORAGE_BUFFER, kScanOutBinding);
        lv.sums->BindBase(GL_SHADER_STORAGE_BUFFER, kScanSumsBinding);
        glDispatchCompute(groups, 1, 1);
        glMemoryBarrier(GL_SHADER_STORAGE_BARRIER_BIT);

        if (groups <= 1)
            return;

        RunLevel(*lv.sums, *lv.scanned, groups, level + 1);

        Level& again = m_levels[level];
        m_add.Bind();
        m_add.SetInt("uCount", static_cast<int>(count));
        output.BindBase(GL_SHADER_STORAGE_BUFFER, kScanOutBinding);
        again.scanned->BindBase(GL_SHADER_STORAGE_BUFFER, kScanSumsBinding);
        glDispatchCompute(groups, 1, 1);
        glMemoryBarrier(GL_SHADER_STORAGE_BARRIER_BIT);
    }

    void GpuScan::Destroy()
    {
        m_levels.clear();
    }
}
