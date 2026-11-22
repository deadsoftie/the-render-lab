#include "pch.h"
#include "pbf/PbfSim.h"

#include <algorithm>
#include <cmath>
#include <glad/glad.h>
#include <glm/gtc/constants.hpp>

namespace Pbf
{
    namespace
    {
        constexpr float kRestDensity = 1000.0f;
        constexpr float kJitter = 0.01f;

        Shape ToShape(const PbfShapeBinding& b)
        {
            Shape s;
            s.type = b.type == PbfShapeType::Sphere ? ShapeType::Sphere : ShapeType::Box;
            s.lo = Vec3{b.boxMin.x, b.boxMin.y, b.boxMin.z};
            s.hi = Vec3{b.boxMax.x, b.boxMax.y, b.boxMax.z};
            s.center = Vec3{b.center.x, b.center.y, b.center.z};
            s.radius = b.radius;
            s.velocity = Vec3{b.velocity.x, b.velocity.y, b.velocity.z};
            s.rate = b.rate;
            return s;
        }

        float Poly6(float r2, float h)
        {
            float h2 = h * h;
            if (r2 >= h2)
                return 0.0f;
            float d = h2 - r2;
            return 315.0f / (64.0f * glm::pi<float>() * std::pow(h, 9.0f)) * d * d * d;
        }
    }

    float RestLatticeDensitySum(float radius, float smoothing)
    {
        float spacing = 2.0f * radius;
        int reach = static_cast<int>(std::ceil(smoothing / spacing));
        float sum = 0.0f;
        for (int i = -reach; i <= reach; ++i)
            for (int j = -reach; j <= reach; ++j)
                for (int k = -reach; k <= reach; ++k)
                {
                    float r2 = spacing * spacing * static_cast<float>(i * i + j * j + k * k);
                    sum += Poly6(r2, smoothing);
                }
        return sum;
    }

    bool PbfSim::Load(const PbfBinding& binding)
    {
        Unload();
        m_binding = binding;
        settings.iterations = binding.iterations;
        settings.substeps = binding.substeps;
        settings.epsilon = binding.epsilon;
        settings.vorticityEps = binding.vorticity;
        settings.xsphC = binding.xsph;
        m_maxParticles = binding.maxParticles;
        m_h = 4.0f * binding.particleRadius;

        size_t bytes = static_cast<size_t>(m_maxParticles) * 4 * sizeof(float);
        bool ok = m_paramsBuffer.Resize(sizeof(ParamsGpu)) && m_positions.Resize(bytes) &&
                  m_predicted.Resize(bytes) && m_velocities.Resize(bytes) && m_sortedPositions.Resize(bytes) &&
                  m_sortedPredicted.Resize(bytes) && m_sortedVelocities.Resize(bytes);
        if (!ok)
        {
            m_status = "Failed to allocate particle buffers";
            Unload();
            return false;
        }

        BuildVao();
        Seed();
        m_loaded = true;
        m_status.clear();
        return true;
    }

    void PbfSim::Seed()
    {
        SeedDesc desc;
        desc.domainMin = Vec3{m_binding.domainMin.x, m_binding.domainMin.y, m_binding.domainMin.z};
        desc.domainMax = Vec3{m_binding.domainMax.x, m_binding.domainMax.y, m_binding.domainMax.z};
        desc.radius = m_binding.particleRadius;
        desc.jitter = kJitter;
        desc.maxParticles = m_maxParticles;

        std::vector<Shape> seeds;
        std::vector<Shape> obstacles;
        for (const PbfShapeBinding& s : m_binding.seeds)
            seeds.push_back(ToShape(s));
        for (const PbfShapeBinding& s : m_binding.obstacles)
            obstacles.push_back(ToShape(s));

        std::vector<Particle> particles;
        m_count = SeedLattice(desc, seeds, obstacles, particles);

        std::vector<float> positions(static_cast<size_t>(m_count) * 4);
        std::vector<float> velocities(static_cast<size_t>(m_count) * 4);
        for (int i = 0; i < m_count; ++i)
        {
            for (int c = 0; c < 4; ++c)
            {
                positions[i * 4 + c] = particles[i].position[c];
                velocities[i * 4 + c] = particles[i].velocity[c];
            }
        }
        size_t bytes = positions.size() * sizeof(float);
        m_positions.Upload(positions.data(), bytes);
        m_predicted.Upload(positions.data(), bytes);
        m_velocities.Upload(velocities.data(), bytes);

        UploadParams();
    }

    void PbfSim::UploadParams()
    {
        float h = m_h;
        glm::vec3 extent = m_binding.domainMax - m_binding.domainMin;
        ParamsGpu& p = m_params;
        p.domainMin[0] = m_binding.domainMin.x;
        p.domainMin[1] = m_binding.domainMin.y;
        p.domainMin[2] = m_binding.domainMin.z;
        p.domainMin[3] = h;
        p.domainMax[0] = m_binding.domainMax.x;
        p.domainMax[1] = m_binding.domainMax.y;
        p.domainMax[2] = m_binding.domainMax.z;
        p.domainMax[3] = m_binding.particleRadius;
        p.gravity[0] = settings.gravity[0];
        p.gravity[1] = settings.gravity[1];
        p.gravity[2] = settings.gravity[2];
        p.gravity[3] = 1.0f / (60.0f * static_cast<float>(settings.substeps));
        p.gridDims[0] = std::max(1, static_cast<int>(std::ceil(extent.x / h)));
        p.gridDims[1] = std::max(1, static_cast<int>(std::ceil(extent.y / h)));
        p.gridDims[2] = std::max(1, static_cast<int>(std::ceil(extent.z / h)));
        p.gridDims[3] = p.gridDims[0] * p.gridDims[1] * p.gridDims[2];
        p.rho0 = kRestDensity;
        p.invRho0 = 1.0f / kRestDensity;
        p.epsilon = settings.epsilon;
        p.sCorrK = settings.sCorrK;
        p.sCorrN = settings.sCorrN;
        p.sCorrDq = settings.sCorrDqFactor * h;
        p.xsphC = settings.xsphC;
        p.vorticityEps = settings.vorticityEps;
        p.poly6 = 315.0f / (64.0f * glm::pi<float>() * std::pow(h, 9.0f));
        p.spikyGrad = -45.0f / (glm::pi<float>() * std::pow(h, 6.0f));
        p.wDq = Poly6(p.sCorrDq * p.sCorrDq, h);
        p.mass = kRestDensity / RestLatticeDensitySum(m_binding.particleRadius, h);
        p.particleCount = m_count;
        p.iterations = settings.iterations;
        m_paramsBuffer.Upload(&p, sizeof(ParamsGpu));
    }

    void PbfSim::BuildVao()
    {
        DestroyVao();
        glGenVertexArrays(1, &m_vao);
        glBindVertexArray(m_vao);
        glBindBuffer(GL_ARRAY_BUFFER, m_positions.ID());
        glEnableVertexAttribArray(0);
        glVertexAttribPointer(0, 4, GL_FLOAT, GL_FALSE, 4 * sizeof(float), static_cast<void*>(0));
        glBindBuffer(GL_ARRAY_BUFFER, 0);
        glBindVertexArray(0);
    }

    void PbfSim::DestroyVao()
    {
        if (m_vao != 0)
        {
            glDeleteVertexArrays(1, &m_vao);
            m_vao = 0;
        }
    }

    void PbfSim::Reset()
    {
        if (m_loaded)
            Seed();
    }

    void PbfSim::Update(float deltaSeconds)
    {
        (void)deltaSeconds;
        if (!m_loaded)
            return;
        m_timer.BeginFrame();
    }

    void PbfSim::Unload()
    {
        DestroyVao();
        m_paramsBuffer.Destroy();
        m_positions.Destroy();
        m_predicted.Destroy();
        m_velocities.Destroy();
        m_sortedPositions.Destroy();
        m_sortedPredicted.Destroy();
        m_sortedVelocities.Destroy();
        m_count = 0;
        m_maxParticles = 0;
        m_loaded = false;
    }

    void PbfSim::Shutdown()
    {
        Unload();
        m_timer.Destroy();
    }
}
