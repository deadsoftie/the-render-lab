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
        constexpr unsigned int kWorkgroupSize = 256;
        constexpr int kStatsInterval = 30;
        constexpr float kFrameSeconds = 1.0f / 60.0f;
        constexpr int kMaxFramesPerUpdate = 2;
        constexpr GLuint64 kForceWaitNs = 4000000;
        constexpr size_t kImpulseBytes = static_cast<size_t>(kMaxBodies) * kImpulseComponents * sizeof(int);
        constexpr GLbitfield kComputeBarrier =
            GL_SHADER_STORAGE_BARRIER_BIT | GL_VERTEX_ATTRIB_ARRAY_BARRIER_BIT | GL_BUFFER_UPDATE_BARRIER_BIT;

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

        float SpikyGradMagnitude(float r, float h)
        {
            if (r >= h || r <= 0.0f)
                return 0.0f;
            float d = h - r;
            return 45.0f / (glm::pi<float>() * std::pow(h, 6.0f)) * d * d;
        }

        void Put(float (&dst)[4], const glm::vec3& v, float w = 0.0f)
        {
            dst[0] = v.x;
            dst[1] = v.y;
            dst[2] = v.z;
            dst[3] = w;
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

    float RestLatticeGradientSum(float radius, float smoothing)
    {
        float spacing = 2.0f * radius;
        int reach = static_cast<int>(std::ceil(smoothing / spacing));
        float sum = 0.0f;
        for (int i = -reach; i <= reach; ++i)
            for (int j = -reach; j <= reach; ++j)
                for (int k = -reach; k <= reach; ++k)
                {
                    if (i == 0 && j == 0 && k == 0)
                        continue;
                    float r = spacing * std::sqrt(static_cast<float>(i * i + j * j + k * k));
                    float g = SpikyGradMagnitude(r, smoothing);
                    sum += g * g;
                }
        return sum;
    }

    bool PbfSim::LoadShaders()
    {
        if (m_shadersLoaded)
            return true;
        m_shadersLoaded = m_scan.Init(m_shaderDir) &&
                          m_cellCountShader.LoadComputeFromFile(m_shaderDir + "pbf_cell_count.comp") &&
                          m_reorderShader.LoadComputeFromFile(m_shaderDir + "pbf_reorder.comp") &&
                          m_neighborStatsShader.LoadComputeFromFile(m_shaderDir + "pbf_neighbor_stats.comp") &&
                          m_predictShader.LoadComputeFromFile(m_shaderDir + "pbf_predict.comp") &&
                          m_lambdaShader.LoadComputeFromFile(m_shaderDir + "pbf_lambda.comp") &&
                          m_deltaShader.LoadComputeFromFile(m_shaderDir + "pbf_delta.comp") &&
                          m_applyShader.LoadComputeFromFile(m_shaderDir + "pbf_apply.comp") &&
                          m_updateShader.LoadComputeFromFile(m_shaderDir + "pbf_update.comp") &&
                          m_vorticityShader.LoadComputeFromFile(m_shaderDir + "pbf_vorticity.comp") &&
                          m_velocityShader.LoadComputeFromFile(m_shaderDir + "pbf_velocity.comp") &&
                          m_mouseShader.LoadComputeFromFile(m_shaderDir + "pbf_mouse.comp") &&
                          m_columnsShader.LoadComputeFromFile(m_shaderDir + "pbf_columns.comp");
        return m_shadersLoaded;
    }

    bool PbfSim::Load(const PbfBinding& binding)
    {
        Unload();
        GLint bindings = 0;
        GLint blocks = 0;
        glGetIntegerv(GL_MAX_SHADER_STORAGE_BUFFER_BINDINGS, &bindings);
        glGetIntegerv(GL_MAX_COMPUTE_SHADER_STORAGE_BLOCKS, &blocks);
        if (bindings < 16 || blocks < 10)
        {
            m_status = "GPU exposes too few shader storage buffers for the fluid (needs 16 bindings, 10 per compute shader)";
            return false;
        }
        if (!LoadShaders())
        {
            m_status = "Failed to compile PBF compute shaders";
            return false;
        }

        m_binding = binding;
        settings.iterations = binding.iterations;
        settings.substeps = binding.substeps;
        settings.epsilon = binding.epsilon;
        settings.vorticityEps = binding.vorticity;
        settings.xsphC = binding.xsph;
        m_maxParticles = binding.maxParticles;
        m_h = 4.0f * binding.particleRadius;

        glm::vec3 extent = binding.domainMax - binding.domainMin;
        int cells = std::max(1, static_cast<int>(std::ceil(extent.x / m_h))) *
                    std::max(1, static_cast<int>(std::ceil(extent.y / m_h))) *
                    std::max(1, static_cast<int>(std::ceil(extent.z / m_h)));

        size_t vec4Bytes = static_cast<size_t>(m_maxParticles) * 4 * sizeof(float);
        size_t uintBytes = static_cast<size_t>(m_maxParticles) * sizeof(unsigned int);
        size_t cellBytes = static_cast<size_t>(cells) * sizeof(unsigned int);
        m_columnsX = std::max(1, static_cast<int>(std::ceil(extent.x / m_h)));
        m_columnsZ = std::max(1, static_cast<int>(std::ceil(extent.z / m_h)));
        size_t columnBytes = static_cast<size_t>(m_columnsX) * m_columnsZ * 4 * sizeof(float);
        m_columnData.clear();
        m_columnsValid = false;
        bool ok = m_paramsBuffer.Resize(sizeof(ParamsGpu)) && m_shapesBuffer.Resize(sizeof(ShapesGpu));
        for (int set = 0; set < 2; ++set)
            for (int a = 0; a < kAttributeCount; ++a)
                ok = ok && m_particles[set][a].Resize(vec4Bytes);
        ok = ok && m_cellCounts.Resize(cellBytes) && m_cellStarts.Resize(cellBytes) &&
             m_particleCells.Resize(uintBytes) && m_particleRanks.Resize(uintBytes) &&
             m_neighborCounts.Resize(uintBytes) && m_stats.Resize(4 * sizeof(unsigned int)) &&
             m_bodiesBuffer.Resize(sizeof(BodiesGpu)) && m_bodyImpulses.Resize(kImpulseBytes) &&
             m_columns.Resize(columnBytes);
        for (ImpulseSlot& slot : m_impulseSlots)
            ok = ok && slot.buffer.Resize(kImpulseBytes) && slot.columns.Resize(columnBytes);
        if (!ok)
        {
            m_status = "Failed to allocate particle buffers";
            Unload();
            return false;
        }

        glGenVertexArrays(1, &m_vao);
        glBindVertexArray(m_vao);
        glEnableVertexAttribArray(0);
        glVertexAttribFormat(0, 4, GL_FLOAT, GL_FALSE, 0);
        glVertexAttribBinding(0, 0);
        glBindVertexArray(0);

        m_current = 0;
        UploadShapes();
        BuildEmitters();
        m_loaded = true;
        m_bodiesGpu = BodiesGpu{};
        m_bodyCount = 0;
        m_bodyForces.clear();
        SetBodies({});
        ClearBodyImpulses();
        Seed();
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
        SeedLattice(desc, seeds, obstacles, particles);
        UploadParticles(particles);
    }

    void PbfSim::BuildEmitters()
    {
        m_emitters.clear();
        SeedDesc desc;
        desc.domainMin = Vec3{m_binding.domainMin.x, m_binding.domainMin.y, m_binding.domainMin.z};
        desc.domainMax = Vec3{m_binding.domainMax.x, m_binding.domainMax.y, m_binding.domainMax.z};
        desc.radius = m_binding.particleRadius;
        desc.jitter = kJitter;
        desc.maxParticles = m_maxParticles;
        float spacing = 2.0f * m_binding.particleRadius;

        for (const PbfShapeBinding& s : m_binding.emitters)
        {
            Emitter e;
            SeedLattice(desc, {ToShape(s)}, {}, e.layer);
            if (e.layer.empty())
                continue;
            float speed = glm::length(s.velocity);
            float byRate = s.rate > 0.0f ? static_cast<float>(e.layer.size()) / s.rate : 0.0f;
            float bySpeed = speed > 1e-4f ? spacing / speed : 0.0f;
            e.interval = std::max(byRate, bySpeed);
            if (e.interval <= 0.0f)
                continue;
            m_emitters.push_back(std::move(e));
        }
    }

    void PbfSim::AppendParticles(const std::vector<Particle>& particles)
    {
        int n = std::min(static_cast<int>(particles.size()), m_maxParticles - m_count);
        if (n <= 0)
            return;
        std::vector<float> positions(static_cast<size_t>(n) * 4);
        std::vector<float> velocities(static_cast<size_t>(n) * 4);
        for (int i = 0; i < n; ++i)
        {
            for (int c = 0; c < 4; ++c)
            {
                positions[i * 4 + c] = particles[i].position[c];
                velocities[i * 4 + c] = particles[i].velocity[c];
            }
        }
        size_t offset = static_cast<size_t>(m_count) * 4 * sizeof(float);
        size_t bytes = positions.size() * sizeof(float);
        m_particles[m_current][kPosition].Upload(positions.data(), bytes, offset);
        m_particles[m_current][kPredicted].Upload(positions.data(), bytes, offset);
        m_particles[m_current][kVelocity].Upload(velocities.data(), bytes, offset);
        m_count += n;
        m_params.particleCount = m_count;
        m_paramsBuffer.Upload(&m_params, sizeof(ParamsGpu));
    }

    int PbfSim::EmitFrame(float seconds)
    {
        int before = m_count;
        for (Emitter& e : m_emitters)
        {
            e.timer += seconds;
            while (e.timer >= e.interval && m_count < m_maxParticles)
            {
                e.timer -= e.interval;
                AppendParticles(e.layer);
            }
            if (m_count >= m_maxParticles)
                e.timer = 0.0f;
        }
        return m_count - before;
    }

    void PbfSim::UploadParticles(const std::vector<Particle>& particles)
    {
        m_count = std::min(static_cast<int>(particles.size()), m_maxParticles);
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
        m_current = 0;
        m_accumulator = 0.0f;
        m_simSeconds = 0.0f;
        for (Emitter& e : m_emitters)
            e.timer = 0.0f;
        size_t bytes = positions.size() * sizeof(float);
        m_particles[0][kPosition].Upload(positions.data(), bytes);
        m_particles[0][kPredicted].Upload(positions.data(), bytes);
        m_particles[0][kVelocity].Upload(velocities.data(), bytes);
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
        p.sCorrK = settings.sCorrK;
        p.sCorrN = settings.sCorrN;
        p.sCorrDq = settings.sCorrDqFactor * h;
        p.xsphC = settings.xsphC;
        p.vorticityEps = settings.vorticityEps;
        p.poly6 = 315.0f / (64.0f * glm::pi<float>() * std::pow(h, 9.0f));
        p.spikyGrad = -45.0f / (glm::pi<float>() * std::pow(h, 6.0f));
        p.wDq = Poly6(p.sCorrDq * p.sCorrDq, h);
        p.mass = kRestDensity / RestLatticeDensitySum(m_binding.particleRadius, h);
        float gradScale = p.mass / kRestDensity;
        float restDenominator = gradScale * gradScale * RestLatticeGradientSum(m_binding.particleRadius, h);
        p.epsilon = settings.epsilon * restDenominator;
        p.lambdaScale = restDenominator > 0.0f ? 1.0f / restDenominator : 0.0f;
        p.particleCount = m_count;
        p.iterations = settings.iterations;
        m_paramsBuffer.Upload(&p, sizeof(ParamsGpu));
    }

    void PbfSim::UploadShapes()
    {
        ShapesGpu shapes;
        int n = 0;
        for (const PbfShapeBinding& s : m_binding.obstacles)
        {
            if (n >= kMaxShapes)
                break;
            if (s.type == PbfShapeType::Sphere)
            {
                shapes.a[n][0] = s.center.x;
                shapes.a[n][1] = s.center.y;
                shapes.a[n][2] = s.center.z;
                shapes.a[n][3] = 1.0f;
                shapes.b[n][0] = s.radius;
            }
            else
            {
                shapes.a[n][0] = s.boxMin.x;
                shapes.a[n][1] = s.boxMin.y;
                shapes.a[n][2] = s.boxMin.z;
                shapes.a[n][3] = 0.0f;
                shapes.b[n][0] = s.boxMax.x;
                shapes.b[n][1] = s.boxMax.y;
                shapes.b[n][2] = s.boxMax.z;
            }
            ++n;
        }
        shapes.count[0] = n;
        m_shapesBuffer.Upload(&shapes, sizeof(ShapesGpu));
    }

    void PbfSim::BindParticles(int set, unsigned int firstBinding) const
    {
        for (int a = 0; a < kAttributeCount; ++a)
            m_particles[set][a].BindBase(GL_SHADER_STORAGE_BUFFER, firstBinding + a);
    }

    void PbfSim::BindCommon() const
    {
        m_paramsBuffer.BindBase(GL_UNIFORM_BUFFER, kParamsBinding);
        m_shapesBuffer.BindBase(GL_UNIFORM_BUFFER, kShapesBinding);
        m_cellCounts.BindBase(GL_SHADER_STORAGE_BUFFER, kCellCountBinding);
        m_cellStarts.BindBase(GL_SHADER_STORAGE_BUFFER, kCellStartBinding);
        m_particleCells.BindBase(GL_SHADER_STORAGE_BUFFER, kParticleCellBinding);
        m_particleRanks.BindBase(GL_SHADER_STORAGE_BUFFER, kParticleRankBinding);
        m_stats.BindBase(GL_SHADER_STORAGE_BUFFER, kStatsBinding);
        m_neighborCounts.BindBase(GL_SHADER_STORAGE_BUFFER, kNeighborCountBinding);
        m_bodiesBuffer.BindBase(GL_UNIFORM_BUFFER, kBodiesBinding);
        m_bodyImpulses.BindBase(GL_SHADER_STORAGE_BUFFER, kBodyImpulseBinding);
    }

    void PbfSim::DispatchParticles(const Shader& shader) const
    {
        shader.Bind();
        unsigned int groups = (static_cast<unsigned int>(m_count) + kWorkgroupSize - 1) / kWorkgroupSize;
        glDispatchCompute(std::max(groups, 1u), 1, 1);
        glMemoryBarrier(kComputeBarrier);
    }

    void PbfSim::DispatchWithBodyTime(const Shader& shader) const
    {
        shader.Bind();
        shader.SetFloat("uBodyTime", m_bodyTime);
        DispatchParticles(shader);
    }

    void PbfSim::BuildGrid()
    {
        if (!m_loaded || m_count == 0)
            return;

        BindCommon();
        BindParticles(m_current, kPositionsBinding);
        m_cellCounts.Clear();
        glMemoryBarrier(kComputeBarrier);

        DispatchParticles(m_cellCountShader);
        m_scan.Run(m_cellCounts, m_cellStarts, static_cast<unsigned int>(CellCount()));

        BindCommon();
        BindParticles(m_current, kPositionsBinding);
        BindParticles(1 - m_current, kSortedPositionsBinding);
        DispatchParticles(m_reorderShader);
        m_current = 1 - m_current;
    }

    void PbfSim::Predict()
    {
        if (!m_loaded || m_count == 0)
            return;
        BindCommon();
        BindParticles(m_current, kPositionsBinding);
        DispatchWithBodyTime(m_predictShader);
    }

    void PbfSim::ComputeLambda()
    {
        if (!m_loaded || m_count == 0)
            return;
        BindCommon();
        BindParticles(m_current, kPositionsBinding);
        DispatchParticles(m_lambdaShader);
    }

    void PbfSim::ApplyDelta()
    {
        if (!m_loaded || m_count == 0)
            return;
        BindCommon();
        BindParticles(m_current, kPositionsBinding);
        m_particles[1 - m_current][kPosition].BindBase(GL_SHADER_STORAGE_BUFFER, kDeltaBinding);
        DispatchParticles(m_deltaShader);
        DispatchWithBodyTime(m_applyShader);
    }

    void PbfSim::UpdateVelocities()
    {
        if (!m_loaded || m_count == 0)
            return;
        BindCommon();
        BindParticles(m_current, kPositionsBinding);
        DispatchWithBodyTime(m_updateShader);
    }

    void PbfSim::ApplyVelocityPasses()
    {
        if (!m_loaded || m_count == 0 || (m_params.vorticityEps <= 0.0f && m_params.xsphC <= 0.0f))
            return;
        BindCommon();
        BindParticles(m_current, kPositionsBinding);
        m_particles[1 - m_current][kPredicted].BindBase(GL_SHADER_STORAGE_BUFFER, kSortedPredictedBinding);
        m_particles[1 - m_current][kVelocity].BindBase(GL_SHADER_STORAGE_BUFFER, kSortedVelocitiesBinding);
        if (m_params.vorticityEps > 0.0f)
            DispatchParticles(m_vorticityShader);
        DispatchParticles(m_velocityShader);

        glBindBuffer(GL_COPY_READ_BUFFER, m_particles[1 - m_current][kVelocity].ID());
        glBindBuffer(GL_COPY_WRITE_BUFFER, m_particles[m_current][kVelocity].ID());
        glCopyBufferSubData(GL_COPY_READ_BUFFER,
                            GL_COPY_WRITE_BUFFER,
                            0,
                            0,
                            static_cast<GLsizeiptr>(m_count) * 4 * sizeof(float));
        glBindBuffer(GL_COPY_READ_BUFFER, 0);
        glBindBuffer(GL_COPY_WRITE_BUFFER, 0);
    }

    void PbfSim::ApplyMouseForce(float scale)
    {
        if (!m_loaded || m_count == 0 || !mouse.active || scale <= 0.0f)
            return;
        BindCommon();
        BindParticles(m_current, kPositionsBinding);
        m_mouseShader.Bind();
        m_mouseShader.SetVec3("uRayOrigin", mouse.origin);
        m_mouseShader.SetVec3("uRayDir", glm::normalize(mouse.direction));
        m_mouseShader.SetFloat("uRadius", mouse.radius);
        m_mouseShader.SetVec3("uImpulse", mouse.impulse * scale);
        DispatchParticles(m_mouseShader);
    }

    void PbfSim::Substep()
    {
        m_timer.Begin("Predict");
        Predict();
        m_timer.Begin("Neighbor grid");
        BuildGrid();
        m_timer.Begin("Solve");
        for (int it = 0; it < settings.iterations; ++it)
        {
            ComputeLambda();
            ApplyDelta();
        }
        m_timer.Begin("Update");
        UpdateVelocities();
        m_timer.Begin("Vorticity + XSPH");
        ApplyVelocityPasses();
        m_timer.End();
        m_simSeconds += TimeStep();
    }

    void PbfSim::StepFrame()
    {
        if (emittersEnabled)
            EmitFrame(kFrameSeconds);
        ApplyMouseForce(std::min(m_mouseSeconds / kFrameSeconds, static_cast<float>(kMaxFramesPerUpdate)));
        m_mouseSeconds = 0.0f;
        for (int s = 0; s < settings.substeps; ++s)
        {
            m_bodyTime = m_bodyFrameOffset - static_cast<float>(settings.substeps - 1 - s) * TimeStep();
            Substep();
        }
        m_bodyTime = 0.0f;
    }

    void PbfSim::SetBodies(const std::vector<CouplingBody>& bodies)
    {
        int n = std::min(static_cast<int>(bodies.size()), kMaxBodies);
        BodiesGpu& g = m_bodiesGpu;
        g.info[0] = n;
        g.info[1] = coupling.twoWay ? 1 : 0;
        g.params[0] = kImpulseScale;
        g.params[1] = std::clamp(coupling.friction, 0.0f, 1.0f);
        for (int i = 0; i < n; ++i)
        {
            const CouplingBody& b = bodies[i];
            Put(g.com[i], b.com);
            Put(g.cObj[i], b.cObj);
            Put(g.axisX[i], b.rotation[0]);
            Put(g.axisY[i], b.rotation[1]);
            Put(g.axisZ[i], b.rotation[2]);
            Put(g.halfExtents[i], b.halfExtents, glm::length(b.halfExtents) + glm::length(b.cObj));
            Put(g.velocity[i], b.velocity);
            Put(g.omega[i], b.omega);
        }
        if (n != m_bodyCount)
        {
            m_bodyCount = n;
            m_bodyForces.assign(static_cast<size_t>(n), BodyForce{});
        }
        if (m_loaded)
            m_bodiesBuffer.Upload(&g, sizeof(BodiesGpu));
    }

    void PbfSim::ClearBodyImpulses()
    {
        if (!m_loaded)
            return;
        m_bodyImpulses.Clear();
        glMemoryBarrier(kComputeBarrier);
    }

    void PbfSim::DownloadBodyImpulses(std::vector<BodyForce>& out) const
    {
        int values[kMaxBodies * kImpulseComponents] = {};
        m_bodyImpulses.Download(values, sizeof(values));
        out.assign(static_cast<size_t>(m_bodyCount), BodyForce{});
        for (int b = 0; b < m_bodyCount; ++b)
        {
            const int* v = values + b * kImpulseComponents;
            out[b].force = glm::vec3(v[0], v[1], v[2]) / kImpulseScale;
            out[b].torque = glm::vec3(v[3], v[4], v[5]) / kImpulseScale;
        }
    }

    void PbfSim::ZeroBodyForces()
    {
        for (BodyForce& f : m_bodyForces)
            f = BodyForce{};
    }

    void PbfSim::ComputeColumns()
    {
        if (!m_loaded)
            return;
        if (m_count == 0)
        {
            m_columns.Clear();
            glMemoryBarrier(kComputeBarrier);
            return;
        }
        BindCommon();
        m_particles[m_current][kVelocity].BindBase(GL_SHADER_STORAGE_BUFFER, kVelocitiesBinding);
        m_columns.BindBase(GL_SHADER_STORAGE_BUFFER, kSortedPositionsBinding);
        m_columnsShader.Bind();
        unsigned int columns = static_cast<unsigned int>(m_columnsX * m_columnsZ);
        glDispatchCompute((columns + 63) / 64, 1, 1);
        glMemoryBarrier(kComputeBarrier);
    }

    void PbfSim::DownloadColumns(std::vector<float>& out) const
    {
        out.resize(static_cast<size_t>(m_columnsX) * m_columnsZ * 4);
        m_columns.Download(out.data(), out.size() * sizeof(float));
    }

    void PbfSim::SubmitReadback(float seconds, bool impulses, bool columns)
    {
        ImpulseSlot& slot = m_impulseSlots[m_nextSlot];
        if (slot.fence != nullptr)
            glDeleteSync(static_cast<GLsync>(slot.fence));
        glMemoryBarrier(kComputeBarrier);
        if (impulses)
        {
            glBindBuffer(GL_COPY_READ_BUFFER, m_bodyImpulses.ID());
            glBindBuffer(GL_COPY_WRITE_BUFFER, slot.buffer.ID());
            glCopyBufferSubData(GL_COPY_READ_BUFFER, GL_COPY_WRITE_BUFFER, 0, 0, static_cast<GLsizeiptr>(kImpulseBytes));
        }
        if (columns)
        {
            glBindBuffer(GL_COPY_READ_BUFFER, m_columns.ID());
            glBindBuffer(GL_COPY_WRITE_BUFFER, slot.columns.ID());
            glCopyBufferSubData(GL_COPY_READ_BUFFER,
                                GL_COPY_WRITE_BUFFER,
                                0,
                                0,
                                static_cast<GLsizeiptr>(m_columnsX) * m_columnsZ * 4 * sizeof(float));
        }
        glBindBuffer(GL_COPY_READ_BUFFER, 0);
        glBindBuffer(GL_COPY_WRITE_BUFFER, 0);
        slot.hasImpulses = impulses;
        slot.hasColumns = columns;
        slot.fence = glFenceSync(GL_SYNC_GPU_COMMANDS_COMPLETE, 0);
        slot.seconds = seconds;
        slot.bodies = m_bodyCount;
        slot.serial = ++m_impulseSerial;
        m_nextSlot = (m_nextSlot + 1) % 3;
    }

    bool PbfSim::PollBodyForces()
    {
        if (!m_loaded)
            return false;
        int order[3] = {0, 1, 2};
        std::sort(order,
                  order + 3,
                  [this](int a, int b) { return m_impulseSlots[a].serial > m_impulseSlots[b].serial; });
        bool newest = true;
        for (int k : order)
        {
            ImpulseSlot& slot = m_impulseSlots[k];
            if (slot.fence == nullptr)
                continue;
            GLuint64 timeout = newest && coupling.waitForForces ? kForceWaitNs : 0;
            newest = false;
            GLenum result = glClientWaitSync(static_cast<GLsync>(slot.fence), GL_SYNC_FLUSH_COMMANDS_BIT, timeout);
            if (result != GL_ALREADY_SIGNALED && result != GL_CONDITION_SATISFIED)
                continue;

            if (slot.hasImpulses)
            {
                int values[kMaxBodies * kImpulseComponents] = {};
                slot.buffer.Download(values, sizeof(values));
                ZeroBodyForces();
                int n = std::min(slot.bodies, m_bodyCount);
                float scale = slot.seconds > 0.0f ? 1.0f / (kImpulseScale * slot.seconds) : 0.0f;
                for (int b = 0; b < n; ++b)
                {
                    const int* v = values + b * kImpulseComponents;
                    m_bodyForces[b].force = glm::vec3(v[0], v[1], v[2]) * scale;
                    m_bodyForces[b].torque = glm::vec3(v[3], v[4], v[5]) * scale;
                }
            }
            if (slot.hasColumns)
            {
                m_columnData.resize(static_cast<size_t>(m_columnsX) * m_columnsZ * 4);
                slot.columns.Download(m_columnData.data(), m_columnData.size() * sizeof(float));
                m_columnsValid = true;
            }
            long long serial = slot.serial;
            for (ImpulseSlot& old : m_impulseSlots)
            {
                if (old.fence != nullptr && old.serial <= serial)
                {
                    glDeleteSync(static_cast<GLsync>(old.fence));
                    old.fence = nullptr;
                }
            }
            return true;
        }
        return false;
    }

    void PbfSim::DiscardBodyFeedback()
    {
        DropImpulseReadbacks();
        ZeroBodyForces();
    }

    void PbfSim::DropImpulseReadbacks()
    {
        for (ImpulseSlot& slot : m_impulseSlots)
        {
            if (slot.fence != nullptr)
                glDeleteSync(static_cast<GLsync>(slot.fence));
            slot.fence = nullptr;
            slot.serial = 0;
        }
    }

    void PbfSim::ComputeNeighborStats()
    {
        if (!m_loaded || m_count == 0)
            return;
        m_stats.Clear();
        BindCommon();
        BindParticles(m_current, kPositionsBinding);
        glMemoryBarrier(kComputeBarrier);
        DispatchParticles(m_neighborStatsShader);

        unsigned int values[4] = {};
        m_stats.Download(values, sizeof(values));
        m_neighborStats.average = static_cast<float>(values[0]) / static_cast<float>(m_count);
        m_neighborStats.maximum = values[1];
        m_neighborStats.densityAverage = static_cast<float>(values[2]) / (1000.0f * static_cast<float>(m_count));
        m_neighborStats.densityMax = static_cast<float>(values[3]) / 10000.0f;
    }

    void PbfSim::DownloadPredicted(std::vector<float>& out) const
    {
        out.resize(static_cast<size_t>(m_count) * 4);
        m_particles[m_current][kPredicted].Download(out.data(), out.size() * sizeof(float));
    }

    void PbfSim::DownloadPositions(std::vector<float>& out) const
    {
        out.resize(static_cast<size_t>(m_count) * 4);
        m_particles[m_current][kPosition].Download(out.data(), out.size() * sizeof(float));
    }

    void PbfSim::DownloadVelocities(std::vector<float>& out) const
    {
        out.resize(static_cast<size_t>(m_count) * 4);
        m_particles[m_current][kVelocity].Download(out.data(), out.size() * sizeof(float));
    }

    void PbfSim::DownloadVorticity(std::vector<float>& out) const
    {
        out.resize(static_cast<size_t>(m_count) * 4);
        m_particles[1 - m_current][kPredicted].Download(out.data(), out.size() * sizeof(float));
    }

    void PbfSim::DownloadNeighborCounts(std::vector<unsigned int>& out) const
    {
        out.resize(static_cast<size_t>(m_count));
        m_neighborCounts.Download(out.data(), out.size() * sizeof(unsigned int));
    }

    void PbfSim::DownloadCellCounts(std::vector<unsigned int>& out) const
    {
        out.resize(static_cast<size_t>(CellCount()));
        m_cellCounts.Download(out.data(), out.size() * sizeof(unsigned int));
    }

    void PbfSim::DownloadCellStarts(std::vector<unsigned int>& out) const
    {
        out.resize(static_cast<size_t>(CellCount()));
        m_cellStarts.Download(out.data(), out.size() * sizeof(unsigned int));
    }

    void PbfSim::BindParticleVao() const
    {
        glBindVertexArray(m_vao);
        glBindVertexBuffer(0, m_particles[m_current][kPosition].ID(), 0, 4 * sizeof(float));
    }

    void PbfSim::UnbindParticleVao() const
    {
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
        if (!m_loaded)
            return;
        DropImpulseReadbacks();
        ZeroBodyForces();
        m_columnsValid = false;
        Seed();
    }

    void PbfSim::Update(float deltaSeconds)
    {
        if (!m_loaded)
            return;

        m_timer.BeginFrame();
        if (paused)
        {
            ZeroBodyForces();
            return;
        }

        UploadParams();
        bool react = coupling.twoWay && m_bodyCount > 0;
        if (!react)
            ZeroBodyForces();
        float elapsed = std::min(deltaSeconds, kFrameSeconds * kMaxFramesPerUpdate);
        m_mouseSeconds = mouse.active ? m_mouseSeconds + elapsed : 0.0f;
        m_accumulator += elapsed;
        int frames = std::min(kMaxFramesPerUpdate, static_cast<int>(std::floor(m_accumulator / kFrameSeconds + 1e-3f)));
        if (frames > 0 && react)
            ClearBodyImpulses();
        for (int f = 0; f < frames; ++f)
        {
            m_bodyFrameOffset = -static_cast<float>(frames - 1 - f) * kFrameSeconds;
            StepFrame();
        }
        m_bodyFrameOffset = 0.0f;
        if (frames == kMaxFramesPerUpdate)
            m_accumulator = 0.0f;
        else
            m_accumulator -= static_cast<float>(frames) * kFrameSeconds;
        if (frames > 0 && columnsEnabled)
            ComputeColumns();
        if (frames > 0 && (react || columnsEnabled))
            SubmitReadback(static_cast<float>(frames) * kFrameSeconds, react, columnsEnabled);

        if (collectNeighborStats && ++m_statsFrame >= kStatsInterval)
        {
            m_statsFrame = 0;
            ComputeNeighborStats();
        }
    }

    void PbfSim::Unload()
    {
        DestroyVao();
        DropImpulseReadbacks();
        for (ImpulseSlot& slot : m_impulseSlots)
            slot.buffer.Destroy();
        m_paramsBuffer.Destroy();
        m_shapesBuffer.Destroy();
        m_bodiesBuffer.Destroy();
        m_bodyImpulses.Destroy();
        m_columns.Destroy();
        for (ImpulseSlot& slot : m_impulseSlots)
            slot.columns.Destroy();
        m_columnData.clear();
        m_columnsValid = false;
        m_bodyCount = 0;
        m_bodyForces.clear();
        for (int set = 0; set < 2; ++set)
            for (int a = 0; a < kAttributeCount; ++a)
                m_particles[set][a].Destroy();
        m_cellCounts.Destroy();
        m_cellStarts.Destroy();
        m_particleCells.Destroy();
        m_particleRanks.Destroy();
        m_stats.Destroy();
        m_neighborCounts.Destroy();
        m_neighborStats = NeighborStats{};
        m_count = 0;
        m_maxParticles = 0;
        m_loaded = false;
    }

    void PbfSim::Shutdown()
    {
        Unload();
        m_scan.Destroy();
        m_timer.Destroy();
    }
}
