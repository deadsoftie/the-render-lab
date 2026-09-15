#include "pch.h"

#include "graphics/resources/Geometry.h"

#include <numbers>

namespace Geometry
{
    static void PushVertex(std::vector<float>& v, const glm::vec3& p, const glm::vec3& n,
                           glm::vec2 uv = {0, 0})
    {
        v.push_back(p.x);
        v.push_back(p.y);
        v.push_back(p.z);
        v.push_back(n.x);
        v.push_back(n.y);
        v.push_back(n.z);
        v.push_back(uv.x);
        v.push_back(uv.y);
    }

    void AppendTangents(std::vector<float>& vertices, const std::vector<unsigned int>& indices,
                       size_t strideFloats)
    {
        size_t vertexCount = vertices.size() / strideFloats;
        std::vector<glm::vec3> accum(vertexCount, glm::vec3(0.0f));

        auto getVec3 = [&](unsigned int i, size_t offset)
        {
            size_t o = static_cast<size_t>(i) * strideFloats + offset;
            return glm::vec3(vertices[o], vertices[o + 1], vertices[o + 2]);
        };
        auto getVec2 = [&](unsigned int i, size_t offset)
        {
            size_t o = static_cast<size_t>(i) * strideFloats + offset;
            return glm::vec2(vertices[o], vertices[o + 1]);
        };

        for (size_t t = 0; t + 2 < indices.size(); t += 3)
        {
            unsigned int i0 = indices[t], i1 = indices[t + 1], i2 = indices[t + 2];

            glm::vec3 p0 = getVec3(i0, 0), p1 = getVec3(i1, 0), p2 = getVec3(i2, 0);
            glm::vec2 uv0 = getVec2(i0, 6), uv1 = getVec2(i1, 6), uv2 = getVec2(i2, 6);

            glm::vec3 e1 = p1 - p0, e2 = p2 - p0;
            glm::vec2 d1 = uv1 - uv0, d2 = uv2 - uv0;

            float denom = d1.x * d2.y - d2.x * d1.y;
            glm::vec3 tangent = (std::abs(denom) > 1e-8f)
                                    ? (e1 * d2.y - e2 * d1.y) * (1.0f / denom)
                                    : glm::vec3(0.0f);

            accum[i0] += tangent;
            accum[i1] += tangent;
            accum[i2] += tangent;
        }

        std::vector<float> out;
        out.reserve(vertices.size() + vertexCount * 3);
        for (size_t v = 0; v < vertexCount; ++v)
        {
            size_t o = v * strideFloats;
            out.insert(out.end(), vertices.begin() + o, vertices.begin() + o + strideFloats);

            glm::vec3 n = getVec3(static_cast<unsigned int>(v), 3);
            glm::vec3 tOrtho = accum[v] - n * glm::dot(n, accum[v]);
            if (glm::length(tOrtho) < 1e-6f)
            {
                glm::vec3 fallback = (std::abs(n.x) < 0.9f) ? glm::vec3(1, 0, 0) : glm::vec3(0, 1, 0);
                tOrtho = fallback - n * glm::dot(n, fallback);
            }
            glm::vec3 tangent = glm::normalize(tOrtho);

            out.push_back(tangent.x);
            out.push_back(tangent.y);
            out.push_back(tangent.z);
        }

        vertices = std::move(out);
    }

    static void AddQuad(Geometry::MeshData& out,
                        const glm::vec3& a,
                        const glm::vec3& b,
                        const glm::vec3& c,
                        const glm::vec3& d,
                        const glm::vec3& n)
    {
        unsigned int base = static_cast<unsigned int>(out.vertices.size() / 8);

        PushVertex(out.vertices, a, n, {0, 0});
        PushVertex(out.vertices, b, n, {1, 0});
        PushVertex(out.vertices, c, n, {1, 1});
        PushVertex(out.vertices, d, n, {0, 1});

        out.indices.push_back(base + 0);
        out.indices.push_back(base + 1);
        out.indices.push_back(base + 2);

        out.indices.push_back(base + 0);
        out.indices.push_back(base + 2);
        out.indices.push_back(base + 3);
    }

    MeshData MakeCube(float h)
    {
        MeshData out;
        out.vertices.reserve(24 * 8);
        out.indices.reserve(36);

        // +Z (front)
        PushVertex(out.vertices, {-h, -h, +h}, {0, 0, 1}, {0, 0});
        PushVertex(out.vertices, {+h, -h, +h}, {0, 0, 1}, {1, 0});
        PushVertex(out.vertices, {+h, +h, +h}, {0, 0, 1}, {1, 1});
        PushVertex(out.vertices, {-h, +h, +h}, {0, 0, 1}, {0, 1});

        // -Z (back)
        PushVertex(out.vertices, {+h, -h, -h}, {0, 0, -1}, {0, 0});
        PushVertex(out.vertices, {-h, -h, -h}, {0, 0, -1}, {1, 0});
        PushVertex(out.vertices, {-h, +h, -h}, {0, 0, -1}, {1, 1});
        PushVertex(out.vertices, {+h, +h, -h}, {0, 0, -1}, {0, 1});

        // +X (right)
        PushVertex(out.vertices, {+h, -h, +h}, {1, 0, 0}, {0, 0});
        PushVertex(out.vertices, {+h, -h, -h}, {1, 0, 0}, {1, 0});
        PushVertex(out.vertices, {+h, +h, -h}, {1, 0, 0}, {1, 1});
        PushVertex(out.vertices, {+h, +h, +h}, {1, 0, 0}, {0, 1});

        // -X (left)
        PushVertex(out.vertices, {-h, -h, -h}, {-1, 0, 0}, {0, 0});
        PushVertex(out.vertices, {-h, -h, +h}, {-1, 0, 0}, {1, 0});
        PushVertex(out.vertices, {-h, +h, +h}, {-1, 0, 0}, {1, 1});
        PushVertex(out.vertices, {-h, +h, -h}, {-1, 0, 0}, {0, 1});

        // +Y (top)
        PushVertex(out.vertices, {-h, +h, +h}, {0, 1, 0}, {0, 0});
        PushVertex(out.vertices, {+h, +h, +h}, {0, 1, 0}, {1, 0});
        PushVertex(out.vertices, {+h, +h, -h}, {0, 1, 0}, {1, 1});
        PushVertex(out.vertices, {-h, +h, -h}, {0, 1, 0}, {0, 1});

        // -Y (bottom)
        PushVertex(out.vertices, {-h, -h, -h}, {0, -1, 0}, {0, 0});
        PushVertex(out.vertices, {+h, -h, -h}, {0, -1, 0}, {1, 0});
        PushVertex(out.vertices, {+h, -h, +h}, {0, -1, 0}, {1, 1});
        PushVertex(out.vertices, {-h, -h, +h}, {0, -1, 0}, {0, 1});

        // Indices (same as before)
        for (unsigned int face = 0; face < 6; ++face)
        {
            unsigned int base = face * 4;
            out.indices.push_back(base + 0);
            out.indices.push_back(base + 1);
            out.indices.push_back(base + 2);
            out.indices.push_back(base + 0);
            out.indices.push_back(base + 2);
            out.indices.push_back(base + 3);
        }

        AppendTangents(out.vertices, out.indices, 8);
        return out;
    }

    CornellMesh MakeCornellBox(glm::vec3 h)
    {
        CornellMesh out{};
        out.mesh.vertices.reserve(5 * 4 * 8);
        out.mesh.indices.reserve(5 * 6);

        float x0 = -h.x, x1 = +h.x;
        float y0 = -h.y, y1 = +h.y;
        float z0 = -h.z, z1 = +h.z;  // front opening is at z = z1 (no wall)

        auto setPart = [&](int i, unsigned int start, unsigned int count, glm::vec3 albedo)
        {
            out.parts[i].indexStart = start;
            out.parts[i].indexCount = count;
            out.parts[i].albedo = albedo;
        };

        // Each quad adds 6 indices.
        // indexStart is measured in "indices", not triangles.
        unsigned int idxStart = 0;

        // 0) Floor (y=y0), inward normal +Y
        {
            AddQuad(out.mesh, {x0, y0, z1}, {x1, y0, z1}, {x1, y0, z0}, {x0, y0, z0}, {0, +1, 0});
            setPart(0, idxStart, 6, {0.73f, 0.73f, 0.73f});
            idxStart += 6;
        }

        // 1) Ceiling (y=y1), inward normal -Y
        {
            AddQuad(out.mesh, {x0, y1, z0}, {x1, y1, z0}, {x1, y1, z1}, {x0, y1, z1}, {0, -1, 0});
            setPart(1, idxStart, 6, {0.73f, 0.73f, 0.73f});
            idxStart += 6;
        }

        // 2) Back wall (z=z0), inward normal +Z
        {
            AddQuad(out.mesh, {x0, y0, z0}, {x1, y0, z0}, {x1, y1, z0}, {x0, y1, z0}, {0, 0, +1});
            setPart(2, idxStart, 6, {0.73f, 0.73f, 0.73f});
            idxStart += 6;
        }

        // 3) Left wall (x=x0), inward normal +X (red)
        {
            AddQuad(out.mesh, {x0, y0, z0}, {x0, y0, z1}, {x0, y1, z1}, {x0, y1, z0}, {+1, 0, 0});
            setPart(3, idxStart, 6, {0.80f, 0.15f, 0.15f});
            idxStart += 6;
        }

        // 4) Right wall (x=x1), inward normal -X (green)
        {
            AddQuad(out.mesh, {x1, y0, z1}, {x1, y0, z0}, {x1, y1, z0}, {x1, y1, z1}, {-1, 0, 0});
            setPart(4, idxStart, 6, {0.15f, 0.80f, 0.15f});
            idxStart += 6;
        }

        AppendTangents(out.mesh.vertices, out.mesh.indices, 8);
        return out;
    }

    MeshData MakeGroundPlane(float halfSize, float y)
    {
        MeshData out;
        out.vertices.reserve(4 * 8);
        out.indices.reserve(6);

        glm::vec3 n(0, 1, 0);

        glm::vec3 a(-halfSize, y, +halfSize);
        glm::vec3 b(+halfSize, y, +halfSize);
        glm::vec3 c(+halfSize, y, -halfSize);
        glm::vec3 d(-halfSize, y, -halfSize);

        AddQuad(out, a, b, c, d, n);
        AppendTangents(out.vertices, out.indices, 8);
        return out;
    }

    MeshData MakeSphere(float radius, int slices, int stacks)
    {
        MeshData out;

        slices = std::max(3, slices);
        stacks = std::max(2, stacks);

        // Vertex count ~ (stacks+1)*(slices+1)
        out.vertices.reserve((stacks + 1) * (slices + 1) * 8);
        out.indices.reserve(stacks * slices * 6);

        constexpr float pi = std::numbers::pi_v<float>;

        for (int y = 0; y <= stacks; ++y)
        {
            float v = static_cast<float>(y) / static_cast<float>(stacks);  // 0..1
            float phi = v * pi;                                            // 0..PI

            float sinPhi = std::sin(phi);
            float cosPhi = std::cos(phi);

            for (int x = 0; x <= slices; ++x)
            {
                float u = static_cast<float>(x) / static_cast<float>(slices);  // 0..1
                float theta = u * (2.0f * pi);                                 // 0..2PI

                float sinTheta = std::sin(theta);
                float cosTheta = std::cos(theta);

                glm::vec3 n(cosTheta * sinPhi, cosPhi, sinTheta * sinPhi);

                glm::vec3 p = n * radius;

                PushVertex(out.vertices, p, glm::normalize(n), {u, v});
            }
        }

        // Indices
        const int stride = slices + 1;
        for (int y = 0; y < stacks; ++y)
        {
            for (int x = 0; x < slices; ++x)
            {
                unsigned int i0 = static_cast<unsigned int>(y * stride + x);
                unsigned int i1 = i0 + 1;
                unsigned int i2 = static_cast<unsigned int>((y + 1) * stride + x);
                unsigned int i3 = i2 + 1;

                // Two triangles per quad on the sphere grid
                out.indices.push_back(i0);
                out.indices.push_back(i2);
                out.indices.push_back(i1);

                out.indices.push_back(i1);
                out.indices.push_back(i2);
                out.indices.push_back(i3);
            }
        }

        AppendTangents(out.vertices, out.indices, 8);
        return out;
    }
}  // namespace Geometry
