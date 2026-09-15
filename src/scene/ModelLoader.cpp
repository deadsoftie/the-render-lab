#include "pch.h"
#include "ModelLoader.h"
#include "scene/UfbxSceneUtil.h"

#include <filesystem>

using UfbxSceneUtil::GetDiffuseColor;
using UfbxSceneUtil::ResolveDiffuseTexture;
using UfbxSceneUtil::ResolveMapTexture;
using UfbxSceneUtil::ScenePtr;
using UfbxSceneUtil::ToGlmVec3;
using UfbxSceneUtil::ToStdString;

namespace
{
    void ProcessMesh(const ufbx_node* node, const ufbx_mesh* mesh,
                     const std::filesystem::path& modelDir, Geometry::MeshData& outMesh,
                     std::vector<Geometry::SubmeshRange>& outSubmeshes, unsigned int& vertexBase)
    {
        if (!mesh->vertex_position.exists || !mesh->vertex_normal.exists)
            return;

        ufbx_matrix normalMatrix = ufbx_get_compatible_matrix_for_normals(node);
        bool hasUV = mesh->vertex_uv.exists;

        for (size_t idx = 0; idx < mesh->num_indices; ++idx)
        {
            ufbx_vec3 p = ufbx_transform_position(
                &node->geometry_to_world, ufbx_get_vertex_vec3(&mesh->vertex_position, idx));
            ufbx_vec3 n = ufbx_transform_direction(
                &normalMatrix, ufbx_get_vertex_vec3(&mesh->vertex_normal, idx));
            glm::vec3 nrm = glm::normalize(ToGlmVec3(n));
            ufbx_vec2 uv = {};
            if (hasUV)
                uv = ufbx_get_vertex_vec2(&mesh->vertex_uv, idx);

            outMesh.vertices.insert(outMesh.vertices.end(),
                                    {static_cast<float>(p.x), static_cast<float>(p.y),
                                     static_cast<float>(p.z), nrm.x, nrm.y, nrm.z,
                                     static_cast<float>(uv.x), static_cast<float>(uv.y)});
        }

        std::vector<uint32_t> triBuf(mesh->max_face_triangles * 3);
        size_t numMaterialPasses = mesh->materials.count > 0 ? mesh->materials.count : 1;

        for (size_t matIdx = 0; matIdx < numMaterialPasses; ++matIdx)
        {
            unsigned int indexStart = static_cast<unsigned int>(outMesh.indices.size());

            for (size_t f = 0; f < mesh->num_faces; ++f)
            {
                bool faceUsesMaterial = mesh->materials.count == 0 ||
                                        (mesh->face_material.count > f &&
                                         mesh->face_material.data[f] == matIdx);
                if (!faceUsesMaterial)
                    continue;

                ufbx_face face = mesh->faces.data[f];
                // ufbx_triangulate_face returns the number of TRIANGLES written, not indices -
                // each triangle is 3 entries in triBuf.
                uint32_t numTriangles =
                    ufbx_triangulate_face(triBuf.data(), triBuf.size(), mesh, face);
                for (uint32_t i = 0; i < numTriangles * 3; ++i)
                    outMesh.indices.push_back(vertexBase + triBuf[i]);
            }

            unsigned int indexCount =
                static_cast<unsigned int>(outMesh.indices.size()) - indexStart;
            if (indexCount == 0)
                continue;

            const ufbx_material* mat = mesh->materials.count > 0 ? mesh->materials.data[matIdx] : nullptr;
            Geometry::SubmeshRange range;
            range.indexStart = indexStart;
            range.indexCount = indexCount;
            range.albedo = GetDiffuseColor(mat);
            range.albedoTexture = ResolveDiffuseTexture(mat, modelDir, "[ModelLoader]");
            if (mat)
            {
                range.specularTexture = ResolveMapTexture(
                    mat->pbr.specular_color.texture ? mat->pbr.specular_color
                                                    : mat->fbx.specular_color,
                    modelDir, "[ModelLoader]");
                range.roughnessTexture =
                    ResolveMapTexture(mat->pbr.roughness, modelDir, "[ModelLoader]");
                range.metallicTexture =
                    ResolveMapTexture(mat->pbr.metalness, modelDir, "[ModelLoader]");
                range.normalTexture = ResolveMapTexture(
                    mat->pbr.normal_map.texture ? mat->pbr.normal_map : mat->fbx.normal_map,
                    modelDir, "[ModelLoader]");
            }
            outSubmeshes.push_back(range);
        }

        vertexBase += static_cast<unsigned int>(mesh->num_indices);
    }

    void ProcessNode(const ufbx_node* node, const std::filesystem::path& modelDir,
                     Geometry::MeshData& outMesh, std::vector<Geometry::SubmeshRange>& outSubmeshes,
                     unsigned int& vertexBase)
    {
        if (node->mesh)
            ProcessMesh(node, node->mesh, modelDir, outMesh, outSubmeshes, vertexBase);

        for (const ufbx_node* child : node->children)
            ProcessNode(child, modelDir, outMesh, outSubmeshes, vertexBase);
    }
}

bool ModelLoader::Load(const std::string& path, Geometry::MeshData& outMesh,
                       std::vector<Geometry::SubmeshRange>& outSubmeshes)
{
    ufbx_load_opts opts = UfbxSceneUtil::MakeLoadOpts();
    ufbx_error error;
    ScenePtr scene(ufbx_load_file(path.c_str(), &opts, &error));

    if (!scene || scene->meshes.count == 0)
    {
        std::cerr << "[ModelLoader] Failed to load " << path << ": "
                  << ToStdString(error.description) << "\n";
        return false;
    }

    outMesh.vertices.clear();
    outMesh.indices.clear();
    outSubmeshes.clear();

    std::filesystem::path modelDir = std::filesystem::path(path).parent_path();

    unsigned int vertexBase = 0;
    ProcessNode(scene->root_node, modelDir, outMesh, outSubmeshes, vertexBase);

    if (outMesh.vertices.empty())
    {
        std::cerr << "[ModelLoader] No usable geometry in " << path << "\n";
        return false;
    }

    Geometry::AppendTangents(outMesh.vertices, outMesh.indices, 8);

    return true;
}
