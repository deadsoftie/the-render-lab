#include "pch.h"
#include "scene/SkinnedModelLoader.h"
#include "scene/UfbxSceneUtil.h"

#include <filesystem>

using UfbxSceneUtil::GetDiffuseColor;
using UfbxSceneUtil::ResolveDiffuseTexture;
using UfbxSceneUtil::ScenePtr;
using UfbxSceneUtil::ToGlmVec3;
using UfbxSceneUtil::ToStdString;

namespace
{
    constexpr int kMaxInfluences = 4;

    // ufbx_skin_vertex weights are pre-sorted by decreasing weight, so the first kMaxInfluences is the strongest set; returns false if it fell back to bone 0 (no real weight found).
    bool GetVertexInfluences(const ufbx_mesh* mesh, const Anim::Skeleton& skeleton,
                            uint32_t vertexIndex, int outBoneIDs[kMaxInfluences],
                            float outWeights[kMaxInfluences], size_t& unresolvedNameCount)
    {
        for (int i = 0; i < kMaxInfluences; ++i)
        {
            outBoneIDs[i] = 0;
            outWeights[i] = 0.0f;
        }

        if (mesh->skin_deformers.count == 0)
        {
            outWeights[0] = 1.0f;
            return false;
        }
        const ufbx_skin_deformer* skin = mesh->skin_deformers.data[0];
        if (vertexIndex >= skin->vertices.count)
        {
            outWeights[0] = 1.0f;
            return false;
        }

        ufbx_skin_vertex sv = skin->vertices.data[vertexIndex];
        int count = 0;
        float sum = 0.0f;
        for (uint32_t w = 0; w < sv.num_weights && count < kMaxInfluences; ++w)
        {
            const ufbx_skin_weight& weight = skin->weights.data[sv.weight_begin + w];
            const ufbx_skin_cluster* cluster = skin->clusters.data[weight.cluster_index];
            if (!cluster->bone_node)
                continue;
            int boneIndex = Anim::FindBone(skeleton, ToStdString(cluster->bone_node->name));
            if (boneIndex < 0)
            {
                ++unresolvedNameCount;
                continue;
            }

            outBoneIDs[count] = boneIndex;
            outWeights[count] = static_cast<float>(weight.weight);
            sum += outWeights[count];
            ++count;
        }

        if (sum > 1e-6f)
        {
            for (int i = 0; i < kMaxInfluences; ++i)
                outWeights[i] /= sum;
            return true;
        }

        outBoneIDs[0] = 0;
        outWeights[0] = 1.0f;
        return false;
    }

    void ProcessMesh(const ufbx_mesh* mesh, const Anim::Skeleton& skeleton,
                     const std::filesystem::path& modelDir, Geometry::SkinnedMeshData& outMesh,
                     std::vector<Geometry::SubmeshRange>& outSubmeshes, unsigned int& vertexBase,
                     size_t& fallbackCount, size_t& totalCount, size_t& unresolvedNameCount)
    {
        if (!mesh->vertex_position.exists || !mesh->vertex_normal.exists)
            return;

        bool hasUV = mesh->vertex_uv.exists;

        // Deliberately NOT baking node->geometry_to_world (unlike ModelLoader): the skin matrix already expects raw untransformed mesh-local input, so pre-baking here would apply the node transform twice.
        for (size_t idx = 0; idx < mesh->num_indices; ++idx)
        {
            ufbx_vec3 p = ufbx_get_vertex_vec3(&mesh->vertex_position, idx);
            ufbx_vec3 n = ufbx_get_vertex_vec3(&mesh->vertex_normal, idx);
            glm::vec3 nrm = glm::normalize(ToGlmVec3(n));
            ufbx_vec2 uv = {};
            if (hasUV)
                uv = ufbx_get_vertex_vec2(&mesh->vertex_uv, idx);

            uint32_t vertexIndex = mesh->vertex_indices.count > idx
                                       ? mesh->vertex_indices.data[idx]
                                       : static_cast<uint32_t>(idx);
            int boneIDs[kMaxInfluences];
            float weights[kMaxInfluences];
            ++totalCount;
            if (!GetVertexInfluences(mesh, skeleton, vertexIndex, boneIDs, weights,
                                     unresolvedNameCount))
                ++fallbackCount;

            outMesh.vertices.insert(
                outMesh.vertices.end(),
                {static_cast<float>(p.x), static_cast<float>(p.y), static_cast<float>(p.z), nrm.x,
                 nrm.y, nrm.z, static_cast<float>(uv.x), static_cast<float>(uv.y),
                 static_cast<float>(boneIDs[0]), static_cast<float>(boneIDs[1]),
                 static_cast<float>(boneIDs[2]), static_cast<float>(boneIDs[3]), weights[0],
                 weights[1], weights[2], weights[3]});
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
                uint32_t numTriIndices =
                    ufbx_triangulate_face(triBuf.data(), triBuf.size(), mesh, face);
                for (uint32_t i = 0; i < numTriIndices; ++i)
                    outMesh.indices.push_back(vertexBase + triBuf[i]);
            }

            unsigned int indexCount =
                static_cast<unsigned int>(outMesh.indices.size()) - indexStart;
            if (indexCount == 0)
                continue;

            const ufbx_material* mat =
                mesh->materials.count > 0 ? mesh->materials.data[matIdx] : nullptr;
            Geometry::SubmeshRange range;
            range.indexStart = indexStart;
            range.indexCount = indexCount;
            range.albedo = GetDiffuseColor(mat);
            range.albedoTexture = ResolveDiffuseTexture(mat, modelDir, "[SkinnedModelLoader]");
            outSubmeshes.push_back(range);
        }

        vertexBase += static_cast<unsigned int>(mesh->num_indices);
    }

    void ProcessNode(const ufbx_node* node, const Anim::Skeleton& skeleton,
                     const std::filesystem::path& modelDir, Geometry::SkinnedMeshData& outMesh,
                     std::vector<Geometry::SubmeshRange>& outSubmeshes, unsigned int& vertexBase,
                     size_t& fallbackCount, size_t& totalCount, size_t& unresolvedNameCount)
    {
        if (node->mesh)
            ProcessMesh(node->mesh, skeleton, modelDir, outMesh, outSubmeshes, vertexBase,
                       fallbackCount, totalCount, unresolvedNameCount);

        for (const ufbx_node* child : node->children)
            ProcessNode(child, skeleton, modelDir, outMesh, outSubmeshes, vertexBase,
                       fallbackCount, totalCount, unresolvedNameCount);
    }
}

bool SkinnedModelLoader::Load(const std::string& path, const Anim::Skeleton& skeleton,
                              Geometry::SkinnedMeshData& outMesh,
                              std::vector<Geometry::SubmeshRange>& outSubmeshes)
{
    ufbx_load_opts opts = UfbxSceneUtil::MakeLoadOpts();
    ufbx_error error;
    ScenePtr scene(ufbx_load_file(path.c_str(), &opts, &error));

    if (!scene || scene->meshes.count == 0)
    {
        std::cerr << "[SkinnedModelLoader] Failed to load " << path << ": "
                  << ToStdString(error.description) << "\n";
        return false;
    }

    outMesh.vertices.clear();
    outMesh.indices.clear();
    outSubmeshes.clear();

    std::filesystem::path modelDir = std::filesystem::path(path).parent_path();

    unsigned int vertexBase = 0;
    size_t fallbackCount = 0, totalCount = 0, unresolvedNameCount = 0;
    ProcessNode(scene->root_node, skeleton, modelDir, outMesh, outSubmeshes, vertexBase,
               fallbackCount, totalCount, unresolvedNameCount);

    if (fallbackCount > 0)
        std::cerr << "[SkinnedModelLoader] " << fallbackCount << "/" << totalCount
                  << " vertices in " << path << " had no skin weight, bound to bone 0\n";
    if (unresolvedNameCount > 0)
        std::cerr << "[SkinnedModelLoader] " << unresolvedNameCount
                  << " weight entries in " << path << " referenced a bone not in the skeleton\n";

    if (outMesh.vertices.empty())
    {
        std::cerr << "[SkinnedModelLoader] No usable geometry in " << path << "\n";
        return false;
    }

    return true;
}
