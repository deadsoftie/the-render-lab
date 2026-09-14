#include "pch.h"
#include "scene/SkinnedModelLoader.h"

#include <filesystem>

#include <assimp/Importer.hpp>
#include <assimp/postprocess.h>
#include <assimp/scene.h>
#include <assimp/material.h>

namespace
{
    constexpr int kMaxInfluences = 4;

    struct VertexInfluences
    {
        int boneIDs[kMaxInfluences] = {0, 0, 0, 0};
        float weights[kMaxInfluences] = {0, 0, 0, 0};
        int count = 0;
    };

    // Keeps the 4 strongest influences seen so far; equivalent to sorting all
    // influences by weight and truncating, without needing to buffer them all.
    void AddInfluence(VertexInfluences& v, int boneID, float weight)
    {
        if (v.count < kMaxInfluences)
        {
            v.boneIDs[v.count] = boneID;
            v.weights[v.count] = weight;
            ++v.count;
            return;
        }

        int weakest = 0;
        for (int i = 1; i < kMaxInfluences; ++i)
            if (v.weights[i] < v.weights[weakest])
                weakest = i;
        if (weight > v.weights[weakest])
        {
            v.boneIDs[weakest] = boneID;
            v.weights[weakest] = weight;
        }
    }

    void NormalizeInfluences(VertexInfluences& v)
    {
        float sum = v.weights[0] + v.weights[1] + v.weights[2] + v.weights[3];
        if (sum > 1e-6f)
        {
            for (float& w : v.weights)
                w /= sum;
        }
        else
        {
            v.boneIDs[0] = 0;
            v.weights[0] = 1.0f;
        }
    }

    glm::vec3 GetDiffuseColor(const aiMaterial* mat)
    {
        aiColor4D c;
        if (mat && aiGetMaterialColor(mat, AI_MATKEY_COLOR_DIFFUSE, &c) == AI_SUCCESS)
            return {c.r, c.g, c.b};
        return {1.0f, 1.0f, 1.0f};
    }

    std::string ResolveDiffuseTexture(const aiMaterial* mat, const std::filesystem::path& modelDir)
    {
        if (!mat || mat->GetTextureCount(aiTextureType_DIFFUSE) == 0)
            return "";

        aiString texPath;
        if (mat->GetTexture(aiTextureType_DIFFUSE, 0, &texPath) != AI_SUCCESS)
            return "";

        std::string p = texPath.C_Str();
        if (p.empty())
            return "";

        if (p[0] == '*')
        {
            std::cerr << "[SkinnedModelLoader] Embedded texture '" << p
                      << "' not supported, export it as an external file instead\n";
            return "";
        }

        return (modelDir / p).lexically_normal().string();
    }

    void ProcessNode(const aiScene* scene, const aiNode* node, const aiMatrix4x4& parentTransform,
                     const Anim::Skeleton& skeleton, const std::filesystem::path& modelDir,
                     Geometry::SkinnedMeshData& outMesh,
                     std::vector<Geometry::SubmeshRange>& outSubmeshes, unsigned int& vertexBase)
    {
        aiMatrix4x4 transform = parentTransform * node->mTransformation;
        aiMatrix3x3 normalMatrix(transform);
        normalMatrix.Inverse();
        normalMatrix.Transpose();

        for (unsigned int m = 0; m < node->mNumMeshes; ++m)
        {
            const aiMesh* mesh = scene->mMeshes[node->mMeshes[m]];
            if (!mesh->HasNormals())
                continue;

            std::vector<VertexInfluences> influences(mesh->mNumVertices);
            for (unsigned int b = 0; b < mesh->mNumBones; ++b)
            {
                const aiBone* bone = mesh->mBones[b];
                int boneIndex = Anim::FindBone(skeleton, bone->mName.C_Str());
                if (boneIndex < 0)
                    continue;

                for (unsigned int w = 0; w < bone->mNumWeights; ++w)
                {
                    const aiVertexWeight& vw = bone->mWeights[w];
                    AddInfluence(influences[vw.mVertexId], boneIndex, vw.mWeight);
                }
            }
            for (auto& v : influences)
                NormalizeInfluences(v);

            unsigned int indexStart = static_cast<unsigned int>(outMesh.indices.size());
            bool hasUV = mesh->HasTextureCoords(0);

            for (unsigned int v = 0; v < mesh->mNumVertices; ++v)
            {
                aiVector3D p = transform * mesh->mVertices[v];
                aiVector3D n = normalMatrix * mesh->mNormals[v];
                n.Normalize();
                aiVector3D uv = hasUV ? mesh->mTextureCoords[0][v] : aiVector3D(0.0f, 0.0f, 0.0f);
                const VertexInfluences& inf = influences[v];
                outMesh.vertices.insert(
                    outMesh.vertices.end(),
                    {p.x, p.y, p.z, n.x, n.y, n.z, uv.x, uv.y,
                     static_cast<float>(inf.boneIDs[0]), static_cast<float>(inf.boneIDs[1]),
                     static_cast<float>(inf.boneIDs[2]), static_cast<float>(inf.boneIDs[3]),
                     inf.weights[0], inf.weights[1], inf.weights[2], inf.weights[3]});
            }

            for (unsigned int f = 0; f < mesh->mNumFaces; ++f)
            {
                const aiFace& face = mesh->mFaces[f];
                for (unsigned int idx = 0; idx < face.mNumIndices; ++idx)
                    outMesh.indices.push_back(vertexBase + face.mIndices[idx]);
            }

            vertexBase += mesh->mNumVertices;

            Geometry::SubmeshRange range;
            range.indexStart = indexStart;
            range.indexCount = static_cast<unsigned int>(outMesh.indices.size()) - indexStart;
            const aiMaterial* mat = scene->mMaterials[mesh->mMaterialIndex];
            range.albedo = GetDiffuseColor(mat);
            range.albedoTexture = ResolveDiffuseTexture(mat, modelDir);
            outSubmeshes.push_back(range);
        }

        for (unsigned int c = 0; c < node->mNumChildren; ++c)
            ProcessNode(scene, node->mChildren[c], transform, skeleton, modelDir, outMesh,
                       outSubmeshes, vertexBase);
    }
}

bool SkinnedModelLoader::Load(const std::string& path, const Anim::Skeleton& skeleton,
                              Geometry::SkinnedMeshData& outMesh,
                              std::vector<Geometry::SubmeshRange>& outSubmeshes)
{
    Assimp::Importer importer;
    const aiScene* scene = importer.ReadFile(
        path, aiProcess_Triangulate | aiProcess_GenSmoothNormals | aiProcess_JoinIdenticalVertices);

    if (!scene || !scene->HasMeshes())
    {
        std::cerr << "[SkinnedModelLoader] Failed to load " << path << ": "
                  << importer.GetErrorString() << "\n";
        return false;
    }

    outMesh.vertices.clear();
    outMesh.indices.clear();
    outSubmeshes.clear();

    std::filesystem::path modelDir = std::filesystem::path(path).parent_path();

    unsigned int vertexBase = 0;
    ProcessNode(scene, scene->mRootNode, aiMatrix4x4(), skeleton, modelDir, outMesh, outSubmeshes,
               vertexBase);

    if (outMesh.vertices.empty())
    {
        std::cerr << "[SkinnedModelLoader] No usable geometry in " << path << "\n";
        return false;
    }

    return true;
}
