#include "pch.h"
#include "ModelLoader.h"

#include <filesystem>

#include <assimp/Importer.hpp>
#include <assimp/postprocess.h>
#include <assimp/scene.h>
#include <assimp/material.h>

namespace
{
    glm::vec3 GetDiffuseColor(const aiMaterial* mat)
    {
        aiColor4D c;
        if (mat && aiGetMaterialColor(mat, AI_MATKEY_COLOR_DIFFUSE, &c) == AI_SUCCESS)
            return {c.r, c.g, c.b};
        return {1.0f, 1.0f, 1.0f};
    }

    // Embedded FBX textures (path "*N") are not supported; export them as external
    // files alongside the model instead.
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
            std::cerr << "[ModelLoader] Embedded texture '" << p
                      << "' not supported, export it as an external file instead\n";
            return "";
        }

        return (modelDir / p).lexically_normal().string();
    }

    void ProcessNode(const aiScene* scene, const aiNode* node, const aiMatrix4x4& parentTransform,
                     const std::filesystem::path& modelDir, Geometry::MeshData& outMesh,
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

            unsigned int indexStart = static_cast<unsigned int>(outMesh.indices.size());
            bool hasUV = mesh->HasTextureCoords(0);

            for (unsigned int v = 0; v < mesh->mNumVertices; ++v)
            {
                aiVector3D p = transform * mesh->mVertices[v];
                aiVector3D n = normalMatrix * mesh->mNormals[v];
                n.Normalize();
                aiVector3D uv = hasUV ? mesh->mTextureCoords[0][v] : aiVector3D(0.0f, 0.0f, 0.0f);
                outMesh.vertices.insert(outMesh.vertices.end(),
                                        {p.x, p.y, p.z, n.x, n.y, n.z, uv.x, uv.y});
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
            ProcessNode(scene, node->mChildren[c], transform, modelDir, outMesh, outSubmeshes,
                       vertexBase);
    }
}

bool ModelLoader::Load(const std::string& path, Geometry::MeshData& outMesh,
                       std::vector<Geometry::SubmeshRange>& outSubmeshes)
{
    Assimp::Importer importer;
    const aiScene* scene = importer.ReadFile(
        path, aiProcess_Triangulate | aiProcess_GenSmoothNormals | aiProcess_JoinIdenticalVertices);

    if (!scene || !scene->HasMeshes())
    {
        std::cerr << "[ModelLoader] Failed to load " << path << ": " << importer.GetErrorString()
                  << "\n";
        return false;
    }

    outMesh.vertices.clear();
    outMesh.indices.clear();
    outSubmeshes.clear();

    std::filesystem::path modelDir = std::filesystem::path(path).parent_path();

    unsigned int vertexBase = 0;
    ProcessNode(scene, scene->mRootNode, aiMatrix4x4(), modelDir, outMesh, outSubmeshes,
               vertexBase);

    if (outMesh.vertices.empty())
    {
        std::cerr << "[ModelLoader] No usable geometry in " << path << "\n";
        return false;
    }

    return true;
}
