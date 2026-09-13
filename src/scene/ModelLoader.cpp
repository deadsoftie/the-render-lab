#include "pch.h"
#include "ModelLoader.h"

#include <assimp/Importer.hpp>
#include <assimp/postprocess.h>
#include <assimp/scene.h>

namespace
{
    void ProcessNode(const aiScene* scene, const aiNode* node, const aiMatrix4x4& parentTransform,
                     Geometry::MeshData& outMesh, unsigned int& vertexBase)
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

            for (unsigned int v = 0; v < mesh->mNumVertices; ++v)
            {
                aiVector3D p = transform * mesh->mVertices[v];
                aiVector3D n = normalMatrix * mesh->mNormals[v];
                n.Normalize();
                outMesh.vertices.insert(outMesh.vertices.end(), {p.x, p.y, p.z, n.x, n.y, n.z});
            }

            for (unsigned int f = 0; f < mesh->mNumFaces; ++f)
            {
                const aiFace& face = mesh->mFaces[f];
                for (unsigned int idx = 0; idx < face.mNumIndices; ++idx)
                    outMesh.indices.push_back(vertexBase + face.mIndices[idx]);
            }

            vertexBase += mesh->mNumVertices;
        }

        for (unsigned int c = 0; c < node->mNumChildren; ++c)
            ProcessNode(scene, node->mChildren[c], transform, outMesh, vertexBase);
    }
}

bool ModelLoader::Load(const std::string& path, Geometry::MeshData& outMesh)
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

    unsigned int vertexBase = 0;
    ProcessNode(scene, scene->mRootNode, aiMatrix4x4(), outMesh, vertexBase);

    if (outMesh.vertices.empty())
    {
        std::cerr << "[ModelLoader] No usable geometry in " << path << "\n";
        return false;
    }

    return true;
}
