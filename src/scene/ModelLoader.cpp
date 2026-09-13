#include "pch.h"
#include "ModelLoader.h"

#include <assimp/Importer.hpp>
#include <assimp/postprocess.h>
#include <assimp/scene.h>

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
    for (unsigned int m = 0; m < scene->mNumMeshes; ++m)
    {
        const aiMesh* mesh = scene->mMeshes[m];
        if (!mesh->HasNormals())
            continue;

        for (unsigned int v = 0; v < mesh->mNumVertices; ++v)
        {
            const aiVector3D& p = mesh->mVertices[v];
            const aiVector3D& n = mesh->mNormals[v];
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

    if (outMesh.vertices.empty())
    {
        std::cerr << "[ModelLoader] No usable geometry in " << path << "\n";
        return false;
    }

    return true;
}
