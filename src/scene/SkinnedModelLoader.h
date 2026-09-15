#pragma once
#include <string>
#include <vector>

#include "anim/Skeleton.h"
#include "graphics/resources/Geometry.h"

namespace SkinnedModelLoader
{
    // Same submesh-merging behavior as ModelLoader::Load, plus up to 4 bone
    // influences per vertex resolved against an already-loaded Skeleton (bone
    // names in the FBX must match skeleton.bones[*].name).
    bool Load(const std::string& path, const Anim::Skeleton& skeleton,
              Geometry::SkinnedMeshData& outMesh,
              std::vector<Geometry::SubmeshRange>& outSubmeshes);
}
