#pragma once
#include <string>
#include <vector>

#include "anim/Skeleton.h"
#include "graphics/resources/Geometry.h"

namespace SkinnedModelLoader
{
    bool Load(const std::string& path,
              const Anim::Skeleton& skeleton,
              Geometry::SkinnedMeshData& outMesh,
              std::vector<Geometry::SubmeshRange>& outSubmeshes,
              size_t* outUnresolvedInfluences = nullptr);
}
