#pragma once
#include <string>

#include "graphics/resources/Geometry.h"

namespace ModelLoader
{
    // Merges all submeshes into one MeshData; per-submesh materials are ignored.
    bool Load(const std::string& path, Geometry::MeshData& outMesh);
}
