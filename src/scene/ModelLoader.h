#pragma once
#include <string>
#include <vector>

#include "graphics/resources/Geometry.h"

namespace ModelLoader
{
    // Merges all submeshes into one MeshData; outSubmeshes has one entry per
    // source ufbx_mesh (draw range + material albedo color/texture), so multi-material
    // imports still render correctly via Mesh::DrawRange.
    bool Load(const std::string& path, Geometry::MeshData& outMesh,
              std::vector<Geometry::SubmeshRange>& outSubmeshes);
}
