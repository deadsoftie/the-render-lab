#pragma once
#include <string>

#include "Scene.h"

namespace SceneLoader
{
    bool Load(const std::string& path, Scene& outScene);
}
