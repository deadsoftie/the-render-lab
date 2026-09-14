#include "anim/Skeleton.h"

namespace Anim
{
    int FindBone(const Skeleton& skeleton, const std::string& name)
    {
        auto it = skeleton.boneNameToIndex.find(name);
        return it != skeleton.boneNameToIndex.end() ? it->second : -1;
    }
}
