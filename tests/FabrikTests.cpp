#include "MotionTests.h"
#include "RotationTestHelpers.h"
#include "TestFramework.h"
#include "anim/IK/Fabrik.h"

#include <cmath>
#include <limits>
#include <stdexcept>

namespace
{
    using namespace Anim;

    bool Near(const Vec3& a, const Vec3& b, float epsilon = 1e-4f)
    {
        return std::abs(a.x - b.x) <= epsilon && std::abs(a.y - b.y) <= epsilon &&
               std::abs(a.z - b.z) <= epsilon;
    }

    template <typename Exception, typename Function>
    bool Throws(Function function)
    {
        try
        {
            function();
        }
        catch (const Exception&)
        {
            return true;
        }
        return false;
    }

    bool AllFinite(const std::vector<Vec3>& joints)
    {
        for (const Vec3& j : joints)
            if (!std::isfinite(j.x) || !std::isfinite(j.y) || !std::isfinite(j.z))
                return false;
        return true;
    }

    std::vector<float> BoneLengths(const std::vector<Vec3>& joints)
    {
        std::vector<float> lengths;
        for (std::size_t i = 0; i + 1 < joints.size(); ++i)
            lengths.push_back(Length(joints[i + 1] - joints[i]));
        return lengths;
    }

    void CheckLengths(const std::vector<Vec3>& joints, const std::vector<float>& expected)
    {
        std::vector<float> lengths = BoneLengths(joints);
        CHECK(lengths.size() == expected.size());
        for (std::size_t i = 0; i < lengths.size() && i < expected.size(); ++i)
            CHECK_NEAR(lengths[i], expected[i], 1e-4f);
    }

    std::vector<Vec3> StraightChain()
    {
        return {{0, 0, 0}, {0, 1, 0}, {0, 3, 0}, {0, 4.5f, 0}};
    }

    void CheckSolver()
    {
        const std::vector<float> lengths{1.0f, 2.0f, 1.5f};
        FabrikSettings settings;

        for (const Vec3& target : {Vec3{2, 2, 1},
                                   Vec3{-1, 0.5f, 2},
                                   Vec3{0.5f, -1, 0.2f},
                                   Vec3{0.5f, 4.0f, 0.5f},
                                   Vec3{1.5f, 1, -1}})
        {
            std::vector<Vec3> joints = StraightChain();
            FabrikResult result = SolveFabrik(joints, target, settings);
            CHECK(result.reachable);
            CHECK(result.converged);
            CHECK(result.error <= settings.tolerance);
            CHECK(result.iterations > 0);
            CHECK(result.iterations <= settings.maxIterations);
            CHECK(Near(joints.back(), target, settings.tolerance * 1.01f));
            CHECK(Near(joints.front(), {0, 0, 0}, 0));
            CheckLengths(joints, lengths);
        }

        std::vector<Vec3> bent{{0, 0, 0}, {1, 0, 0}, {1, 2, 0}, {-0.5f, 2, 0}};
        const std::vector<float> bentLengths = BoneLengths(bent);
        FabrikResult folded = SolveFabrik(bent, Vec3{0.3f, 0.4f, 0.2f}, settings);
        CHECK(folded.reachable);
        CHECK(folded.converged);
        CheckLengths(bent, bentLengths);

        std::vector<Vec3> far = StraightChain();
        const Vec3 farTarget{10, 2, -3};
        FabrikResult unreachable = SolveFabrik(far, farTarget, settings);
        CHECK(!unreachable.reachable);
        CHECK(!unreachable.converged);
        CHECK(unreachable.iterations == 0);
        Vec3 direction = Normalize(farTarget);
        CHECK(Near(far.front(), {0, 0, 0}, 0));
        CHECK(Near(far.back(), direction * 4.5f));
        for (const Vec3& joint : far)
            CHECK(Length(Cross(joint, direction)) < 1e-4f);
        CHECK_NEAR(unreachable.error, Length(farTarget) - 4.5f, 1e-4f);
        CheckLengths(far, lengths);

        std::vector<Vec3> atTarget = StraightChain();
        FabrikResult already = SolveFabrik(atTarget, Vec3{0, 4.5f, 0}, settings);
        CHECK(already.reachable);
        CHECK(already.converged);
        CHECK(already.iterations == 0);
        CHECK(Near(atTarget.back(), {0, 4.5f, 0}, 0));

        std::vector<Vec3> rootTarget{{0, 0, 0}, {0, 1, 0}, {0, 2, 0}};
        FabrikResult toRoot = SolveFabrik(rootTarget, Vec3{0, 0, 0}, settings);
        CHECK(toRoot.reachable);
        CHECK(AllFinite(rootTarget));
        CHECK(Near(rootTarget.front(), {0, 0, 0}, 0));
        CheckLengths(rootTarget, {1.0f, 1.0f});

        FabrikSettings limited;
        limited.maxIterations = 1;
        limited.tolerance = 0.0f;
        std::vector<Vec3> capped = StraightChain();
        FabrikResult cappedResult = SolveFabrik(capped, Vec3{2, 2, 1}, limited);
        CHECK(cappedResult.iterations == 1);
        CHECK(cappedResult.error >= 0.0f);

        std::vector<Vec3> one{{0, 0, 0}};
        CHECK(Throws<std::invalid_argument>([&] { SolveFabrik(one, Vec3{1, 0, 0}); }));
        std::vector<Vec3> valid = StraightChain();
        const float nan = std::numeric_limits<float>::quiet_NaN();
        CHECK(Throws<std::invalid_argument>([&] { SolveFabrik(valid, Vec3{nan, 0, 0}); }));
        FabrikSettings negative;
        negative.tolerance = -1.0f;
        CHECK(Throws<std::invalid_argument>([&] { SolveFabrik(valid, Vec3{1, 1, 0}, negative); }));
        std::vector<Vec3> badJoint = StraightChain();
        badJoint[2].y = nan;
        CHECK(Throws<std::invalid_argument>([&] { SolveFabrik(badJoint, Vec3{1, 1, 0}); }));
    }

    Skeleton TestSkeleton(std::vector<VQS>& localPose)
    {
        Skeleton skeleton;
        auto add = [&](const char* name, int parent, const VQS& local)
        {
            Bone bone;
            bone.name = name;
            bone.parentIndex = parent;
            bone.localBindPose = local;
            skeleton.boneNameToIndex[name] = static_cast<int>(skeleton.bones.size());
            skeleton.bones.push_back(bone);
            localPose.push_back(local);
        };
        add("root", -1, {{0.5f, 1, 0}, FromAxisAngle({0, 1, 0}, 0.7f), 1.0f});
        add("shoulder", 0, {{0.3f, 0.5f, 0}, FromAxisAngle({0, 0, 1}, -0.4f), 1.0f});
        add("elbow", 1, {{0, 1.2f, 0}, FromAxisAngle({1, 0, 0}, 0.3f), 1.0f});
        add("hand", 2, {{0, 1.0f, 0}, FromAxisAngle({0, 0, 1}, 0.2f), 1.0f});
        add("finger", 3, {{0.05f, 0.3f, 0}, FromAxisAngle({1, 0, 0}, 0.5f), 1.0f});
        add("other", 0, {{-0.3f, 0.5f, 0}, Quat{}, 1.0f});
        return skeleton;
    }

    std::vector<VQS> WorldFromLocal(const Skeleton& skeleton, const std::vector<VQS>& localPose)
    {
        std::vector<VQS> world(skeleton.bones.size());
        for (std::size_t i = 0; i < skeleton.bones.size(); ++i)
        {
            int parent = skeleton.bones[i].parentIndex;
            world[i] = Concat(parent >= 0 ? world[parent] : VQS{}, localPose[i]);
        }
        return world;
    }

    void CheckChain()
    {
        std::vector<VQS> localPose;
        Skeleton skeleton = TestSkeleton(localPose);

        std::vector<int> chain = BuildChain(skeleton, 1, 3);
        CHECK(chain == std::vector<int>({1, 2, 3}));
        CHECK(BuildChain(skeleton, 0, 4) == std::vector<int>({0, 1, 2, 3, 4}));
        CHECK(BuildChain(skeleton, 2, 2) == std::vector<int>({2}));
        CHECK(BuildChain(skeleton, 1, 5).empty());
        CHECK(BuildChain(skeleton, 3, 1).empty());
        CHECK(BuildChain(skeleton, -1, 3).empty());
        CHECK(BuildChain(skeleton, 1, 99).empty());

        std::vector<VQS> worldPose = WorldFromLocal(skeleton, localPose);
        const std::vector<VQS> originalLocal = localPose;
        const std::vector<VQS> originalWorld = worldPose;

        std::vector<Vec3> positions = GatherChainPositions(worldPose, chain);
        CHECK(positions.size() == chain.size());
        for (std::size_t k = 0; k < chain.size(); ++k)
            CHECK(Near(positions[k], worldPose[chain[k]].v, 0));

        Vec3 target = positions.front() + Vec3{1.2f, 0.4f, 0.9f};
        FabrikResult result = SolveFabrik(positions, target);
        CHECK(result.converged);

        ApplyChainPositions(skeleton, chain, positions, localPose, worldPose);
        for (std::size_t k = 0; k < chain.size(); ++k)
            CHECK(Near(worldPose[chain[k]].v, positions[k], 1e-4f));
        CHECK(Near(worldPose[3].v, target, 2e-3f));

        CHECK(Test::NearRotation(localPose[3].q, originalLocal[3].q, 0));
        CHECK(Test::NearRotation(localPose[4].q, originalLocal[4].q, 0));
        CHECK(Test::NearRotation(localPose[0].q, originalLocal[0].q, 0));
        CHECK(Test::NearRotation(localPose[5].q, originalLocal[5].q, 0));
        for (std::size_t i = 0; i < localPose.size(); ++i)
            CHECK(Near(localPose[i].v, originalLocal[i].v, 0));
        CHECK(Near(worldPose[0].v, originalWorld[0].v, 0));
        CHECK(Near(worldPose[5].v, originalWorld[5].v, 0));
        CHECK(Test::NearRotation(worldPose[5].q, originalWorld[5].q, 0));

        std::vector<VQS> rebuilt = WorldFromLocal(skeleton, localPose);
        for (std::size_t i = 0; i < rebuilt.size(); ++i)
        {
            CHECK(Near(rebuilt[i].v, worldPose[i].v, 1e-5f));
            CHECK(Test::NearRotation(rebuilt[i].q, worldPose[i].q, 1e-5f));
        }

        std::vector<Vec3> unchanged = GatherChainPositions(originalWorld, chain);
        std::vector<VQS> sameLocal = originalLocal;
        std::vector<VQS> sameWorld = originalWorld;
        ApplyChainPositions(skeleton, chain, unchanged, sameLocal, sameWorld);
        for (std::size_t i = 0; i < sameLocal.size(); ++i)
        {
            CHECK(Test::NearRotation(sameLocal[i].q, originalLocal[i].q, 1e-5f));
            CHECK(Near(sameWorld[i].v, originalWorld[i].v, 1e-5f));
        }

        std::vector<Vec3> wrongCount(2);
        CHECK(Throws<std::invalid_argument>(
            [&] { ApplyChainPositions(skeleton, chain, wrongCount, localPose, worldPose); }));
        std::vector<VQS> shortPose(2);
        CHECK(Throws<std::invalid_argument>(
            [&] { ApplyChainPositions(skeleton, chain, positions, shortPose, worldPose); }));
    }
}  // namespace

void RunFabrikTests()
{
    CheckSolver();
    CheckChain();
}
