#include "MotionTests.h"
#include "RotationTestHelpers.h"
#include "TestFramework.h"
#include "anim/IK/Fabrik.h"

#include <algorithm>
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

    // Position-only FABRIK: reachable targets converge with lengths kept and the root fixed,
    // unreachable ones straighten the chain, and bad input throws.
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

    // Chain helpers: BuildChain order, and ApplyChainPositions turning solved positions back
    // into rotations that rebuild to the same world pose.
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

    constexpr float kDegrees = 3.14159265f / 180.0f;

    // A 7-bone test rig shaped like the Mixamo right arm: root, spine, clavicle, upper arm
    // (bent down 70 degrees), forearm (bent 30 degrees), hand and a finger. Bones point along -X.
    std::vector<VQS> ArmLocalPose(Skeleton& skeleton)
    {
        std::vector<VQS> local;
        auto add = [&](int parent, const VQS& pose)
        {
            Bone bone;
            bone.parentIndex = parent;
            bone.localBindPose = pose;
            skeleton.bones.push_back(bone);
            local.push_back(pose);
        };
        add(-1, {{0, 1, 0}, Quat{}, 1.0f});
        add(0, {{0, 0.25f, 0}, Quat{}, 1.0f});
        add(1, {{-0.08f, 0.2f, 0}, Quat{}, 1.0f});
        add(2, {{-0.12f, 0, 0}, FromAxisAngle({0, 0, 1}, 70.0f * kDegrees), 1.0f});
        add(3, {{-0.28f, 0, 0}, FromAxisAngle({0, 1, 0}, 30.0f * kDegrees), 1.0f});
        add(4, {{-0.26f, 0, 0}, Quat{}, 1.0f});
        add(5, {{-0.08f, 0, 0}, Quat{}, 1.0f});
        return local;
    }

    // Tier 1: shoulder cone with twist alignment, elbow hinge 0 to 145 degrees, free hand.
    std::vector<IKJoint> ArmJoints(float coneDegrees = 110.0f)
    {
        IKJoint upper;
        upper.bone = 3;
        upper.limit.type = JointLimitType::Cone;
        upper.limit.axis = {-1, 0, 0};
        upper.limit.maxAngle = coneDegrees * kDegrees;
        upper.limit.maxTwist = 70.0f * kDegrees;
        IKJoint fore;
        fore.bone = 4;
        fore.limit.type = JointLimitType::Hinge;
        fore.limit.axis = {0, 1, 0};
        fore.limit.minAngle = 0.0f;
        fore.limit.maxAngle = 145.0f * kDegrees;
        IKJoint hand;
        hand.bone = 5;
        return {upper, fore, hand};
    }

    // Tier 2: spine and clavicle cones around the animated pose, followed by the arm.
    std::vector<IKJoint> SpineTier(float spineStiffness, float clavicleStiffness)
    {
        auto cone = [](int bone, float degrees, float stiffness)
        {
            IKJoint joint;
            joint.bone = bone;
            joint.limit.type = JointLimitType::Cone;
            joint.limit.followAnimation = true;
            joint.limit.maxAngle = degrees * kDegrees;
            joint.stiffness = stiffness;
            return joint;
        };
        std::vector<IKJoint> joints{cone(1, 15.0f, spineStiffness),
                                    cone(2, 20.0f, clavicleStiffness)};
        for (const IKJoint& joint : ArmJoints())
            joints.push_back(joint);
        return joints;
    }

    float AngleBetween(const Quat& a, const Quat& b)
    {
        return 2.0f * std::acos(std::min(1.0f, std::abs(Dot(a, b))));
    }

    float AngleBetween(const Vec3& a, const Vec3& b)
    {
        return std::acos(std::clamp(Dot(Normalize(a), Normalize(b)), -1.0f, 1.0f));
    }

    ChainSolveSettings PoleSettings()
    {
        ChainSolveSettings settings;
        settings.pole = Normalize(Vec3{-0.3f, -1.0f, -0.5f});
        return settings;
    }

    // Shared checks on a solved arm: world pose consistent with locals, bone lengths and the
    // hand rotation unchanged, spine untouched, elbow in its hinge plane and limits, upper arm
    // inside its cone, and (optionally) the elbow on the pole side of the shoulder-wrist line.
    void CheckArmPose(const Skeleton& skeleton,
                      const std::vector<VQS>& original,
                      const std::vector<VQS>& local,
                      const std::vector<VQS>& world,
                      bool expectPoleSide)
    {
        std::vector<VQS> rebuilt = WorldFromLocal(skeleton, local);
        for (std::size_t i = 0; i < world.size(); ++i)
            CHECK(Near(rebuilt[i].v, world[i].v, 1e-4f));

        std::vector<VQS> originalWorld = WorldFromLocal(skeleton, original);
        for (int bone : {4, 5, 6})
            CHECK_NEAR(Length(world[bone].v - world[bone - 1].v),
                       Length(originalWorld[bone].v - originalWorld[bone - 1].v),
                       1e-4f);
        CHECK(Test::NearRotation(local[5].q, original[5].q, 0));
        for (int bone : {0, 1, 2})
            CHECK(Test::NearRotation(local[bone].q, original[bone].q, 0));

        Vec3 straight, axis;
        CHECK(HingeFrame(original, 4, 5, {0, 1, 0}, straight, axis));
        Vec3 forearm = Normalize(Rotate(local[4].q, local[5].v));
        CHECK(std::abs(Dot(forearm, axis)) < 1e-3f);
        float flex = std::atan2(Dot(Cross(straight, forearm), axis), Dot(straight, forearm));
        CHECK(flex >= -1e-3f);
        CHECK(flex <= 145.0f * kDegrees + 1e-3f);

        Vec3 upper = Rotate(local[3].q, Normalize(local[4].v));
        CHECK(AngleBetween(upper, Vec3{-1, 0, 0}) <= 110.0f * kDegrees + 1e-3f);

        if (expectPoleSide)
        {
            Vec3 shoulder = world[3].v;
            Vec3 line = Normalize(world[5].v - shoulder);
            Vec3 elbow = world[4].v - shoulder;
            elbow = elbow - line * Dot(elbow, line);
            Vec3 pole = PoleSettings().pole;
            pole = pole - line * Dot(pole, line);
            CHECK(Dot(elbow, pole) > 0.0f);
        }
    }

    // Constrained solve: seven reachable targets converge with every limit kept, a target behind
    // the back fails only while constraints are on, a narrow cone is respected, a target already
    // reached costs no iterations, and broken chains throw.
    void CheckConstrainedArm()
    {
        Skeleton skeleton;
        const std::vector<VQS> original = ArmLocalPose(skeleton);
        const std::vector<VQS> originalWorld = WorldFromLocal(skeleton, original);
        const Vec3 shoulder = originalWorld[3].v;

        Vec3 straight, axis;
        CHECK(HingeFrame(original, 4, 5, {0, 1, 0}, straight, axis));
        CHECK(Near(straight, {-1, 0, 0}, 1e-5f));
        CHECK(std::abs(Dot(straight, axis)) < 1e-5f);
        std::vector<VQS> straightArm = original;
        straightArm[4].q = Quat{};
        CHECK(!HingeFrame(straightArm, 4, 5, {0, 1, 0}, straight, axis));
        CHECK(Near(axis, {0, 1, 0}, 1e-5f));

        for (const Vec3& offset : {Vec3{-0.3f, -0.2f, 0.2f},
                                   Vec3{-0.1f, 0, 0.4f},
                                   Vec3{0.1f, 0.1f, 0.35f},
                                   Vec3{-0.4f, 0.2f, 0.1f},
                                   Vec3{-0.2f, -0.4f, 0},
                                   Vec3{0, 0.3f, 0.3f},
                                   Vec3{0.2f, -0.1f, 0.2f}})
        {
            std::vector<VQS> local = original;
            std::vector<VQS> world = originalWorld;
            ChainSolveResult result =
                SolveChain(skeleton, ArmJoints(), shoulder + offset, PoleSettings(), local, world);
            CHECK(result.reachable);
            CHECK(result.converged);
            CHECK(result.iterations <= 32);
            CHECK(Near(world[5].v, shoulder + offset, 2e-3f));
            CheckArmPose(skeleton, original, local, world, true);
        }

        std::vector<VQS> local = original;
        std::vector<VQS> world = originalWorld;
        ChainSolveResult behind = SolveChain(skeleton,
                                             ArmJoints(),
                                             shoulder + Vec3{-0.1f, -0.2f, -0.2f},
                                             PoleSettings(),
                                             local,
                                             world);
        CHECK(behind.reachable);
        CHECK(!behind.converged);
        CheckArmPose(skeleton, original, local, world, false);

        ChainSolveSettings unconstrainedSettings = PoleSettings();
        unconstrainedSettings.useConstraints = false;
        unconstrainedSettings.usePole = false;
        local = original;
        world = originalWorld;
        ChainSolveResult unconstrained = SolveChain(skeleton,
                                                    ArmJoints(),
                                                    shoulder + Vec3{-0.1f, -0.2f, -0.2f},
                                                    unconstrainedSettings,
                                                    local,
                                                    world);
        CHECK(unconstrained.converged);

        local = original;
        world = originalWorld;
        ChainSolveResult coned = SolveChain(skeleton,
                                            ArmJoints(30.0f),
                                            shoulder + Vec3{0, 0, 0.5f},
                                            PoleSettings(),
                                            local,
                                            world);
        CHECK(!coned.converged);
        Vec3 upper = Rotate(local[3].q, Normalize(local[4].v));
        CHECK(AngleBetween(upper, Vec3{-1, 0, 0}) <= 30.0f * kDegrees + 1e-3f);

        local = original;
        world = originalWorld;
        ChainSolveResult none =
            SolveChain(skeleton, ArmJoints(), world[5].v, PoleSettings(), local, world);
        CHECK(none.converged);
        CHECK(none.iterations == 0);
        for (std::size_t i = 0; i < local.size(); ++i)
            CHECK(Test::NearRotation(local[i].q, original[i].q, 0));

        std::vector<IKJoint> broken{ArmJoints()[0], ArmJoints()[2]};
        CHECK(Throws<std::invalid_argument>(
            [&] { SolveChain(skeleton, broken, shoulder, PoleSettings(), local, world); }));
        std::vector<IKJoint> single{ArmJoints()[0]};
        CHECK(Throws<std::invalid_argument>(
            [&] { SolveChain(skeleton, single, shoulder, PoleSettings(), local, world); }));
        std::vector<std::vector<IKJoint>> noTiers;
        CHECK(Throws<std::invalid_argument>(
            [&] { SolvePrioritized(skeleton, noTiers, shoulder, PoleSettings(), local, world); }));
    }

    // Priority: a near target is solved by the arm alone, a target just past arm length needs
    // tier 2 within the spine limits, and a stiffer spine bends less than a loose one.
    void CheckPriorityTiers()
    {
        Skeleton skeleton;
        const std::vector<VQS> original = ArmLocalPose(skeleton);
        const std::vector<VQS> originalWorld = WorldFromLocal(skeleton, original);
        const Vec3 shoulder = originalWorld[3].v;
        const std::vector<std::vector<IKJoint>> stiffTiers{ArmJoints(), SpineTier(0.25f, 0.5f)};
        const std::vector<std::vector<IKJoint>> looseTiers{ArmJoints(), SpineTier(1.0f, 1.0f)};
        const std::vector<std::vector<IKJoint>> armTier{ArmJoints()};

        std::vector<VQS> local = original;
        std::vector<VQS> world = originalWorld;
        ChainSolveResult close = SolvePrioritized(skeleton,
                                                  stiffTiers,
                                                  shoulder + Vec3{-0.3f, -0.2f, 0.2f},
                                                  PoleSettings(),
                                                  local,
                                                  world);
        CHECK(close.converged);
        CHECK(close.tier == 1);
        for (int bone : {0, 1, 2})
            CHECK(Test::NearRotation(local[bone].q, original[bone].q, 0));

        const Vec3 farTarget = shoulder + Vec3{-0.35f, -0.05f, 0.55f} * 0.86f;
        local = original;
        world = originalWorld;
        ChainSolveResult armOnly =
            SolvePrioritized(skeleton, armTier, farTarget, PoleSettings(), local, world);
        CHECK(!armOnly.converged);
        CHECK(armOnly.tier == 1);

        local = original;
        world = originalWorld;
        ChainSolveResult stiff =
            SolvePrioritized(skeleton, stiffTiers, farTarget, PoleSettings(), local, world);
        CHECK(stiff.converged);
        CHECK(stiff.tier == 2);
        CHECK(stiff.error < armOnly.error);
        CHECK(Near(world[5].v, farTarget, 2e-3f));
        float stiffSpine = AngleBetween(local[1].q, original[1].q);
        CHECK(stiffSpine <= 15.0f * kDegrees + 1e-3f);
        CHECK(AngleBetween(local[2].q, original[2].q) <= 20.0f * kDegrees + 1e-3f);
        CHECK(Test::NearRotation(local[0].q, original[0].q, 0));
        CHECK(Near(world[1].v, originalWorld[1].v, 0));

        local = original;
        world = originalWorld;
        ChainSolveResult loose =
            SolvePrioritized(skeleton, looseTiers, farTarget, PoleSettings(), local, world);
        CHECK(loose.converged);
        CHECK(loose.tier == 2);
        CHECK(AngleBetween(local[1].q, original[1].q) > stiffSpine + 2.0f * kDegrees);

        local = original;
        world = originalWorld;
        ChainSolveResult limited =
            SolvePrioritized(skeleton, stiffTiers, farTarget, PoleSettings(), local, world, 1);
        CHECK(limited.tier == 1);
        CHECK(!limited.converged);
        CHECK_NEAR(limited.error, armOnly.error, 1e-6f);
        for (int bone : {0, 1, 2})
            CHECK(Test::NearRotation(local[bone].q, original[bone].q, 0));
        auto noTier = [&]
        { SolvePrioritized(skeleton, stiffTiers, farTarget, PoleSettings(), local, world, 0); };
        CHECK(Throws<std::invalid_argument>(noTier));
    }

    // Roll of a solved local rotation about its own bone, measured against the swing-only
    // rotation from the animated pose (what the solver would give with no twist at all).
    float TwistFromAnimation(const Quat& solved, const Quat& animated, const Vec3& offset)
    {
        Vec3 direction = Normalize(Rotate(solved, offset));
        Quat swing = Normalize(FromTo(Rotate(animated, offset), direction) * animated);
        Quat roll = Normalize(solved * Conjugate(swing));
        float twist = 2.0f * std::atan2(Dot(Vec3{roll.x, roll.y, roll.z}, direction), roll.w);
        if (twist > 3.14159265f)
            twist -= 2.0f * 3.14159265f;
        else if (twist < -3.14159265f)
            twist += 2.0f * 3.14159265f;
        return twist;
    }

    // Twist limit: a stiff shoulder starts every pass from the last one's rotation, yet its
    // total roll against the animation stays within maxTwist for every target.
    void CheckTwistLimit()
    {
        Skeleton skeleton;
        const std::vector<VQS> original = ArmLocalPose(skeleton);
        const std::vector<VQS> originalWorld = WorldFromLocal(skeleton, original);
        const Vec3 shoulder = originalWorld[3].v;
        const Vec3 offset = Normalize(original[4].v);

        std::vector<IKJoint> joints = ArmJoints();
        joints[0].stiffness = 0.5f;
        joints[0].limit.maxTwist = 10.0f * kDegrees;

        for (const Vec3& target : {Vec3{-0.3f, -0.2f, 0.2f},
                                   Vec3{-0.1f, 0, 0.4f},
                                   Vec3{0.1f, 0.1f, 0.35f},
                                   Vec3{-0.4f, 0.2f, 0.1f},
                                   Vec3{0, 0.3f, 0.3f},
                                   Vec3{0.2f, -0.1f, 0.2f}})
        {
            std::vector<VQS> local = original;
            std::vector<VQS> world = originalWorld;
            SolveChain(skeleton, joints, shoulder + target, PoleSettings(), local, world);
            float twist = TwistFromAnimation(local[3].q, original[3].q, offset);
            CHECK(std::abs(twist) <= 10.0f * kDegrees + 1e-3f);
            CHECK(std::isfinite(twist));
        }
    }
}  // namespace

void RunFabrikTests()
{
    CheckSolver();
    CheckChain();
    CheckConstrainedArm();
    CheckPriorityTiers();
    CheckTwistLimit();
}
