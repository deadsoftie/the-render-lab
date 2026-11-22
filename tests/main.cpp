#include "MathTests.h"
#include "MotionTests.h"
#include "PhysicsTests.h"
#include "TestFramework.h"

int main()
{
    RunVec3Tests();
    RunQuatTests();
    RunMat4Tests();
    RunMat3Tests();
    RunVQSTests();
    RunPathTests();
    RunArcLengthTests();
    RunArcLengthLookupTests();
    RunSpeedProfileTests();
    RunPathFollowerTests();
    RunAnimatorBlendTests();
    RunFabrikTests();
    RunApproachPlannerTests();
    RunReachControllerTests();
    RunRigidBodyTests();
    RunIntegratorTests();
    RunPhysicsWorldTests();
    return Test::Summarize();
}
