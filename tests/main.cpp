#include "MathTests.h"
#include "MotionTests.h"
#include "TestFramework.h"

int main()
{
    RunVec3Tests();
    RunQuatTests();
    RunMat4Tests();
    RunVQSTests();
    RunPathTests();
    RunArcLengthTests();
    RunArcLengthLookupTests();
    return Test::Summarize();
}
