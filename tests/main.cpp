#include "MathTests.h"
#include "TestFramework.h"

int main()
{
    RunVec3Tests();
    RunQuatTests();
    RunMat4Tests();
    RunVQSTests();
    return Test::Summarize();
}
