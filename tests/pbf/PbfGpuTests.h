#pragma once
#include <string>

std::string ShaderPath(const char* file);

void RunGpuBufferTests();
void RunGpuQueryTimerTests();
void RunPbfShaderCompileTests();
void RunGridTests();
void RunSolverTests();
void RunVelocityTests();
void RunCouplingTests();
void RunSurfaceTests();
void RunSmoothingTests();
