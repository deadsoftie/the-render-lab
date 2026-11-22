layout(std140, binding = 0) uniform PbfParams
{
    vec4 domainMin;
    vec4 domainMax;
    vec4 gravity;
    ivec4 gridDims;
    float rho0;
    float invRho0;
    float epsilon;
    float sCorrK;
    float sCorrDq;
    float sCorrN;
    float xsphC;
    float vorticityEps;
    float poly6;
    float spikyGrad;
    float wDq;
    float mass;
    int particleCount;
    int iterations;
    int pad0;
    int pad1;
};

float SmoothingRadius()
{
    return domainMin.w;
}

float ParticleRadius()
{
    return domainMax.w;
}

float TimeStep()
{
    return gravity.w;
}
