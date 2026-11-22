layout(std140, binding = 2) uniform PbfBodies
{
    ivec4 bodyInfo;
    vec4 bodyParams;
    vec4 bodyCom[16];
    vec4 bodyCObj[16];
    vec4 bodyAxisX[16];
    vec4 bodyAxisY[16];
    vec4 bodyAxisZ[16];
    vec4 bodyHalf[16];
    vec4 bodyVel[16];
    vec4 bodyOmega[16];
};

layout(std430, binding = 15) buffer BodyImpulses { int bodyImpulse[]; };

uniform float uBodyTime;

shared int sBodyImpulse[96];

int BodyCount()
{
    return bodyInfo.x;
}

bool TwoWayCoupling()
{
    return bodyInfo.y != 0;
}

float FrictionCoefficient()
{
    return bodyParams.y;
}

vec3 RotateByVector(vec3 v, vec3 w)
{
    float angle = length(w);
    if (angle < 1e-8)
        return v;
    vec3 k = w / angle;
    float c = cos(angle);
    float s = sin(angle);
    return v * c + cross(k, v) * s + k * (dot(k, v) * (1.0 - c));
}

vec3 BodyComAt(int b)
{
    return bodyCom[b].xyz + uBodyTime * bodyVel[b].xyz;
}

vec3 BodyToLocal(int b, vec3 p)
{
    vec3 r = RotateByVector(p - BodyComAt(b), -uBodyTime * bodyOmega[b].xyz);
    return vec3(dot(bodyAxisX[b].xyz, r), dot(bodyAxisY[b].xyz, r), dot(bodyAxisZ[b].xyz, r)) + bodyCObj[b].xyz;
}

vec3 BodyDirToWorld(int b, vec3 d)
{
    vec3 r = bodyAxisX[b].xyz * d.x + bodyAxisY[b].xyz * d.y + bodyAxisZ[b].xyz * d.z;
    return RotateByVector(r, uBodyTime * bodyOmega[b].xyz);
}

vec3 BodyToWorld(int b, vec3 l)
{
    return BodyComAt(b) + BodyDirToWorld(b, l - bodyCObj[b].xyz);
}

vec3 BodyPointVelocity(int b, vec3 p)
{
    return bodyVel[b].xyz + cross(bodyOmega[b].xyz, p - BodyComAt(b));
}

bool NearBody(int b, vec3 p, float reach)
{
    vec3 d = p - BodyComAt(b);
    float R = bodyHalf[b].w + reach;
    return dot(d, d) < R * R;
}

void BeginBodyImpulses()
{
    if (gl_LocalInvocationIndex < 96u)
        sBodyImpulse[gl_LocalInvocationIndex] = 0;
    memoryBarrierShared();
    barrier();
}

void AddBodyImpulse(int b, vec3 impulse, vec3 point)
{
    float s = bodyParams.x;
    vec3 torque = cross(point - BodyComAt(b), impulse);
    int base = b * 6;
    atomicAdd(sBodyImpulse[base + 0], int(round(impulse.x * s)));
    atomicAdd(sBodyImpulse[base + 1], int(round(impulse.y * s)));
    atomicAdd(sBodyImpulse[base + 2], int(round(impulse.z * s)));
    atomicAdd(sBodyImpulse[base + 3], int(round(torque.x * s)));
    atomicAdd(sBodyImpulse[base + 4], int(round(torque.y * s)));
    atomicAdd(sBodyImpulse[base + 5], int(round(torque.z * s)));
}

void EndBodyImpulses()
{
    memoryBarrierShared();
    barrier();
    uint l = gl_LocalInvocationIndex;
    if (l < uint(BodyCount() * 6) && sBodyImpulse[l] != 0)
        atomicAdd(bodyImpulse[l], sBodyImpulse[l]);
}

vec3 CollideBodies(vec3 p)
{
    float r = ParticleRadius();
    bool react = TwoWayCoupling();
    float impulsePerMeter = mass / TimeStep();
    for (int b = 0; b < BodyCount(); ++b)
    {
        if (!NearBody(b, p, r))
            continue;
        vec3 l = BodyToLocal(b, p);
        vec3 halfExt = bodyHalf[b].xyz;
        vec3 q = abs(l) - halfExt;
        float outside = max(q.x, max(q.y, q.z));
        if (outside >= r)
            continue;
        vec3 target = l;
        if (outside < 0.0)
        {
            if (q.x >= q.y && q.x >= q.z)
                target.x = sign(l.x + 1e-9) * (halfExt.x + r);
            else if (q.y >= q.z)
                target.y = sign(l.y + 1e-9) * (halfExt.y + r);
            else
                target.z = sign(l.z + 1e-9) * (halfExt.z + r);
        }
        else
        {
            vec3 closest = clamp(l, -halfExt, halfExt);
            vec3 d = l - closest;
            float dist = length(d);
            if (dist >= r || dist < 1e-7)
                continue;
            target = closest + d * (r / dist);
        }
        vec3 np = BodyToWorld(b, target);
        if (react)
            AddBodyImpulse(b, (p - np) * impulsePerMeter, BodyToWorld(b, clamp(target, -halfExt, halfExt)));
        p = np;
    }
    return p;
}

vec3 BodyFriction(vec3 p, vec3 v)
{
    float f = FrictionCoefficient();
    if (f <= 0.0)
        return v;
    float reach = 1.5 * ParticleRadius();
    for (int b = 0; b < BodyCount(); ++b)
    {
        if (!NearBody(b, p, reach))
            continue;
        vec3 l = BodyToLocal(b, p);
        vec3 closest = clamp(l, -bodyHalf[b].xyz, bodyHalf[b].xyz);
        vec3 d = l - closest;
        float dist = length(d);
        if (dist >= reach || dist < 1e-7)
            continue;
        vec3 n = BodyDirToWorld(b, d / dist);
        vec3 rel = v - BodyPointVelocity(b, p);
        vec3 dv = -f * (rel - dot(rel, n) * n);
        v += dv;
        if (TwoWayCoupling())
            AddBodyImpulse(b, -mass * dv, BodyToWorld(b, closest));
        break;
    }
    return v;
}
