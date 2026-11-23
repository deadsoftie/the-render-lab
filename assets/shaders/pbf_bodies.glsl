// Planks as oriented boxes. uBodyTime (<= 0) moves them back to the current substep.
// bodyHalf.w = bounding radius.
layout(std140, binding = 3) uniform PbfBodies
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

// Impulse coupling: force and torque per body as fixed-point ints (no float atomics in GL 4.3).
layout(std430, binding = 15) buffer BodyImpulses { int bodyImpulse[]; };

uniform float uBodyTime;

// Workgroup partial sums, 16 bodies x 6 components.
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

// Rodrigues rotation of v by |w| radians about w.
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

// World point to body model space at the substep time.
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

// Contain barriers: call outside the particle-count check.
void BeginBodyImpulses()
{
    for (uint k = gl_LocalInvocationIndex; k < 96u; k += uint(PBF_GROUP_SIZE))
        sBodyImpulse[k] = 0;
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
    uint used = uint(BodyCount() * 6);
    for (uint l = gl_LocalInvocationIndex; l < used; l += uint(PBF_GROUP_SIZE))
        if (sBodyImpulse[l] != 0)
            atomicAdd(bodyImpulse[l], sBodyImpulse[l]);
}

// Pushes a particle out of every plank. Impulse coupling adds the reaction m dx / dt to the plank.
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

// Damps tangential velocity relative to a nearby plank; reaction goes to the plank.
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
