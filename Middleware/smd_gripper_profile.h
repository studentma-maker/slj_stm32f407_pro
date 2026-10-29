#ifndef SMD_GRIPPER_PROFILE_H
#define SMD_GRIPPER_PROFILE_H

#include <math.h>

/* Signed acceleration, Hz/s; jerk, Hz/s^2. Distance is in STEP pulses. */
static float SMD_GripperBrakeDistance(float v, float a, float end_v,
                                      float acc, float jerk)
{
    float peak, t1, t2, t3, distance, next_v;
    if (v <= end_v) return 0.0f;
    if (acc < 1.0f) acc = 1.0f;
    if (jerk < 1.0f) jerk = 1.0f;
    if (a > acc) a = acc;
    if (a < -acc) a = -acc;
    peak = sqrtf(jerk * (v - end_v) + 0.5f * a * a);
    /* Already decelerating harder than needed: release acceleration now. */
    if (a < 0.0f && peak < -a)
    {
        float discriminant = a * a - 2.0f * jerk * (v - end_v);
        if (discriminant < 0.0f) discriminant = 0.0f;
        float t = (-a - sqrtf(discriminant)) / jerk;
        return v * t + 0.5f * a * t * t + jerk * t * t * t / 6.0f;
    }
    if (peak > acc) peak = acc;
    t1 = (a + peak) / jerk;
    t3 = peak / jerk;
    next_v = v + a * t1 - 0.5f * jerk * t1 * t1;
    t2 = (next_v - end_v - 0.5f * peak * peak / jerk) / peak;
    if (t2 < 0.0f) t2 = 0.0f;
    distance = v * t1 + 0.5f * a * t1 * t1 - jerk * t1 * t1 * t1 / 6.0f;
    distance += next_v * t2 - 0.5f * peak * t2 * t2;
    next_v -= peak * t2;
    distance += next_v * t3 - 0.5f * peak * t3 * t3 + jerk * t3 * t3 * t3 / 6.0f;
    return distance;
}

static float SMD_GripperProfileTick(float v, float *a, float target,
                                   float acc, float jerk, float dt)
{
    float dv = target - v;
    float desired = 0.0f;
    float previous = *a;
    float next;
    if (acc < 1.0f) acc = 1.0f;
    if (jerk < 1.0f) jerk = 1.0f;
    if (dv > 0.0f)
        desired = (*a > 0.0f && dv <= *a * *a / (2.0f * jerk)) ? 0.0f : acc;
    else if (dv < 0.0f)
        desired = (*a < 0.0f && -dv <= *a * *a / (2.0f * jerk)) ? 0.0f : -acc;
    if (*a < desired)
    {
        *a += jerk * dt;
        if (*a > desired) *a = desired;
    }
    else if (*a > desired)
    {
        *a -= jerk * dt;
        if (*a < desired) *a = desired;
    }
    if (*a > acc) *a = acc;
    if (*a < -acc) *a = -acc;
    next = v + 0.5f * (previous + *a) * dt;
    if ((dv >= 0.0f && next >= target) || (dv <= 0.0f && next <= target))
    {
        next = target;
        *a = 0.0f;
    }
    return next > 0.0f ? next : 0.0f;
}

#endif
