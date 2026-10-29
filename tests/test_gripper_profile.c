/* Host: gcc -std=c99 -O2 -Wall -Wextra -Werror tests/test_gripper_profile.c
 *           -lm -o /tmp/test_gripper_profile && /tmp/test_gripper_profile */
#include <assert.h>
#include <stdint.h>
#include <stdio.h>
#include "../Middleware/smd_gripper_profile.h"

/* Independent fine-step integration of the jerk-limited braking trajectory. */
static void check_brake(float v, float a, float end, float acc, float jerk)
{
    float prediction = SMD_GripperBrakeDistance(v, a, end, acc, jerk);
    double distance = 0.0;
    double speed = v, acceleration = a;
    const double dt = 0.00001;
    unsigned i;
    assert(isfinite(prediction) && prediction >= 0.0f);
    for (i = 0; speed > end && i < 10000000u; ++i)
    {
        double release = acceleration < 0.0 ? acceleration * acceleration / (2.0 * jerk) : 0.0;
        if (acceleration < 0.0 && speed - end <= release)
            acceleration += jerk * dt;
        else
            acceleration -= jerk * dt;
        if (acceleration < -acc) acceleration = -acc;
        distance += speed * dt;
        speed += acceleration * dt;
    }
    assert(i < 10000000u);
    assert(fabs(distance - prediction) < 0.5 + prediction * 0.001);
}

/* Event model: ARR/CCR writes latch at overflow; counting/gating occurs at
 * the falling edge, independently of the 1ms planner. Positions are integer
 * pulses, including one-pulse and reverse moves (distance is absolute). */
static void check_move(unsigned length, float maximum, float acc, float jerk)
{
    float v = 50.0f, a = 0.0f;
    unsigned pulses = 0, low_speed_pulses = 0;
    unsigned pending_period = 20000u, period = 20000u;
    uint64_t tick = 1000u, rise = 1u, fall = 10001u;
    uint64_t now = 0;
    int braking = 0;
    while (pulses < length)
    {
        if (fall <= tick && fall < rise)
        {
            now = fall;
            ++pulses;
            if (period >= 9000u) ++low_speed_pulses;
            fall = UINT64_MAX;
            if (pulses == length) break; /* TIM3 OPM closes TIM5's gate. */
        }
        else if (rise <= tick)
        {
            now = rise;
            period = pending_period;
            fall = now + period / 2u;
            rise = now + period;
        }
        else
        {
            float end = maximum < 100.0f ? maximum : 100.0f;
            float distance = SMD_GripperBrakeDistance(v, a, end, acc, jerk);
            if ((float)(length - pulses) <= distance + v * 0.003f + 2.0f) braking = 1;
            v = SMD_GripperProfileTick(v, &a, braking ? end : maximum, acc, jerk, 0.001f);
            assert(isfinite(v) && isfinite(a));
            assert(v >= 49.99f && v <= maximum + 0.01f);
            assert(fabsf(a) <= acc + 0.01f);
            pending_period = 1000000u / (unsigned)(v + 0.5f);
            now = tick;
            tick += 1000u;
        }
        assert(now < 2000000000u);
    }
    assert(pulses == length);
    if (length >= 1000u) assert(1000000.0 / period <= 200.0);
    if (maximum > 1000.0f && length >= 1000u)
        assert(low_speed_pulses < 64u); /* Includes startup; old tail alone ~100. */
    printf("move %5u pulses, max %4.0f Hz: %7.3f s, low-speed pulses %u, end %.0f Hz\n",
           length, maximum, now / 1000000.0, low_speed_pulses, 1000000.0 / period);
}

int main(void)
{
    static const unsigned distances[] = {1, 2, 3, 10, 100, 1000, 10000, 65534};
    static const float speeds[] = {100, 1600, 4000, 8000};
    unsigned i, j;
    check_brake(8000, 0, 100, 4000, 8000);
    check_brake(4000, 3000, 100, 4000, 8000);
    check_brake(2000, -2000, 100, 4000, 8000);
    check_brake(150, -2000, 100, 4000, 8000);
    check_brake(500, 0, 100, 4000, 8000);
    assert(SMD_GripperBrakeDistance(50, 0, 100, 4000, 8000) == 0.0f);
    for (i = 0; i < sizeof(distances) / sizeof(distances[0]); ++i)
        for (j = 0; j < sizeof(speeds) / sizeof(speeds[0]); ++j)
            check_move(distances[i], speeds[j], 4000, 8000);
    puts("gripper profile tests passed");
    return 0;
}
