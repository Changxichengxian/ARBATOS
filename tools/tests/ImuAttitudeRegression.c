#include "ImuAttitude.h"
#include <assert.h>
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#define G 9.80665f

static void near(float actual, float expected, float tolerance)
{
    if (fabsf(actual - expected) > tolerance) {
        fprintf(stderr, "actual %.6f expected %.6f tolerance %.6f\n",
                actual, expected, tolerance);
        assert(0);
    }
}

static void set_filter(ImuAttitude *s, const float a[3])
{
    for (unsigned i = 0; i < 3; ++i)
        second_order_filter_reset(&s->accelFilter[i], a[i]);
}

static void tilt_and_input(void)
{
    ImuAttitude s, before;
    const float zero[3] = {0}, tilted[3] = {0, G * 0.5f, G * 0.8660254f};
    const float bad[3] = {NAN, 0, G};
    ImuAttitudeInit(&s);
    assert(ImuAttitudeUpdate(&s, .001f, 1, zero, tilted, NULL, 0));
    near(s.angle[1], PI / 6, .01f);
    assert(s.acc.trust > .99f && !s.acc.rejected);
    before = s;
    assert(!ImuAttitudeUpdate(&s, .001f, 2, zero, bad, NULL, 0));
    assert(memcmp(&s, &before, sizeof(s)) == 0);
    assert(!ImuAttitudeUpdate(&s, NAN, 2, zero, tilted, NULL, 0));
    assert(memcmp(&s, &before, sizeof(s)) == 0);
    ImuAttitudeSetAccRef(&s, G * 1.02f);
    near(s.accRef, G * 1.02f, .0001f);
    ImuAttitudeSetAccRef(&s, NAN);
    ImuAttitudeSetAccRef(&s, G * 2);
    near(s.accRef, G * 1.02f, .0001f);
}

static void trust_and_gain(void)
{
    ImuAttitude s;
    const float zero[3] = {0}, up[3] = {0, 0, G};
    const float shifted[3] = {0, 0, G * 1.2f};
    const float turning[3] = {0, 0, .5f};
    ImuAttitudeInit(&s);
    assert(ImuAttitudeUpdate(&s, .001f, UINT32_MAX - 100u, zero, up, NULL, 0));
    near(s.acc.kpGain, 2.5f, .01f);
    assert(ImuAttitudeUpdate(&s, .001f, UINT32_MAX - 99u, zero, shifted, NULL, 0));
    assert(s.filteredAcc[2] > G && s.filteredAcc[2] < shifted[2]);
    set_filter(&s, shifted);
    assert(ImuAttitudeUpdate(&s, .001f, UINT32_MAX - 90u, zero, shifted, NULL, 0));
    assert(s.acc.rejected && s.acc.trust == 0);
    set_filter(&s, up);
    assert(ImuAttitudeUpdate(&s, .001f, 88u, zero, up, NULL, 0));
    assert(s.acc.rejected);
    assert(ImuAttitudeUpdate(&s, .001f, 89u, zero, up, NULL, 0));
    assert(!s.acc.rejected);
    assert(ImuAttitudeUpdate(&s, .001f, 90u, turning, up, NULL, 0));
    near(s.acc.kpGain, .25f, .01f);
    assert(ImuAttitudeUpdate(&s, .001f, 340u, zero, up, NULL, 0));
    near(s.acc.kpGain, .25f, .01f);
    assert(ImuAttitudeUpdate(&s, .001f, 341u, zero, up, NULL, 0));
    near(s.acc.kpGain, 25.f, .1f);
    assert(ImuAttitudeUpdate(&s, .001f, 841u, zero, up, NULL, 0));
    assert(ImuAttitudeUpdate(&s, .001f, 842u, zero, up, NULL, 0));
    near(s.acc.kpGain, 2.5f, .05f);
}

static void yaw_and_modes(void)
{
    ImuAttitude s;
    const float turn[3] = {0, 0, 1.f}, zero[3] = {0}, up[3] = {0, 0, G};
    const float mag[3] = {1, 0, 0}, no_mag[3] = {0};
    ImuAttitudeInit(&s);
    assert(ImuAttitudeUpdate(&s, .001f, 0, zero, up, NULL, 0));
    for (uint32_t i = 1; i <= 15000; ++i)
        assert(ImuAttitudeUpdate(&s, .001f, i, turn, up, NULL, 0));
    near(fabsf(s.continuousYaw), 15.f, .04f);
    near(s.angle[0], s.continuousYaw, .01f);
    float previous = s.continuousYaw;
    assert(ImuAttitudeUpdate(&s, .001f, 15001, zero, up, mag, 1));
    assert(s.mode == 1);
    assert(fabsf(s.continuousYaw - previous) < .02f);
    previous = s.continuousYaw;
    assert(ImuAttitudeUpdate(&s, .001f, 15002, zero, up, no_mag, 1));
    assert(s.mode == 0);
    assert(fabsf(s.continuousYaw - previous) < .02f);
}

static void trust_thresholds(void)
{
    ImuAttitude s;
    const float zero[3] = {0}, up[3] = {0, 0, G};
    const float angle12[3] = {G * 0.20791169f, 0, G * 0.9781476f};
    const float norm5[3] = {0, 0, G * 1.05f};
    const float angle20[3] = {G * 0.34202014f, 0, G * 0.93969262f};
    ImuAttitudeInit(&s);
    assert(ImuAttitudeUpdate(&s, .001f, 0, zero, up, NULL, 0));
    set_filter(&s, angle12);
    assert(ImuAttitudeUpdate(&s, .001f, 1, zero, angle12, NULL, 0));
    near(s.acc.trust, .5f, .02f);
    set_filter(&s, norm5);
    assert(ImuAttitudeUpdate(&s, .001f, 2, zero, norm5, NULL, 0));
    near(s.acc.trust, .5f, .02f);
    set_filter(&s, angle20);
    assert(ImuAttitudeUpdate(&s, .001f, 3, zero, angle20, NULL, 0));
    assert(s.acc.rejected && s.acc.trust == 0);
    near(s.fusedAcc[2], G, .01f);
}

int main(void)
{
    tilt_and_input();
    trust_and_gain();
    trust_thresholds();
    yaw_and_modes();
    puts("姿态回归通过");
    return 0;
}
