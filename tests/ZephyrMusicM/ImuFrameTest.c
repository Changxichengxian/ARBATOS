#include <assert.h>
#include <math.h>
#include <stdio.h>
#include "ImuFrame.h"

static void Check(const float got[3], const float want[3])
{
    for (unsigned i = 0; i < 3; i++) assert(fabsf(got[i] - want[i]) < 0.000001f);
}

int main(void)
{
    float raw[3] = {1, 2, 3}, out[3];
    ImuFrameRotate(out, raw);
#if defined(CONFIG_ARBATOS_TARGET_HERO_M)
    const float rotated[3] = {1, 2, 3};
    const float oldBias[3] = {-2, 1, 3};
    assert(ImuFrameVersion() == 2);
#else
    const float rotated[3] = {2, -1, 3};
    const float oldBias[3] = {1, 2, 3};
    assert(ImuFrameVersion() == 1);
#endif
    Check(out, rotated);
    assert(ImuFrameBiasConvert(out, raw, 1));
    Check(out, oldBias);
    assert(ImuFrameBiasConvert(out, raw, 2));
    Check(out, rotated);
    assert(!ImuFrameBiasConvert(out, raw, 3));
    Check(out, rotated);

    /* 转换零偏与先在旧坐标系补偿再转换，结果必须相同。 */
    const float biasV1[3] = {0.001f, -0.002f, 0.003f};
    float bias[3], corrected[3], expected[3];
    float oldCorrected[3] = {raw[1] + biasV1[0], -raw[0] + biasV1[1], raw[2] + biasV1[2]};
    assert(ImuFrameBiasConvert(bias, biasV1, 1));
    ImuFrameRotate(corrected, raw);
    for (unsigned i = 0; i < 3; i++) corrected[i] += bias[i];
    assert(ImuFrameBiasConvert(expected, oldCorrected, 1));
    Check(corrected, expected);
    puts("PASS: IMU frame and calibration migration");
    return 0;
}
