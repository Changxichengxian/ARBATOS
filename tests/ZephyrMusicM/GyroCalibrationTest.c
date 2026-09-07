/* 使用真实校准算法，检查异常采样不会被当作静止校准成功。 */
#include "GyroZeroCali.h"
#include <assert.h>
#include <stdio.h>

int main(void)
{
    GyroZeroCaliSampleState sample = {0};
    float gyro[3] = {0.001f, -0.002f, 0.003f};
    float accel[3] = {0, 0, 9.80665f};
    for (unsigned i = 0; i < 2999; i++) {
        assert(!GyroZeroCaliCollectSample(&sample, gyro, accel, 3000));
    }
    assert(GyroZeroCaliCollectSample(&sample, gyro, accel, 3000));
    float offset[3];
    GyroZeroCaliCalcOffset(&sample, offset);
    for (unsigned i = 0; i < 3; i++) assert(fabsf(offset[i] + gyro[i]) < 0.000001f);

    gyro[0] = NAN;
    assert(!GyroZeroCaliCollectSample(&sample, gyro, accel, 3000));
    assert(sample.sample_count == 0);
    gyro[0] = INFINITY;
    assert(!GyroZeroCaliCollectSample(&sample, gyro, accel, 3000));
    gyro[0] = 0.2f;
    assert(!GyroZeroCaliCollectSample(&sample, gyro, accel, 3000));
    gyro[0] = 0;
    accel[2] = NAN;
    assert(!GyroZeroCaliCollectSample(&sample, gyro, accel, 3000));
    accel[2] = 0;
    assert(!GyroZeroCaliCollectSample(&sample, gyro, accel, 3000));
    accel[2] = 9.80665f;
    assert(!GyroZeroCaliCollectSample(&sample, gyro, accel, 3000));
    assert(sample.sample_count == 1);
    accel[2] = 12;
    assert(!GyroZeroCaliCollectSample(&sample, gyro, accel, 3000));
    assert(sample.sample_count == 0);

    GyroZeroCaliTempState temp = {0};
    assert(!GyroZeroCaliTempStableUpdate(&temp, 40, 40, 100, 1));
    assert(!GyroZeroCaliTempStableUpdate(&temp, 40, 40, 2099, 1));
    assert(GyroZeroCaliTempStableUpdate(&temp, 40, 40, 2100, 1));
    assert(!GyroZeroCaliTempStableUpdate(&temp, NAN, 40, 2200, 1));
    assert(temp.stable_since_ms == 0);
    assert(!GyroZeroCaliTempStableUpdate(&temp, 40, 40, 5000, 0));
    assert(!GyroZeroCaliTempStableUpdate(&temp, 50, 40, 5000, 1));
    puts("PASS: bias, sample count, moving board, invalid sensor values, temperature stability");
    return 0;
}
