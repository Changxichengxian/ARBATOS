/* 外置减速器纯数学回归：默认兼容、理想变换和双向对称性。 */

#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "MotorTransmission.h"

static int TestCheck(int condition, const char *message)
{
    if (condition != 0)
    {
        return 1;
    }

    (void)fprintf(stderr, "FAIL: %s\n", message);
    return 0;
}

static uint32_t TestFloatBits(fp32 value)
{
    uint32_t bits = 0u;

    (void)memcpy(&bits, &value, sizeof(bits));
    return bits;
}

static int TestNear(fp32 left, fp32 right)
{
    const fp32 error = (left >= right) ? left - right : right - left;
    return (error <= 0.00001f) ? 1 : 0;
}

static int TestDirectDriveUnchanged(void)
{
    const fp32 values[] = {0.0f, -0.0f, 1.25f, -37.5f};
    const fp32 ratios[] = {0.0f, 1.0f, -2.0f, 0.5f, 10001.0f};

    for (uint32_t r = 0u; r < (uint32_t)(sizeof(ratios) / sizeof(ratios[0])); r++)
    {
        for (uint32_t i = 0u; i < (uint32_t)(sizeof(values) / sizeof(values[0])); i++)
        {
            const uint32_t expected = TestFloatBits(values[i]);

            if (!TestCheck(TestFloatBits(MotorTransPositionToMotor(values[i], ratios[r])) == expected &&
                               TestFloatBits(MotorTransPositionToOutput(values[i], ratios[r])) == expected &&
                               TestFloatBits(MotorTransGainToMotor(values[i], ratios[r])) == expected &&
                               TestFloatBits(MotorTransGainToOutput(values[i], ratios[r])) == expected &&
                               TestFloatBits(MotorTransTorqueToMotor(values[i], ratios[r])) == expected &&
                               TestFloatBits(MotorTransTorqueToOutput(values[i], ratios[r])) == expected,
                           "0/1 或异常减速比改变了原值位模式"))
            {
                return 0;
            }
        }
    }
    return 1;
}

static int TestIdealMapping(void)
{
    const fp32 ratio = 4.0f;

    return TestCheck(MotorTransPositionToMotor(2.0f, ratio) == 8.0f &&
                         MotorTransVelocityToMotor(-3.0f, ratio) == -12.0f,
                     "q/dq 未按减速比放大到电机侧") &&
           TestCheck(MotorTransGainToMotor(32.0f, ratio) == 2.0f,
                     "kp/kd 未按减速比平方缩小到电机侧") &&
           TestCheck(MotorTransTorqueToMotor(20.0f, ratio) == 5.0f,
                     "tau 未按减速比缩小到电机侧") &&
           TestCheck(MotorTransPositionToOutput(8.0f, ratio) == 2.0f &&
                         MotorTransVelocityToOutput(-12.0f, ratio) == -3.0f,
                     "位置或速度反馈未恢复到输出侧") &&
           TestCheck(MotorTransGainToOutput(2.0f, ratio) == 32.0f &&
                         MotorTransTorqueToOutput(5.0f, ratio) == 20.0f,
                     "增益或力矩未恢复到输出侧");
}

static int TestRoundTrip(void)
{
    const fp32 ratio = 2.5f;
    const fp32 value = 7.25f;

    return TestCheck(TestNear(MotorTransPositionToOutput(
                                  MotorTransPositionToMotor(value, ratio), ratio), value),
                     "位置双向换算不对称") &&
           TestCheck(TestNear(MotorTransVelocityToOutput(
                                  MotorTransVelocityToMotor(value, ratio), ratio), value),
                     "速度双向换算不对称") &&
           TestCheck(TestNear(MotorTransGainToOutput(
                                  MotorTransGainToMotor(value, ratio), ratio), value),
                     "增益双向换算不对称") &&
           TestCheck(TestNear(MotorTransTorqueToOutput(
                                  MotorTransTorqueToMotor(value, ratio), ratio), value),
                     "力矩双向换算不对称");
}

int main(void)
{
    if (!TestDirectDriveUnchanged() || !TestIdealMapping() || !TestRoundTrip())
    {
        return 1;
    }

    (void)puts("PASS: motor transmission regression");
    return 0;
}
