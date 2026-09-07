#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

#include "DifferentialChassis.h"
#include "SpeedGimbal.h"

/* 算法不依赖板子、操作系统、总线或全局 RobotConfig；这里直接运行同一份 C。 */
typedef union
{
    long double alignment;
    unsigned char bytes[192];
} TestState;

static void TestDifferential(void)
{
    TestState state = {0};
    ControlAlgorithmInput input = {0};
    ControlAlgorithmOutput output = {0};
    const ControlParam params[] = {{"max_current", 1000.0f}};
    assert(DifferentialChassis.stateBytes <= sizeof(state.bytes));
    assert(DifferentialChassis.configure(state.bytes, params, 1) == ControlAlgorithmResultOk);
    input.outputCount = 2;
    input.motor[0].online = input.motor[1].online = 1;
    input.dt = 0.002f;
    input.axis[ControlAxisChassisX] = 1.0f;
    assert(DifferentialChassis.step(state.bytes, &input, &output) == ControlAlgorithmResultOk);
    assert(output.outputCount == 2 && output.motor[0].current == 1000 && output.motor[1].current == 1000);
    input.axis[ControlAxisChassisX] = 0.0f;
    input.axis[ControlAxisChassisWz] = 1.0f;
    assert(DifferentialChassis.step(state.bytes, &input, &output) == ControlAlgorithmResultOk);
    assert(output.motor[0].current < 0 && output.motor[1].current > 0);
    input.axis[ControlAxisChassisWz] = 0.0f;
    input.motor[0].speedRpm = input.motor[1].speedRpm = 20;
    assert(DifferentialChassis.step(state.bytes, &input, &output) == ControlAlgorithmResultOk);
    assert(output.motor[0].current < 0 && output.motor[1].current < 0);
    input.outputCount = 1;
    assert(DifferentialChassis.step(state.bytes, &input, &output) == ControlAlgorithmResultBadInput);
}

static void TestGimbal(void)
{
    TestState state = {0};
    ControlAlgorithmInput input = {0};
    ControlAlgorithmOutput output = {0};
    const ControlParam params[] = {{"max_current", 100.0f}};
    assert(SpeedGimbal.stateBytes <= sizeof(state.bytes));
    assert(SpeedGimbal.configure(state.bytes, params, 1) == ControlAlgorithmResultOk);
    input.outputCount = 2;
    input.dt = 0.002f;
    input.axis[ControlAxisGimbalYaw] = 1.0f;
    input.axis[ControlAxisGimbalPitch] = -1.0f;
    assert(SpeedGimbal.step(state.bytes, &input, &output) == ControlAlgorithmResultOk);
    assert(output.motor[0].current == 100 && output.motor[1].current == -100);
    input.axis[ControlAxisGimbalYaw] = input.axis[ControlAxisGimbalPitch] = 0;
    input.motor[0].speedRpm = 5;
    input.motor[1].speedRpm = -5;
    assert(SpeedGimbal.step(state.bytes, &input, &output) == ControlAlgorithmResultOk);
    assert(output.motor[0].current == -50 && output.motor[1].current == 50);
    input.axis[ControlAxisGimbalYaw] = NAN;
    assert(SpeedGimbal.step(state.bytes, &input, &output) == ControlAlgorithmResultBadInput);
}

int main(void)
{
    TestDifferential();
    TestGimbal();
    puts("Controller algorithm regressions passed");
    return 0;
}
