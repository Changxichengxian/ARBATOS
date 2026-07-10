/* Unitree 命令策略主机回归：默认超时、阻尼换算和缓存失效边界。 */

#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "UnitreeMotorPolicy.h"

static int TestCheck(int condition, const char *message)
{
    if (condition != 0)
    {
        return 1;
    }

    (void)fprintf(stderr, "FAIL: %s\n", message);
    return 0;
}

static int TestTimeoutDefault(void)
{
    return TestCheck(UnitreeMotorRxTimeoutMs(0u) == UNITREE_MOTOR_DEFAULT_RX_TIMEOUT_MS,
                     "rx_timeout_ms=0 未使用驱动默认值") &&
           TestCheck(UnitreeMotorRxTimeoutMs(73u) == 73u,
                     "显式 Unitree 超时被错误改写");
}

static int TestDampingMapping(void)
{
    MotorCmd src;
    UnitreeMotorCmd out;
    UnitreeMotorCmd applied;

    (void)memset(&src, 0, sizeof(src));
    src.active = 1u;
    src.mode = (uint8_t)MotorModeDamping;
    src.dq = 7.0f; /* 故意注入残留速度，阻尼模式必须忽略。 */
    src.kd = 12.0f;
    src.tau = 6.0f;

    if (!TestCheck(UnitreeMotorMapLowCmd(&src, 2.0f, &out) != 0u,
                     "Damping 命令未被 Unitree 策略接受") ||
        !TestCheck(out.speed_rad_s == 0.0f,
                     "Damping 实际目标速度必须为零") ||
        !TestCheck(out.kd == 3.0f,
                     "Damping kd 未按减速比平方换算") ||
        !TestCheck(out.torque_nm == 3.0f,
                   "Damping 前馈力矩未按减速比换算"))
    {
        return 0;
    }

    UnitreeMotorMapAppliedOutput(&out, 2.0f, &applied);
    return TestCheck(applied.speed_rad_s == 0.0f &&
                         applied.kd == 12.0f && applied.torque_nm == 6.0f,
                     "MotorApplied 未恢复为 LowCmd 输出侧单位") &&
           TestCheck(UnitreeMotorBrakeRequired(MotorModeDisable) != 0u &&
                         UnitreeMotorBrakeRequired(MotorModeDamping) == 0u,
                     "Disable 应发 BRAKE，Damping 应保留 FOC 阻尼");
}

static int TestOutputSideMapping(void)
{
    MotorCmd src;
    UnitreeMotorCmd out;

    (void)memset(&src, 0, sizeof(src));
    src.active = 1u;
    src.mode = (uint8_t)MotorModeStateTorque;
    src.q = 1.0f;
    src.dq = 2.0f;
    src.kp = 20.0f;
    src.kd = 8.0f;
    src.tau = 10.0f;

    return TestCheck(UnitreeMotorMapLowCmd(&src, 2.0f, &out) != 0u,
                     "状态力矩命令未被 Unitree 策略接受") &&
           TestCheck(out.position_rad == 2.0f && out.speed_rad_s == 4.0f,
                     "位置或速度未从输出侧换到转子侧") &&
           TestCheck(out.kp == 5.0f && out.kd == 2.0f && out.torque_nm == 5.0f,
                     "刚度、阻尼或力矩减速比换算错误");
}

static int TestLatestCommandBoundary(void)
{
    MotorCmd cached;
    MotorCmd latest;

    (void)memset(&cached, 0, sizeof(cached));
    cached.active = 1u;
    cached.mode = (uint8_t)MotorModeCurrent;
    cached.writer = (uint16_t)LOWCMD_WRITER_CONTROL;
    cached.seq = 10u;
    cached.current = 16000;
    latest = cached;

    if (!TestCheck(UnitreeMotorSafeCurrent(&cached,
                                           &latest,
                                           (uint16_t)LOWCMD_WRITER_NONE) == 16000,
                   "一致且未禁写的最新电流不应被清零"))
    {
        return 0;
    }

    latest.active = 0u;
    if (!TestCheck(UnitreeMotorSafeCurrent(&cached,
                                           &latest,
                                           (uint16_t)LOWCMD_WRITER_SAFETY) == 0,
                   "缓存非零电流在最新命令 inactive 后仍被放行"))
    {
        return 0;
    }

    latest = cached;
    if (!TestCheck(UnitreeMotorSafeCurrent(&cached,
                                           &latest,
                                           (uint16_t)LOWCMD_WRITER_SAFETY) == 0,
                   "CONTROL 缓存电流越过 SAFETY 禁写"))
    {
        return 0;
    }

    latest.writer = (uint16_t)LOWCMD_WRITER_SAFETY;
    latest.seq++;
    if (!TestCheck(UnitreeMotorSafeCurrent(&cached,
                                           &latest,
                                           (uint16_t)LOWCMD_WRITER_SAFETY) == 0,
                   "缓存命令在 seq 变化后仍被物理发送"))
    {
        return 0;
    }

    cached.writer = (uint16_t)LOWCMD_WRITER_NONE;
    latest = cached;
    return TestCheck(UnitreeMotorSafeCurrent(&cached,
                                             &latest,
                                             (uint16_t)LOWCMD_WRITER_MANUAL) == 0,
                     "writer NONE 未按 CONTROL 归一，越过更高优先级禁写");
}

static int TestTxSchedule(void)
{
    UnitreeMotorTxSchedule schedule;
    MotorCmd cmd;
    MotorCmd latest;
    MotorCmd brake;

    (void)memset(&schedule, 0, sizeof(schedule));
    (void)memset(&cmd, 0, sizeof(cmd));
    cmd.active = 1u;
    cmd.mode = (uint8_t)MotorModeSpeed;
    cmd.seq = 20u;

    if (!TestCheck(UnitreeMotorTxDue(&schedule, &cmd, 100u, 5u) != 0u,
                   "首条 Unitree 命令应立即发送"))
    {
        return 0;
    }
    UnitreeMotorTxMark(&schedule, &cmd, 100u);
    if (!TestCheck(UnitreeMotorTxDue(&schedule, &cmd, 104u, 5u) == 0u &&
                       UnitreeMotorTxDue(&schedule, &cmd, 105u, 5u) != 0u,
                   "稳定命令未按配置周期节流"))
    {
        return 0;
    }

    cmd.seq++;
    if (!TestCheck(UnitreeMotorTxDue(&schedule, &cmd, 101u, 5u) != 0u,
                   "命令 seq 变化被错误延迟"))
    {
        return 0;
    }

    cmd.seq = schedule.last_seq;
    cmd.mode = (uint8_t)MotorModeDisable;
    if (!TestCheck(UnitreeMotorTxDue(&schedule, &cmd, 101u, 5u) != 0u,
                   "Disable/BRAKE 模式变化必须立即发送"))
    {
        return 0;
    }

    cmd.mode = (uint8_t)MotorModeSpeed;
    latest = cmd;
    latest.active = 0u;
    (void)memset(&brake, 0, sizeof(brake));
    brake.active = 1u;
    brake.mode = (uint8_t)MotorModeDisable;
    if (UnitreeMotorCmdSnapshotAllowed(&cmd,
                                       &latest,
                                       (uint16_t)LOWCMD_WRITER_SAFETY) == 0u)
    {
        cmd = brake;
    }
    return TestCheck(UnitreeMotorTxDue(&schedule, &cmd, 101u, 5u) != 0u,
                     "稳定速度未到期时，最新 inactive/SAFETY 禁写仍应立即 BRAKE");
}

int main(void)
{
    if (!TestTimeoutDefault() ||
        !TestDampingMapping() ||
        !TestOutputSideMapping() ||
        !TestLatestCommandBoundary() ||
        !TestTxSchedule())
    {
        return 1;
    }

    (void)puts("PASS: Unitree motor policy host regression");
    return 0;
}
