/* SPDX-License-Identifier: Apache-2.0 */
#include <stdint.h>
#include "ArbatosDt.h"
#include "RobotFaultZephyr.h"
#include "RobotLifecycle.h"
#include "RobotMode.h"
#include "RobotSafety.h"
#include "ServoOutputPolicy.h"
#include <zephyr/drivers/pwm.h>
#include <zephyr/kernel.h>

#if DT_NODE_HAS_PROP(DT_PATH(arbatos_platform), servo_pwms)
static const struct pwm_dt_spec Servo[] = {
    ARBATOS_PWM_DT_SPEC_GET_BY_IDX(DT_PATH(arbatos_platform), servo_pwms, 0),
    ARBATOS_PWM_DT_SPEC_GET_BY_IDX(DT_PATH(arbatos_platform), servo_pwms, 1),
    ARBATOS_PWM_DT_SPEC_GET_BY_IDX(DT_PATH(arbatos_platform), servo_pwms, 2),
    ARBATOS_PWM_DT_SPEC_GET_BY_IDX(DT_PATH(arbatos_platform), servo_pwms, 3),
};
volatile int ServoPwmLastError;
static int ServoPwmSetRaw(uint16_t pwm, uint8_t i)
{
    /* 参数为微秒，0 关闭。 */
    if (i >= ARRAY_SIZE(Servo) || pwm > 2500u || k_is_in_isr()) {
        ServoPwmLastError = -EINVAL;
        return ServoPwmLastError;
    }
    if (!pwm_is_ready_dt(&Servo[i])) {
        ServoPwmLastError = -ENODEV;
        return ServoPwmLastError;
    }
    ServoPwmLastError = pwm_set_dt(&Servo[i], Servo[i].period, PWM_USEC(pwm));
    return ServoPwmLastError;
}

uint16_t ServoPwmApply(const ServoChannelConfig *channel,
                       MotorId id,
                       const MotorCmd *cmd,
                       const ControlOutputStamp *owner,
                       uint32_t nowMs)
{
    uint16_t pulse = 0u;

    if (ServoChannelValid(channel) == 0u || k_is_in_isr()) return 0u;
    // 最终许可检查与寄存器更新共用中断锁，快照后的停机或禁写不能穿过去。
    const unsigned int key = irq_lock();
    if (!IS_ENABLED(CONFIG_ARBATOS_PREFLIGHT_ONLY) &&
        RobotFaultZephyrBootLocked() == 0u &&
        RobotSafetyOutputLocked() == 0u &&
        RobotLifecycleOutputAllowed() != 0u &&
        (robot_mode_current() == ROBOT_RUN_MODE_FULL ||
         robot_mode_is_single_task(ROBOT_TASK_MODULE_SERVO) != 0u))
    {
        (void)ServoCommandPulseBound(channel, id, cmd, owner, nowMs, 0u, &pulse);
    }
    if (ServoPwmSetRaw(pulse, channel->port) != 0) pulse = 0u;
    irq_unlock(key);
    return pulse;
}

void ServoPwmDisable(uint8_t port)
{
    if (k_is_in_isr()) return;
    const unsigned int key = irq_lock();
    (void)ServoPwmSetRaw(0u, port);
    irq_unlock(key);
}
#else
uint16_t ServoPwmApply(const ServoChannelConfig *channel,
                       MotorId id,
                       const MotorCmd *cmd,
                       const ControlOutputStamp *owner,
                       uint32_t nowMs)
{
    (void)channel; (void)id; (void)cmd; (void)owner; (void)nowMs;
    return 0u;
}
void ServoPwmDisable(uint8_t port) { (void)port; }
#endif
