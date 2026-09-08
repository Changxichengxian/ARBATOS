/* SPDX-License-Identifier: Apache-2.0 */
#include <stdint.h>
#include "ArbatosDt.h"
#include "RobotFaultZephyr.h"
#include "RobotLifecycle.h"
#include "RobotMode.h"
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
void ServoPwmSet(uint16_t pwm, uint8_t i)
{
    /* 参数为微秒，0 关闭；静态准备版禁止启动任何舵机输出。 */
    if (i >= ARRAY_SIZE(Servo) || pwm > 2500u || k_is_in_isr()) {
        ServoPwmLastError = -EINVAL;
        return;
    }
    if (!pwm_is_ready_dt(&Servo[i])) {
        ServoPwmLastError = -ENODEV;
        return;
    }
    // STM32 PWM 写寄存器不等待；许可检查与写入共用中断锁，不能在锁定后提交旧脉宽。
    const unsigned int key = irq_lock();
    const uint8_t allowed = (uint8_t)(!IS_ENABLED(CONFIG_ARBATOS_PREFLIGHT_ONLY) &&
        RobotFaultZephyrBootLocked() == 0u && RobotLifecycleOutputAllowed() != 0u &&
        (robot_mode_current() == ROBOT_RUN_MODE_FULL || robot_mode_is_single_task(ROBOT_TASK_MODULE_SERVO) != 0u));
    ServoPwmLastError = pwm_set_dt(&Servo[i], Servo[i].period, PWM_USEC(allowed != 0u ? pwm : 0u));
    irq_unlock(key);
}
#else
void ServoPwmSet(uint16_t pwm, uint8_t i) { (void)pwm; (void)i; }
#endif
