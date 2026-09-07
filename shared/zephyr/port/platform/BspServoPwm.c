/* SPDX-License-Identifier: Apache-2.0 */
#include <stdint.h>
#include "ArbatosDt.h"
#include "RobotFaultZephyr.h"
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
    if ((IS_ENABLED(CONFIG_ARBATOS_PREFLIGHT_ONLY) || RobotFaultZephyrBootLocked() != 0u) && pwm != 0u) {
        ServoPwmLastError = -EPERM;
        return;
    }
    ServoPwmLastError = pwm_is_ready_dt(&Servo[i]) ?
        pwm_set_dt(&Servo[i], Servo[i].period, PWM_USEC(pwm)) : -ENODEV;
}
#else
void ServoPwmSet(uint16_t pwm, uint8_t i) { (void)pwm; (void)i; }
#endif
