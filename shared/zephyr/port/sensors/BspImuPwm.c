/* SPDX-License-Identifier: Apache-2.0 */
#include "BspImuPwm.h"
#include "SensorsDt.h"
#include "ArbatosDt.h"

#include <errno.h>
#include <zephyr/drivers/pwm.h>

#if !DT_NODE_HAS_PROP(ARBATOS_SENSORS_NODE, imu_heater_pwms) || \
    !DT_NODE_HAS_PROP(ARBATOS_SENSORS_NODE, imu_heater_period_cycles)
#error "IMU 加热器需要 imu-heater-pwms 和 imu-heater-period-cycles。"
#endif

static const struct pwm_dt_spec ImuHeater =
    ARBATOS_PWM_DT_SPEC_GET_BY_IDX(ARBATOS_SENSORS_NODE, imu_heater_pwms, 0);
static const uint32_t ImuHeaterPeriod = DT_PROP(ARBATOS_SENSORS_NODE, imu_heater_period_cycles);
static int ImuPwmLastError = -ENODEV;

void imu_pwm_set(uint16_t pwm)
{
    uint32_t pulse;

    if (!pwm_is_ready_dt(&ImuHeater)) {
        ImuPwmLastError = -ENODEV;
        return;
    }
    if (pwm > ImuHeaterPeriod) {
        pwm = (uint16_t)ImuHeaterPeriod;
    }
    /* 旧 __HAL_TIM_SetCompare 直接接收 CCR 原始值；不能按配置上限重新归一化。 */
    pulse = pwm;
    ImuPwmLastError = pwm_set_cycles(ImuHeater.dev, ImuHeater.channel, ImuHeaterPeriod, pulse, ImuHeater.flags);
}

int imu_pwm_last_error(void) { return ImuPwmLastError; }
