/* SPDX-License-Identifier: Apache-2.0 */
#ifndef BSP_IMU_PWM_H
#define BSP_IMU_PWM_H

#include <stdint.h>
void imu_pwm_set(uint16_t pwm);
int imu_pwm_last_error(void);

#endif
