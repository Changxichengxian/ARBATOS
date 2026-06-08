/*
 * SPDX-FileCopyrightText: 2026 陈轩 <2811158416@qq.com>
 * SPDX-License-Identifier: Apache-2.0
 *
 * First published in this repository: 2026-04-06
 * Use of this file is governed by the LICENSE file in the repository root.
 */

#include "config.h"
#include "robot_task_build_config.h"

#if ROBOT_TASK_BUILD_SERVO

#include "servo_control_task.h"
#include "cmsis_os.h"
#include "bsp_servo_pwm.h"
#include "manual_input.h"
#include "robot_safety.h"

#define SERVO_MIN_PWM   500
#define SERVO_MAX_PWM   2500

#define PWM_DETAL_VALUE 10

#define SERVO1_ADD_PWM_KEY  KEY_PRESSED_OFFSET_Z
#define SERVO2_ADD_PWM_KEY  KEY_PRESSED_OFFSET_X
#define SERVO3_ADD_PWM_KEY  KEY_PRESSED_OFFSET_C
#define SERVO4_ADD_PWM_KEY  KEY_PRESSED_OFFSET_V

#define SERVO_MINUS_PWM_KEY KEY_PRESSED_OFFSET_SHIFT

const static uint16_t servo_key[4] = {SERVO1_ADD_PWM_KEY, SERVO2_ADD_PWM_KEY, SERVO3_ADD_PWM_KEY, SERVO4_ADD_PWM_KEY};
uint16_t servo_pwm[4] = {SERVO_MIN_PWM, SERVO_MIN_PWM, SERVO_MIN_PWM, SERVO_MIN_PWM};
/**
  * @brief          servo control task
  * @param[in]      pvParameters: NULL
  * @retval         none
  */
/**
  * @brief          舵机任务
  * @param[in]      pvParameters: NULL
  * @retval         none
  */
void servo_control_task(void const * argument)
{
    (void)argument;

    while(1)
    {
        manual_input_state_t servo_rc = {0};
        const uint8_t output_locked = robot_safety_output_locked();

        (void)manual_input_get_current_copy(&servo_rc);
        for(uint8_t i = 0; i < 4; i++)
        {
            if(output_locked != 0u)
            {
                servo_pwm_set(0u, i);
                continue;
            }

            if( (servo_rc.key.v & SERVO_MINUS_PWM_KEY) && (servo_rc.key.v & servo_key[i]))
            {
                servo_pwm[i] -= PWM_DETAL_VALUE;
            }
            else if(servo_rc.key.v & servo_key[i])
            {
                servo_pwm[i] += PWM_DETAL_VALUE;
            }

            //limit the pwm
           //限制pwm
            if(servo_pwm[i] < SERVO_MIN_PWM)
            {
                servo_pwm[i] = SERVO_MIN_PWM;
            }
            else if(servo_pwm[i] > SERVO_MAX_PWM)
            {
                servo_pwm[i] = SERVO_MAX_PWM;
            }

            servo_pwm_set(servo_pwm[i], i);
        }
        osDelay(10);
    }
}

#endif
