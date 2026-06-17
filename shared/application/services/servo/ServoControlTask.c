/*
 * SPDX-FileCopyrightText: 2026 陈轩 <2811158416@qq.com>
 * SPDX-License-Identifier: Apache-2.0
 *
 * First published in this repository: 2026-04-06
 * Use of this file is governed by the LICENSE file in the repository root.
 */

#include "config.h"
#include "RobotTaskBuildConfig.h"

#if ROBOT_TASK_BUILD_SERVO

#include "ServoControlTask.h"
#include "cmsis_os.h"
#include "BspServoPwm.h"
#include "ManualInput.h"
#include "RobotSafety.h"

#define SERVO_MIN_PWM   500
#define SERVO_MAX_PWM   2500

#define PWM_DELTA_VALUE 10

#define SERVO1_ADD_PWM_KEY  KEY_PRESSED_OFFSET_Z
#define SERVO2_ADD_PWM_KEY  KEY_PRESSED_OFFSET_X
#define SERVO3_ADD_PWM_KEY  KEY_PRESSED_OFFSET_C
#define SERVO4_ADD_PWM_KEY  KEY_PRESSED_OFFSET_V

#define SERVO_MINUS_PWM_KEY KEY_PRESSED_OFFSET_SHIFT

static const uint16_t ServoKey[4] = {SERVO1_ADD_PWM_KEY, SERVO2_ADD_PWM_KEY, SERVO3_ADD_PWM_KEY, SERVO4_ADD_PWM_KEY};
uint16_t ServoPwm[4] = {SERVO_MIN_PWM, SERVO_MIN_PWM, SERVO_MIN_PWM, SERVO_MIN_PWM};
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
void ServoControlTask(void const * argument)
{
    (void)argument;

    while(1)
    {
        ManualInputState ServoRc = {0};
        const uint8_t output_locked = RobotSafetyOutputLocked();

        (void)ManualInputGetCurrentCopy(&ServoRc);
        for(uint8_t i = 0; i < 4; i++)
        {
            if(output_locked != 0u)
            {
                ServoPwmSet(0u, i);
                continue;
            }

            if( (ServoRc.key.v & SERVO_MINUS_PWM_KEY) && (ServoRc.key.v & ServoKey[i]))
            {
                ServoPwm[i] -= PWM_DELTA_VALUE;
            }
            else if(ServoRc.key.v & ServoKey[i])
            {
                ServoPwm[i] += PWM_DELTA_VALUE;
            }

            //limit the pwm
           //限制pwm
            if(ServoPwm[i] < SERVO_MIN_PWM)
            {
                ServoPwm[i] = SERVO_MIN_PWM;
            }
            else if(ServoPwm[i] > SERVO_MAX_PWM)
            {
                ServoPwm[i] = SERVO_MAX_PWM;
            }

            ServoPwmSet(ServoPwm[i], i);
        }
        osDelay(10);
    }
}

#endif
