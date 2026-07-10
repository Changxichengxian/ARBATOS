/*
 * SPDX-FileCopyrightText: 2026 陈轩 <2811158416@qq.com>
 * SPDX-License-Identifier: Apache-2.0
 *
 * First published in this repository: 2026-04-06
 * Use of this file is governed by the LICENSE file in the repository root.
 */

#include "RobotConfig.h"
#include "RobotTaskBuildConfig.h"

#if ROBOT_TASK_BUILD_SERVO

#include "ServoControlTask.h"
#include "cmsis_os.h"
#include "BspServoPwm.h"
#include "ManualInputSnapshot.h"
#include "RobotSafety.h"
#include "ServoInputPolicy.h"

#define SERVO_MIN_PWM   500
#define SERVO_MAX_PWM   2500

#define PWM_DELTA_VALUE 10

#define SERVO1_ADD_PWM_KEY  KEY_PRESSED_OFFSET_Z
#define SERVO2_ADD_PWM_KEY  KEY_PRESSED_OFFSET_X
#define SERVO3_ADD_PWM_KEY  KEY_PRESSED_OFFSET_C
#define SERVO4_ADD_PWM_KEY  KEY_PRESSED_OFFSET_V

#define SERVO_MINUS_PWM_KEY KEY_PRESSED_OFFSET_SHIFT
#define SERVO_ACTION_KEY_MASK \
    (SERVO1_ADD_PWM_KEY | SERVO2_ADD_PWM_KEY | SERVO3_ADD_PWM_KEY | SERVO4_ADD_PWM_KEY)

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
    ServoInputGate inputGate;

    (void)argument;
    ServoInputGateInit(&inputGate);

    while(1)
    {
        ManualInputSnapshot manualInput;
        const uint8_t inputValid = ManualInputSnapshotRead(&manualInput);
        const uint8_t output_locked = RobotSafetyOutputLocked();
        const uint8_t controlAllowed =
            (uint8_t)(inputValid != 0u &&
                      manualInput.online != 0u &&
                      output_locked == 0u);
        const uint16_t observedKeys =
            (inputValid != 0u) ? manualInput.manual.key.v : 0u;
        ServoInputGateSync(&inputGate,
                           (inputValid != 0u) ? manualInput.authoritySeq : 0u,
                           (inputValid != 0u) ? manualInput.semanticsSeq : 0u);
        const uint16_t inputKeys =
            ServoInputGateApply(&inputGate,
                                controlAllowed,
                                SERVO_ACTION_KEY_MASK,
                                observedKeys);
        const uint8_t servoOutputAllowed =
            (uint8_t)(controlAllowed != 0u && ServoInputGateReady(&inputGate) != 0u);

        for(uint8_t i = 0; i < 4; i++)
        {
            int32_t requestedPwm = ServoPwm[i];

            if(servoOutputAllowed == 0u)
            {
                ServoPwmSet(0u, i);
                continue;
            }

            if( (inputKeys & SERVO_MINUS_PWM_KEY) && (inputKeys & ServoKey[i]))
            {
                requestedPwm -= PWM_DELTA_VALUE;
            }
            else if(inputKeys & ServoKey[i])
            {
                requestedPwm += PWM_DELTA_VALUE;
            }

            //limit the pwm
           //限制pwm
            if(requestedPwm < SERVO_MIN_PWM)
            {
                requestedPwm = SERVO_MIN_PWM;
            }
            else if(requestedPwm > SERVO_MAX_PWM)
            {
                requestedPwm = SERVO_MAX_PWM;
            }

            ServoPwm[i] = (uint16_t)requestedPwm;
            ServoPwmSet(ServoPwm[i], i);
        }
        osDelay(10);
    }
}

#endif
