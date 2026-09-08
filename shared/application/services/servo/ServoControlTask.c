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
#include "ServoOutputPolicy.h"
#include "RobotLifecycle.h"
#include "RobotMode.h"

#define SERVO1_ADD_PWM_KEY  KEY_PRESSED_OFFSET_Z
#define SERVO2_ADD_PWM_KEY  KEY_PRESSED_OFFSET_X
#define SERVO3_ADD_PWM_KEY  KEY_PRESSED_OFFSET_C
#define SERVO4_ADD_PWM_KEY  KEY_PRESSED_OFFSET_V

#define SERVO_MINUS_PWM_KEY KEY_PRESSED_OFFSET_SHIFT
#define SERVO_ACTION_KEY_MASK \
    (SERVO1_ADD_PWM_KEY | SERVO2_ADD_PWM_KEY | SERVO3_ADD_PWM_KEY | SERVO4_ADD_PWM_KEY)

static const uint16_t ServoKey[4] = {SERVO1_ADD_PWM_KEY, SERVO2_ADD_PWM_KEY, SERVO3_ADD_PWM_KEY, SERVO4_ADD_PWM_KEY};
uint16_t ServoPwm[4];
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
    uint8_t axisReady[4] = {0u};

    (void)argument;
    ServoInputGateInit(&inputGate);
    if (g_config.servo.configured == 1u) {
        for (uint8_t i = 0u; i < 4u; i++) {
            ServoPwm[i] = g_config.servo.channels[i].centerUs;
        }
    }

    while(1)
    {
        ManualInputSnapshot manualInput;
        const uint8_t inputValid = ManualInputSnapshotRead(&manualInput);
        const uint8_t output_locked = RobotSafetyOutputLocked();
        const uint8_t modeAllowed = (uint8_t)(robot_mode_current() == ROBOT_RUN_MODE_FULL ||
            robot_mode_is_single_task(ROBOT_TASK_MODULE_SERVO) != 0u);
        const uint8_t controlAllowed =
            (uint8_t)(inputValid != 0u &&
                      manualInput.online != 0u &&
                      manualInput.dataValid != 0u &&
                      modeAllowed != 0u &&
                      output_locked == 0u);
        const uint16_t observedKeys =
            (inputValid != 0u) ? manualInput.manual.key.v : 0u;
        ServoInputGateSync(&inputGate,
                           (inputValid != 0u) ? manualInput.authoritySeq : 0u,
                           (inputValid != 0u) ? manualInput.semanticsSeq : 0u);
        if (inputGate.waitRelease != 0u || controlAllowed == 0u) {
            for (uint8_t i = 0u; i < 4u; i++) axisReady[i] = 0u;
        }
        const uint16_t inputKeys =
            ServoInputGateApply(&inputGate,
                                controlAllowed,
                                SERVO_ACTION_KEY_MASK,
                                observedKeys);
        const uint8_t servoOutputAllowed =
            (uint8_t)(controlAllowed != 0u && ServoInputGateReady(&inputGate) != 0u);

        if (g_config.servo.configured == 1u) {
            uint16_t pulses[4] = {0u};
            const uint8_t allowed = (uint8_t)(servoOutputAllowed != 0u && modeAllowed != 0u &&
                RobotLifecycleOutputAllowed() != 0u && ServoConfigValid(&g_config.servo) != 0u);

            for (uint8_t i = 0u; i < 4u; i++) {
                const ServoChannelConfig *channel = &g_config.servo.channels[i];
                if (allowed == 0u || channel->enabled == 0u) {
                    axisReady[i] = 0u;
                    continue;
                }
                if (channel->inputMode == 1u) {
                    const int16_t axis = manualInput.manual.rc.ch[channel->inputChannel];
                    // 接收机切换或掉线恢复后，先回中再允许舵机跟随，避免突然摆动。
                    if (axisReady[i] == 0u) {
                        if (axis >= -20 && axis <= 20) axisReady[i] = 1u;
                        continue;
                    }
                    ServoPwm[i] = ServoPulseFromAxis(channel, axis);
                } else {
                    const int8_t direction = (inputKeys & ServoKey[i]) == 0u ? 0 :
                        ((inputKeys & SERVO_MINUS_PWM_KEY) != 0u ? -1 : 1);
                    ServoPwm[i] = ServoPulseStep(channel, ServoPwm[i], direction);
                }
                pulses[channel->port] = ServoPwm[i];
            }
            for (uint8_t port = 0u; port < 4u; port++) ServoPwmSet(pulses[port], port);
            osDelay(10);
            continue;
        }

        // 只启用任务但尚未填写舵机表时，四路保持关闭，与客户端默认值一致。
        for (uint8_t i = 0u; i < 4u; i++) ServoPwmSet(0u, i);
        osDelay(10);
    }
}

#endif
