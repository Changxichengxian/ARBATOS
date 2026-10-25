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
#include "BspTime.h"
#include "ControlMgr.h"
#include "LowCmd.h"
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
static const MotorId ServoOutputId[LOWCMD_SERVO_COUNT] = {Motor18, Motor19, Motor20, Motor21};
static MotorCmd ServoCommands[LOWCMD_SERVO_COUNT];
static MotorCmd ServoSnapshots[LOWCMD_SERVO_COUNT];
static ControlOutputStamp ServoOwners[LOWCMD_SERVO_COUNT];
static ServoConfig ServoBoundConfig;
static uint8_t ServoBoundConfigValid;
static uint8_t ServoBoundPortMask;
uint16_t ServoPwm[4];
fp32 ServoPosition[4];
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
    ServoBoundConfig = g_config.servo;
    ServoBoundConfigValid = ServoConfigValid(&ServoBoundConfig);
    ServoBoundPortMask = 0u;
    for (uint8_t i = 0u; i < LOWCMD_SERVO_COUNT; i++) {
        ServoPwm[i] = 0u;
        ServoPosition[i] = 0.0f;
        if (ServoBoundConfigValid != 0u && ServoBoundConfig.channels[i].enabled != 0u) {
            ServoBoundPortMask |= (uint8_t)(1u << ServoBoundConfig.channels[i].port);
        }
    }

    while(1)
    {
        ManualInputSnapshot manualInput;
        ControlCtx controlCtx = {0};
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
        const uint32_t servoMask = (uint32_t)0x0Fu << (uint32_t)LOWCMD_SERVO_FIRST;
        const uint8_t managerAllowed =
            (uint8_t)(ControlMgrUpdateDomain(ControlDomainSystem, &controlCtx) == ControlResultOk &&
                      ControlMgrOutputPermitValid(&controlCtx.outputPermit, servoMask) != 0u);
        uint8_t published = 0u;

        for (uint8_t i = 0u; i < LOWCMD_SERVO_COUNT; i++) {
            ServoCommands[i] = (MotorCmd){0};
            ServoCommands[i].active = 1u;
            ServoCommands[i].mode = (uint8_t)MotorModeDisable;
            ServoCommands[i].timeoutMs = 30u;
        }

        if (ServoBoundConfig.configured == 1u) {
            const uint8_t allowed = (uint8_t)(servoOutputAllowed != 0u && modeAllowed != 0u &&
                RobotLifecycleOutputAllowed() != 0u && ServoBoundConfigValid != 0u &&
                managerAllowed != 0u);

            for (uint8_t i = 0u; i < 4u; i++) {
                const ServoChannelConfig *channel = &ServoBoundConfig.channels[i];
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
                    ServoPosition[i] = ServoPositionFromAxis(axis);
                } else {
                    const int8_t direction = (inputKeys & ServoKey[i]) == 0u ? 0 :
                        ((inputKeys & SERVO_MINUS_PWM_KEY) != 0u ? -1 : 1);
                    ServoPosition[i] = ServoPositionStep(channel, ServoPosition[i], direction);
                }
                ServoCommands[i].mode = (uint8_t)MotorModePosVel;
                ServoCommands[i].q = ServoPosition[i];
            }
        }

        if (managerAllowed != 0u) {
            published = LowCmdSetMotorManyWithPermit(ServoOutputId,
                                                      ServoCommands,
                                                      LOWCMD_SERVO_COUNT,
                                                      &controlCtx.outputPermit);
        }
        if (published != 0u) {
            published = LowCmdGetMotorManyStamped(ServoOutputId,
                                                   ServoSnapshots,
                                                   ServoOwners,
                                                   LOWCMD_SERVO_COUNT);
        }
        if (published != 0u && ServoBoundConfigValid != 0u) {
            const uint32_t nowMs = BspTimeGetTickMs();

            for (uint8_t i = 0u; i < LOWCMD_SERVO_COUNT; i++) {
                const ServoChannelConfig *channel = &ServoBoundConfig.channels[i];

                if (channel->enabled == 0u) continue;
                ServoPwm[i] = ServoPwmApply(channel,
                                            ServoOutputId[i],
                                            &ServoSnapshots[i],
                                            &ServoOwners[i],
                                            nowMs);
            }
            for (uint8_t port = 0u; port < 4u; port++) {
                if ((ServoBoundPortMask & (uint8_t)(1u << port)) == 0u) ServoPwmDisable(port);
            }
        } else {
            for (uint8_t i = 0u; i < LOWCMD_SERVO_COUNT; i++) ServoPwm[i] = 0u;
            for (uint8_t port = 0u; port < 4u; port++) ServoPwmDisable(port);
        }
        osDelay(10);
    }
}

#endif
