/*
 * SPDX-FileCopyrightText: 2026 陈轩 <2811158416@qq.com>
 * SPDX-License-Identifier: Apache-2.0
 *
 * First published in this repository: 2026-04-06
 * Use of this file is governed by the LICENSE file in the repository root.
 */

/*
 * 阅读地图：
 * - 前段：拨杆/图传输入解释、娱乐模式蜂鸣器音乐、摩擦轮/拨弹输出清零。
 * - 中段：ShootRuntimeStep() 串起状态机、反馈更新、PID 电流输出。
 * - 后段：ShootSetMode() 决定射击状态，feedback_update() 维护编码器圈数和堵转信息。
 * - 输出：拨弹电流作为返回值，摩擦轮电流写入 LowCmd。
 */

#include "RobotConfig.h"
#include "RobotTaskBuildConfig.h"

#if ROBOT_TASK_BUILD_SHOOT_RM

#include "Shoot.h"
#include "ShootRuntime.h"

#include <math.h>
#include <string.h>
#include "cmsis_os.h"

#include "BspShootTrig.h"
#include "UserLib.h"
#include "Referee.h"

#include "CanReceive.h"
#include "LowCmd.h"
#include "FaultMgr.h"
#include "MotorHealth.h"
#include "MotorInst.h"
#include "MotorConfig.h"
#include "GimbalState.h"
#include "ShootState.h"
#include "ShootFaultPolicy.h"
#include "ShootInputPolicy.h"
#include "ManualInputSnapshot.h"
#include "ControlInput.h"
#include "DetectTask.h"
#include "RobotMode.h"
#include "Pid.h"
#include "BspTime.h"

// 微动开关 GPIO 是板相关差异，通过 BSP 读取。

typedef struct
{
    uint16_t rawSwitch;
    uint16_t effectiveSwitch;
    ManualInputSemanticsConfig semantics;
    uint32_t semanticsSeq;
    uint32_t actionSeq;
    uint8_t manualOnline;
    uint8_t mousePressL;
    uint8_t mousePressR;
} ShootFrameInput;

static ShootInputGateState s_shootInputGate;

static const ManualInputSemanticsConfig ShootDefaultSemantics = {
    .GimbalSafePos = MANUAL_INPUT_SWITCH_POS_UP,
    .ChassisSafePos = MANUAL_INPUT_SWITCH_POS_UP,
    .ChassisFollowPos = MANUAL_INPUT_SWITCH_POS_MID,
    .ChassisSpinPos = MANUAL_INPUT_SWITCH_POS_DOWN,
    .ShootStopPos = MANUAL_INPUT_SWITCH_POS_UP,
    .ShootReadyPos = MANUAL_INPUT_SWITCH_POS_MID,
    .ShootFirePos = MANUAL_INPUT_SWITCH_POS_DOWN,
    .image_vt13_shoot_switch_input = MANUAL_INPUT_IMAGE_SWITCH_CHASSIS,
};

/**
  * @brief          射击状态机设置，遥控器上拨一次开启，再上拨关闭，下拨1次发射1颗，一直处在下，则持续发射，用于3min准备时间清理子弹
  * @param[in]      void
  * @retval         void
  */
static void ShootSetMode(const ShootFrameInput *frame);
/**
  * @brief          update shoot feedback data.
  * @param[in]      void
  * @retval         void
  */
static void ShootFeedbackUpdate(const ShootFrameInput *frame);

/**
  * @brief          清空摩擦轮输出（速度环 PID / 目标转速 / 电流指令）
  * @param[in]      void
  * @retval         void
  */
static void ShootClearFricOutput(void);
static void ShootClearTriggerOutput(void);

/**
  * @brief          摩擦轮到速判定（基于电机反馈转速）
  * @param[in]      void
  * @retval         1: ready 0: not ready
  */
static bool_t ShootFricSpeedReady(void);
static bool_t ShootGimbalCmdToShootStop(void);
static void ShootWriteState(void);
static void ShootFaultInit(void);
static void ShootFaultUpdate(uint8_t switch_safe);
static void ShootFaultSyncInhibit(void);
static uint8_t ShootFaultStopsDomain(void);
static uint8_t ShootFaultStopsTrigger(void);
static uint8_t ShootFaultTriggerUsable(void);

static const char *const ShootFrictionMotorNames[FRIC_MOTOR_NUM] = {
    "motor.friction0",
    "motor.friction1",
    "motor.friction2",
    "motor.friction3",
};
static MotorCurrentBind ShootFrictionCurrentBindings[FRIC_MOTOR_NUM];
static const int16_t ShootFricZeroCurrentCmd[FRIC_MOTOR_NUM] = {0};

#define SHOOT_FAULT_DEVICE_COUNT (1u + FRIC_MOTOR_NUM)
#define SHOOT_FAULT_TRIGGER_DEVICE 0u
#define SHOOT_FAULT_DOMAIN_ID 0u
#define SHOOT_FAULT_RECOVERY_MS 200u
#define SHOOT_FAULT_REASON_MOTOR (1u << 0)

typedef struct
{
    FaultMgr mgr;
    MotorId motorId[SHOOT_FAULT_DEVICE_COUNT];
    uint16_t reason[SHOOT_FAULT_DEVICE_COUNT];
    uint8_t configuredMask;
    uint8_t activeMask;
    uint8_t blockingMask;
    uint8_t recoveryMask;
    uint8_t fricMask;
    uint8_t inhibitMask;
    uint8_t holdZeroMask;
    uint8_t domainAction;
    uint8_t triggerAction;
    uint8_t domainConfigured;
    uint8_t initialized;
    uint32_t inhibitFailCount;
    uint32_t releaseFailCount;
} ShootFaultRuntime;

static ShootFaultRuntime s_shootFault;

/**
  * @brief          trigger jam reverse handling.
  * @param[in]      void
  * @retval         void
  */
static void trigger_motor_turn_back(void);

/**
  * @brief          射击控制，控制拨弹电机角度，完成一次发射
  * @param[in]      void
  * @retval         void
  */
static void ShootBulletControl(void);

static uint16_t ShootTickMs(void)
{
    return (SHOOT_CONTROL_TIME > 0u) ? SHOOT_CONTROL_TIME : 1u;
}

static uint16_t ShootU16AddSat(uint16_t value, uint16_t add_ms, uint16_t max_value)
{
    const uint32_t next = (uint32_t)value + (uint32_t)add_ms;
    return (next >= (uint32_t)max_value) ? max_value : (uint16_t)next;
}

static uint16_t ShootSwitchRawFromPos(uint8_t pos)
{
    return (uint16_t)ControlInputSwitchPosToRaw(pos);
}

static const ManualInputSemanticsConfig *ShootSemanticsOrDefault(
    const ManualInputSemanticsConfig *semantics)
{
    return (semantics != NULL) ? semantics : &ShootDefaultSemantics;
}

static uint8_t ShootSwitchIsStop(uint16_t raw_sw,
                                 const ManualInputSemanticsConfig *semantics)
{
    semantics = ShootSemanticsOrDefault(semantics);
    return ControlInputSwitchIsPos(raw_sw, semantics->ShootStopPos);
}

static uint8_t ShootSwitchIsReady(uint16_t raw_sw,
                                  const ManualInputSemanticsConfig *semantics)
{
    semantics = ShootSemanticsOrDefault(semantics);
    return ControlInputSwitchIsPos(raw_sw, semantics->ShootReadyPos);
}

static uint8_t ShootSwitchIsFire(uint16_t raw_sw,
                                 const ManualInputSemanticsConfig *semantics)
{
    semantics = ShootSemanticsOrDefault(semantics);
    return ControlInputSwitchIsPos(raw_sw, semantics->ShootFirePos);
}

static uint32_t ShootFaultTimeoutMs(uint8_t deviceId)
{
    if (deviceId == SHOOT_FAULT_TRIGGER_DEVICE)
    {
        const uint16_t configured = g_config.detect.items[TRIGGER_MOTOR_TOE].offline_time_ms;

        /* 拨弹轴仍沿用原检测参数，但异常配置不能放大故障窗口。 */
        if (configured >= 10u && configured <= 100u)
        {
            return configured;
        }
        return 50u;
    }

    return 100u;
}

static const motor_node_param_t *ShootFaultNode(uint8_t deviceId)
{
    if (deviceId == SHOOT_FAULT_TRIGGER_DEVICE)
    {
        return &g_config.motor.trigger;
    }
    if (deviceId < SHOOT_FAULT_DEVICE_COUNT)
    {
        return &g_config.motor.friction[deviceId - 1u];
    }
    return NULL;
}

static void ShootFaultInit(void)
{
    static const char *const motorNames[SHOOT_FAULT_DEVICE_COUNT] = {
        "motor.trigger",
        "motor.friction0",
        "motor.friction1",
        "motor.friction2",
        "motor.friction3",
    };
    FaultMgrConfig config;
    uint32_t memberMask = 0u;
    uint32_t criticalMask = 0u;

    (void)memset(&s_shootFault, 0, sizeof(s_shootFault));
    (void)memset(&config, 0, sizeof(config));
    config.deviceCount = SHOOT_FAULT_DEVICE_COUNT;

    for (uint8_t i = 0u; i < SHOOT_FAULT_DEVICE_COUNT; i++)
    {
        const motor_node_param_t *node = ShootFaultNode(i);
        uint8_t configured = (MotorCfgNodeId(node) != 0u) ? 1u : 0u;

        if (i != SHOOT_FAULT_TRIGGER_DEVICE && SHOOT_FRIC_DIR(i - 1u) == 0)
        {
            configured = 0u;
        }

        s_shootFault.motorId[i] = MotorInstIdByName(motorNames[i]);
        if (configured == 0u)
        {
            continue;
        }

        s_shootFault.configuredMask |= (uint8_t)(1u << i);
        memberMask |= 1u << i;
        if (i == SHOOT_FAULT_TRIGGER_DEVICE)
        {
            config.device[i].stableMs = SHOOT_FAULT_RECOVERY_MS;
            config.device[i].requireSafeInput = 1u;
        }
        else
        {
            criticalMask |= 1u << i;
            s_shootFault.fricMask |= (uint8_t)(1u << i);
        }
    }

    if (criticalMask != 0u)
    {
        config.domainCount = 1u;
        config.domain[SHOOT_FAULT_DOMAIN_ID].memberMask = memberMask;
        config.domain[SHOOT_FAULT_DOMAIN_ID].criticalMask = criticalMask;
        config.domain[SHOOT_FAULT_DOMAIN_ID].recovery.stableMs = SHOOT_FAULT_RECOVERY_MS;
        config.domain[SHOOT_FAULT_DOMAIN_ID].recovery.requireSafeInput = 1u;
        s_shootFault.domainConfigured = 1u;
    }

    s_shootFault.initialized =
        (FaultMgrInit(&s_shootFault.mgr, &config) == FaultMgrResultOk) ? 1u : 0u;
    s_shootFault.domainAction = (s_shootFault.initialized != 0u) ?
                                    (uint8_t)FaultActionRun : (uint8_t)FaultActionStopGlobal;
    s_shootFault.triggerAction = s_shootFault.domainAction;
}

static void ShootFaultUpdate(uint8_t switch_safe)
{
    const uint32_t nowMs = BspTimeGetTickMs();
    uint32_t deviceSafeMask = 0u;
    uint32_t domainSafeMask = 0u;

    s_shootFault.activeMask = 0u;
    s_shootFault.blockingMask = 0u;
    s_shootFault.recoveryMask = 0u;

    if (s_shootFault.initialized == 0u)
    {
        s_shootFault.domainAction = (uint8_t)FaultActionStopGlobal;
        s_shootFault.triggerAction = (uint8_t)FaultActionStopGlobal;
        s_shootFault.blockingMask = s_shootFault.configuredMask;
        return;
    }

    for (uint8_t i = 0u; i < SHOOT_FAULT_DEVICE_COUNT; i++)
    {
        MotorHealthResult health;

        s_shootFault.reason[i] = 0u;
        if ((s_shootFault.configuredMask & (1u << i)) == 0u)
        {
            continue;
        }

        (void)MotorHealthRead(s_shootFault.motorId[i],
                              nowMs,
                              ShootFaultTimeoutMs(i),
                              &health);
        s_shootFault.reason[i] = health.reasonMask;
        if (health.healthy == 0u)
        {
            s_shootFault.activeMask |= (uint8_t)(1u << i);
        }
        (void)FaultMgrSetDeviceFault(&s_shootFault.mgr,
                                     i,
                                     SHOOT_FAULT_REASON_MOTOR,
                                     (health.healthy == 0u) ? 1u : 0u,
                                     nowMs);
    }

    if (switch_safe != 0u)
    {
        deviceSafeMask |= 1u << SHOOT_FAULT_TRIGGER_DEVICE;
        domainSafeMask |= 1u << SHOOT_FAULT_DOMAIN_ID;
    }
    (void)FaultMgrUpdate(&s_shootFault.mgr,
                         nowMs,
                         deviceSafeMask,
                         domainSafeMask,
                         switch_safe);

    s_shootFault.domainAction = (s_shootFault.domainConfigured != 0u) ?
        (uint8_t)FaultMgrDomainAction(&s_shootFault.mgr, SHOOT_FAULT_DOMAIN_ID) :
        (uint8_t)FaultActionRun;
    s_shootFault.triggerAction =
        ((s_shootFault.configuredMask & (1u << SHOOT_FAULT_TRIGGER_DEVICE)) != 0u) ?
            (uint8_t)FaultMgrDeviceAction(&s_shootFault.mgr, SHOOT_FAULT_TRIGGER_DEVICE) :
            (uint8_t)FaultActionRun;

    for (uint8_t i = 0u; i < SHOOT_FAULT_DEVICE_COUNT; i++)
    {
        FaultDeviceStatus status;

        if ((s_shootFault.configuredMask & (1u << i)) == 0u)
        {
            continue;
        }
        if (FaultMgrGetDeviceStatus(&s_shootFault.mgr, i, &status) != FaultMgrResultOk)
        {
            s_shootFault.blockingMask |= (uint8_t)(1u << i);
            continue;
        }
        if (status.action != FaultActionRun)
        {
            s_shootFault.blockingMask |= (uint8_t)(1u << i);
        }
        if (status.recoveryPending != 0u)
        {
            s_shootFault.recoveryMask |= (uint8_t)(1u << i);
        }
    }

    if (s_shootFault.domainConfigured != 0u)
    {
        FaultDomainStatus status;

        if (FaultMgrGetDomainStatus(&s_shootFault.mgr,
                                    SHOOT_FAULT_DOMAIN_ID,
                                    &status) == FaultMgrResultOk &&
            status.recoveryPending != 0u)
        {
            s_shootFault.recoveryMask |= (uint8_t)status.memberMask;
        }
    }
}

static uint8_t ShootFaultStopsDomain(void)
{
    return (s_shootFault.domainAction != (uint8_t)FaultActionRun ||
            (s_shootFault.holdZeroMask & s_shootFault.fricMask) != 0u) ? 1u : 0u;
}

static uint8_t ShootFaultStopsTrigger(void)
{
    return (s_shootFault.triggerAction != (uint8_t)FaultActionRun ||
            (s_shootFault.holdZeroMask & (1u << SHOOT_FAULT_TRIGGER_DEVICE)) != 0u) ? 1u : 0u;
}

static uint8_t ShootFaultTriggerUsable(void)
{
    const uint8_t configured =
        (uint8_t)((s_shootFault.configuredMask >> SHOOT_FAULT_TRIGGER_DEVICE) & 1u);
    return (configured != 0u && ShootFaultStopsTrigger() == 0u) ? 1u : 0u;
}

static input_switch_e ShootGetImageSwitchInput(
    const ManualInputSemanticsConfig *semantics)
{
    semantics = ShootSemanticsOrDefault(semantics);
    switch (semantics->image_vt13_shoot_switch_input)
    {
    case MANUAL_INPUT_IMAGE_SWITCH_CHASSIS:
        return INPUT_SW_CHASSIS_MODE;
    case MANUAL_INPUT_IMAGE_SWITCH_GIMBAL:
        return INPUT_SW_GIMBAL_MODE;
    case MANUAL_INPUT_IMAGE_SWITCH_SHOOT:
    default:
        return INPUT_SW_SHOOT_MODE;
    }
}

static uint16_t ShootGetRawSwitch(const ManualInputSnapshot *manualInput,
                                  uint8_t manualOnline,
                                  const ManualInputSemanticsConfig *semantics)
{
    uint16_t raw_sw;

    semantics = ShootSemanticsOrDefault(semantics);
    raw_sw = ShootSwitchRawFromPos(semantics->ShootStopPos);

    if (manualInput == NULL || manualOnline == 0u)
    {
        return raw_sw;
    }

    raw_sw = (uint16_t)manualInput->control.sw[INPUT_SW_SHOOT_MODE];

    if (manualInput->activeSource != MANUAL_INPUT_SRC_IMAGE)
    {
        return raw_sw;
    }

    if (manualInput->sourceProtocol != MANUAL_INPUT_PROTOCOL_IMAGE_VT13)
    {
        return raw_sw;
    }

    raw_sw = (uint16_t)manualInput->control.sw[ShootGetImageSwitchInput(semantics)];
    return ShootSwitchIsStop(raw_sw, semantics) ? ShootSwitchRawFromPos(semantics->ShootStopPos) :
                                                  ShootSwitchRawFromPos(semantics->ShootFirePos);
}

static uint16_t ShootGetEffectiveSwitch(uint16_t raw_sw,
                                        uint8_t manualOnline,
                                        const ManualInputSemanticsConfig *semantics)
{
    uint16_t ShootStopRaw;
    uint16_t ShootReadyRaw;
    uint16_t ShootFireRaw;

    semantics = ShootSemanticsOrDefault(semantics);
    ShootStopRaw = ShootSwitchRawFromPos(semantics->ShootStopPos);
    ShootReadyRaw = ShootSwitchRawFromPos(semantics->ShootReadyPos);
    ShootFireRaw = ShootSwitchRawFromPos(semantics->ShootFirePos);

    return ShootInputGateSwitch(&s_shootInputGate,
                                raw_sw,
                                manualOnline,
                                ShootStopRaw,
                                ShootReadyRaw,
                                ShootFireRaw);
}

static void ShootResetInputGate(const ManualInputSemanticsConfig *semantics)
{
    semantics = ShootSemanticsOrDefault(semantics);
    ShootInputGateReset(&s_shootInputGate,
                        ShootSwitchRawFromPos(semantics->ShootStopPos));
}

static void ShootMouseRearmSyncSafe(const ShootFrameInput *observed)
{
    const uint8_t manualOnline = (observed != NULL) ? observed->manualOnline : 0u;
    const uint8_t pressLeft = (observed != NULL) ? observed->mousePressL : 0u;
    const uint8_t pressRight = (observed != NULL) ? observed->mousePressR : 0u;

    ShootInputGateSyncSafeMouse(&s_shootInputGate,
                                manualOnline,
                                pressLeft,
                                pressRight);
}

static void ShootMouseRearmApply(ShootFrameInput *frame)
{
    if (frame == NULL)
    {
        return;
    }

    ShootInputGateApplyFrameMouse(&s_shootInputGate,
                                  frame->manualOnline,
                                  &frame->mousePressL,
                                  &frame->mousePressR);
}

static void ShootFrameInputCapture(const ManualInputSnapshot *manualInput,
                                   ShootFrameInput *frame)
{
    if (frame == NULL)
    {
        return;
    }

    memset(frame, 0, sizeof(*frame));
    frame->semantics = (manualInput != NULL) ?
                           manualInput->semantics :
                           ShootDefaultSemantics;
    frame->semanticsSeq = (manualInput != NULL) ? manualInput->semanticsSeq : 0u;
    frame->actionSeq = (manualInput != NULL) ? manualInput->actionSeq : 0u;
    frame->manualOnline = (uint8_t)(manualInput != NULL && manualInput->online != 0u);
    if (frame->manualOnline != 0u)
    {
        frame->mousePressL = manualInput->manual.mouse.press_l;
        frame->mousePressR = manualInput->manual.mouse.press_r;
    }

    if (frame->manualOnline != 0u &&
        (manualInput->sourceFlags & MANUAL_INPUT_SOURCE_FLAG_AUX_FIRE) != 0u)
    {
        frame->mousePressL = 1u;
    }

    frame->rawSwitch = ShootGetRawSwitch(manualInput,
                                         frame->manualOnline,
                                         &frame->semantics);
    frame->effectiveSwitch = frame->rawSwitch;
}

static void ShootFrameInputBuild(const ManualInputSnapshot *manualInput,
                                 ShootFrameInput *frame)
{
    ShootFrameInputCapture(manualInput, frame);
    if (frame == NULL)
    {
        return;
    }

    (void)ShootInputGateSyncSemantics(
        &s_shootInputGate,
        frame->semanticsSeq,
        ShootSwitchRawFromPos(frame->semantics.ShootStopPos));
    (void)ShootInputGateSyncAction(
        &s_shootInputGate,
        frame->actionSeq,
        ShootSwitchRawFromPos(frame->semantics.ShootStopPos));
    frame->effectiveSwitch = ShootGetEffectiveSwitch(frame->rawSwitch,
                                                     frame->manualOnline,
                                                     &frame->semantics);
    ShootMouseRearmApply(frame);
}



static ShootControl g_shoot;          // shoot control data

const ShootControl *get_shoot_control_point(void)
{
    return &g_shoot;
}

void ShootTuneApplyFricSpeedPid(void)
{
    taskENTER_CRITICAL();
    for (uint8_t i = 0; i < FRIC_MOTOR_NUM; i++)
    {
        PidTypeDef *dst = &g_shoot.fric_speed_pid[i];
        dst->Kp = g_config.shoot.fric_speed_pid.kp;
        dst->Ki = g_config.shoot.fric_speed_pid.ki;
        dst->Kd = g_config.shoot.fric_speed_pid.kd;
        dst->max_out = g_config.shoot.fric_speed_pid.max_out;
        dst->max_iout = g_config.shoot.fric_speed_pid.max_iout;
        PID_clear(dst);
    }
    taskEXIT_CRITICAL();
}

void ShootTuneApplyTriggerPid(void)
{
    taskENTER_CRITICAL();
    g_shoot.trigger_motor_pid.Kp = g_config.shoot.trigger_angle_pid.kp;
    g_shoot.trigger_motor_pid.Ki = g_config.shoot.trigger_angle_pid.ki;
    g_shoot.trigger_motor_pid.Kd = g_config.shoot.trigger_angle_pid.kd;
    PID_clear(&g_shoot.trigger_motor_pid);
    taskEXIT_CRITICAL();
}

static bool_t ShootGimbalCmdToShootStop(void)
{
    GimbalState state = {0};
    const uint8_t fresh = GimbalStateReadFresh(&state, GIMBAL_STATE_FRESH_TIMEOUT_MS);

    /* 云台状态缺失或过期时按不可射击处理，不能沿用任务卡死前的允许位。 */
    return (ShootGimbalStateBlocksFire(fresh, state.valid, state.fire_allowed) != 0u) ? 1 : 0;
}

static void ShootWriteState(void)
{
    ShootState state = {0};

    state.valid = 1u;
    state.mode = (uint8_t)g_shoot.mode;
    state.fric_speed_set = g_shoot.fric_speed_set;
    state.trigger_speed_set = g_shoot.trigger_speed_set;
    state.speed = g_shoot.speed;
    state.speed_set = g_shoot.speed_set;
    state.angle = g_shoot.angle;
    state.set_angle = g_shoot.set_angle;
    state.given_current = g_shoot.given_current;
    state.ecd_count = g_shoot.ecd_count;
    state.trigger_measure_ready = g_shoot.trigger_measure_ready;
    state.press_l = (uint8_t)g_shoot.press_l;
    state.press_r = (uint8_t)g_shoot.press_r;
    state.last_press_l = (uint8_t)g_shoot.last_press_l;
    state.last_press_r = (uint8_t)g_shoot.last_press_r;
    state.press_l_time = g_shoot.press_l_time;
    state.press_r_time = g_shoot.press_r_time;
    state.rc_s_time = g_shoot.rc_s_time;
    state.block_time = g_shoot.block_time;
    state.reverse_time = g_shoot.reverse_time;
    state.move_flag = (uint8_t)g_shoot.move_flag;
    state.key = (uint8_t)g_shoot.key;
    state.key_time = g_shoot.key_time;
    state.heat_limit = g_shoot.heat_limit;
    state.heat = g_shoot.heat;
    state.fault_configured_mask = s_shootFault.configuredMask;
    state.fault_active_mask = s_shootFault.activeMask;
    state.fault_blocking_mask = s_shootFault.blockingMask;
    state.fault_recovery_mask = s_shootFault.recoveryMask;
    state.fault_inhibit_mask = s_shootFault.inhibitMask;
    state.fault_hold_zero_mask = s_shootFault.holdZeroMask;
    state.fault_domain_action = s_shootFault.domainAction;
    state.trigger_fault_action = s_shootFault.triggerAction;
    state.fault_inhibit_fail_count = s_shootFault.inhibitFailCount;
    state.fault_release_fail_count = s_shootFault.releaseFailCount;
    state.trigger_motor_pid = g_shoot.trigger_motor_pid;

    for (uint8_t i = 0u; i < SHOOT_STATE_FAULT_DEVICE_COUNT; i++)
    {
        state.fault_reason[i] = s_shootFault.reason[i];
    }
    for (uint8_t i = 0u; i < FRIC_MOTOR_NUM && i < SHOOT_STATE_FRIC_MOTOR_COUNT; i++)
    {
        state.fric_speed_pid[i] = g_shoot.fric_speed_pid[i];
        state.fric_current_set[i] = g_shoot.fric_current_set[i];
    }

    (void)ShootStateWrite(&state);
}

static void ShootClearTriggerOutput(void)
{
    PID_clear(&g_shoot.trigger_motor_pid);
    g_shoot.trigger_speed_set = 0.0f;
    g_shoot.speed_set = 0.0f;
    g_shoot.given_current = 0;
    g_shoot.move_flag = 0;
    g_shoot.block_time = 0;
    g_shoot.reverse_time = 0;
}

static void ShootClearFricOutput(void)
{
    for (uint8_t i = 0; i < FRIC_MOTOR_NUM; i++)
    {
        PID_clear(&g_shoot.fric_speed_pid[i]);
        g_shoot.fric_current_set[i] = 0;
    }

    (void)MotorInstSetCurrentBindsBestEffort(ShootFrictionCurrentBindings,
                                                              ShootFricZeroCurrentCmd,
                                                              FRIC_MOTOR_NUM);

    g_shoot.fric_speed_ramp.out = SHOOT_FRIC_SPEED_OFF_RPM;
    g_shoot.fric_speed_set = SHOOT_FRIC_SPEED_OFF_RPM;
}

static uint8_t ShootFaultCollectIds(uint8_t mask,
                                    MotorId out[SHOOT_FAULT_DEVICE_COUNT],
                                    uint8_t *collectedMask)
{
    uint8_t count = 0u;
    uint8_t actualMask = 0u;

    for (uint8_t i = 0u; i < SHOOT_FAULT_DEVICE_COUNT; i++)
    {
        const uint8_t bit = (uint8_t)(1u << i);

        if ((mask & bit) == 0u ||
            (s_shootFault.configuredMask & bit) == 0u ||
            (uint32_t)s_shootFault.motorId[i] >= (uint32_t)MotorCount)
        {
            continue;
        }
        out[count++] = s_shootFault.motorId[i];
        actualMask |= bit;
    }

    if (collectedMask != NULL)
    {
        *collectedMask = actualMask;
    }
    return count;
}

static void ShootFaultSyncInhibit(void)
{
    const ShootFaultInhibitPlan plan =
        ShootFaultInhibitPlanMake(s_shootFault.configuredMask,
                                  (uint8_t)(1u << SHOOT_FAULT_TRIGGER_DEVICE),
                                  (s_shootFault.triggerAction != (uint8_t)FaultActionRun) ? 1u : 0u,
                                  (s_shootFault.domainAction != (uint8_t)FaultActionRun) ? 1u : 0u,
                                  s_shootFault.inhibitMask);
    MotorId ids[SHOOT_FAULT_DEVICE_COUNT];
    uint8_t actualMask = 0u;
    uint8_t count;

    s_shootFault.holdZeroMask = plan.holdZeroMask;

    count = ShootFaultCollectIds(plan.acquireMask, ids, &actualMask);
    if (count != 0u)
    {
        if (LowCmdInhibitManyFrom(ids, count, (uint16_t)LOWCMD_WRITER_SAFETY) != 0u)
        {
            s_shootFault.inhibitMask |= actualMask;
        }
        else
        {
            uint8_t acquiredMask = 0u;

            /* 某一轴已被更高优先级接管时，其余射击轴仍需逐个禁写。 */
            for (uint8_t i = 0u; i < SHOOT_FAULT_DEVICE_COUNT; i++)
            {
                const uint8_t bit = (uint8_t)(1u << i);
                const MotorId id = s_shootFault.motorId[i];

                if ((actualMask & bit) != 0u &&
                    LowCmdInhibitManyFrom(&id, 1u, (uint16_t)LOWCMD_WRITER_SAFETY) != 0u)
                {
                    acquiredMask |= bit;
                }
            }
            s_shootFault.inhibitMask |= acquiredMask;
            if (acquiredMask != actualMask)
            {
                s_shootFault.inhibitFailCount++;
            }
        }
    }

    count = ShootFaultCollectIds(plan.releaseMask, ids, &actualMask);
    if (count == 0u)
    {
        return;
    }

    /* 恢复帧先清内部状态和 LowCmd，再释放；holdZeroMask 让本帧仍保持零输出。 */
    if ((actualMask & (1u << SHOOT_FAULT_TRIGGER_DEVICE)) != 0u)
    {
        ShootClearTriggerOutput();
        g_shoot.trigger_measure_ready = 0u;
    }
    if ((actualMask & s_shootFault.fricMask) != 0u)
    {
        ShootClearFricOutput();
    }
    if (LowCmdClearManyFrom(ids, count, (uint16_t)LOWCMD_WRITER_SAFETY) != 0u &&
        LowCmdReleaseInhibitManyFrom(ids, count, (uint16_t)LOWCMD_WRITER_SAFETY) != 0u)
    {
        s_shootFault.inhibitMask &= (uint8_t)~actualMask;
        return;
    }

    {
        uint8_t releasedMask = 0u;

        for (uint8_t i = 0u; i < SHOOT_FAULT_DEVICE_COUNT; i++)
        {
            const uint8_t bit = (uint8_t)(1u << i);
            const MotorId id = s_shootFault.motorId[i];

            if ((actualMask & bit) != 0u &&
                LowCmdClearManyFrom(&id, 1u, (uint16_t)LOWCMD_WRITER_SAFETY) != 0u &&
                LowCmdReleaseInhibitManyFrom(&id, 1u, (uint16_t)LOWCMD_WRITER_SAFETY) != 0u)
            {
                releasedMask |= bit;
            }
        }
        s_shootFault.inhibitMask &= (uint8_t)~releasedMask;
        if (releasedMask != actualMask)
        {
            s_shootFault.releaseFailCount++;
        }
    }
}

void ShootRuntimeStop(void)
{
    ShootResetInputGate(NULL);
    ShootInputGateBlockMouse(&s_shootInputGate);
    g_shoot.mode = SHOOT_STOP;
    ShootClearTriggerOutput();
    ShootClearFricOutput();
    ShootWriteState();
}

static bool_t ShootFricSpeedReady(void)
{
    const fp32 speed_set = g_shoot.fric_speed_ramp.max_value;
    if (speed_set <= 1.0f)
    {
        return 1;
    }

    for (uint8_t i = 0; i < FRIC_MOTOR_NUM; i++)
    {
        const int8_t dir = SHOOT_FRIC_DIR(i);
        if (dir == 0)
        {
            continue;
        }

        const motor_measure_t *m = get_friction_motor_measure_point(i);
        if (m == NULL)
        {
            return 0;
        }

        if (fabsf((fp32)m->speed_rpm) < fabsf(speed_set) * SHOOT_FRIC_READY_RATIO)
        {
            return 0;
        }
    }

    return 1;
}


/**
  * @brief          射击初始化，初始化PID和电机指针
  * @param[in]      void
  * @retval         返回空
  */
void ShootRuntimeInit(void)
{

    const fp32 Trigger_speed_pid[3] = {TRIGGER_ANGLE_PID_KP, TRIGGER_ANGLE_PID_KI, TRIGGER_ANGLE_PID_KD};
    const fp32 Fric_speed_pid[3] = {g_config.shoot.fric_speed_pid.kp, g_config.shoot.fric_speed_pid.ki, g_config.shoot.fric_speed_pid.kd};
    (void)MotorInstBindCurrent(ShootFrictionMotorNames,
                                              FRIC_MOTOR_NUM,
                                              ShootFrictionCurrentBindings,
                                              FRIC_MOTOR_NUM);
    ShootFaultInit();
    ShootResetInputGate(NULL);
    ShootInputGateBlockMouse(&s_shootInputGate);
    g_shoot.mode = SHOOT_STOP;
    // motor feedback pointer
    g_shoot.ShootMotorMeasure = get_trigger_motor_measure_point();
    //初始化PID
    PID_init(&g_shoot.trigger_motor_pid, PID_POSITION, Trigger_speed_pid, TRIGGER_READY_PID_MAX_OUT, TRIGGER_READY_PID_MAX_IOUT);
    for (uint8_t i = 0; i < FRIC_MOTOR_NUM; i++)
    {
        PID_init(&g_shoot.fric_speed_pid[i], PID_POSITION, Fric_speed_pid, g_config.shoot.fric_speed_pid.max_out, g_config.shoot.fric_speed_pid.max_iout);
        g_shoot.fric_current_set[i] = 0;
    }
    ramp_init(&g_shoot.fric_speed_ramp, SHOOT_CONTROL_TIME * 0.001f, SHOOT_FRIC_SPEED_RPM, SHOOT_FRIC_SPEED_OFF_RPM);
    g_shoot.fric_speed_ramp.out = SHOOT_FRIC_SPEED_OFF_RPM;
    g_shoot.fric_speed_set = SHOOT_FRIC_SPEED_OFF_RPM;
    g_shoot.ecd_count = 0;
    g_shoot.angle = (g_shoot.ShootMotorMeasure != NULL) ?
        (g_shoot.ShootMotorMeasure->ecd * MOTOR_ECD_TO_ANGLE) : 0.0f;
    g_shoot.trigger_measure_ready = 0u;
    g_shoot.given_current = 0;
    g_shoot.move_flag = 0;
    g_shoot.set_angle = g_shoot.angle;
    g_shoot.speed = 0.0f;
    g_shoot.speed_set = 0.0f;
    g_shoot.key_time = 0;
    ShootWriteState();
}

void ShootRuntimeSafeStep(const struct ManualInputSnapshot *manualInput)
{
    ShootFrameInput observed;
    ShootFrameInput safeFrame = {0};
    uint8_t switchSafe;

    ShootFrameInputCapture(manualInput, &observed);
    switchSafe = (uint8_t)(observed.manualOnline != 0u &&
                           ShootSwitchIsStop(observed.rawSwitch,
                                             &observed.semantics) != 0u);
    ShootFaultUpdate(switchSafe);
    ShootFaultSyncInhibit();

    (void)ShootInputGateSyncSemantics(
        &s_shootInputGate,
        observed.semanticsSeq,
        ShootSwitchRawFromPos(observed.semantics.ShootStopPos));
    (void)ShootInputGateSyncAction(
        &s_shootInputGate,
        observed.actionSeq,
        ShootSwitchRawFromPos(observed.semantics.ShootStopPos));
    ShootResetInputGate(&observed.semantics);
    ShootMouseRearmSyncSafe(&observed);
    safeFrame.semantics = observed.semantics;
    safeFrame.semanticsSeq = observed.semanticsSeq;
    safeFrame.actionSeq = observed.actionSeq;
    safeFrame.rawSwitch = ShootSwitchRawFromPos(safeFrame.semantics.ShootStopPos);
    safeFrame.effectiveSwitch = safeFrame.rawSwitch;
    g_shoot.mode = SHOOT_STOP;
    ShootFeedbackUpdate(&safeFrame);
    ShootClearTriggerOutput();
    ShootClearFricOutput();
    ShootWriteState();
}

/**
  * @brief          射击循环
  * @param[in]      void
  * @retval         返回can控制值
  */
int16_t ShootRuntimeStep(const struct ManualInputSnapshot *manualInput)
{
    static uint8_t entertain_entered = 0u;
    ShootFrameInput frame;

    ShootFrameInputBuild(manualInput, &frame);
    const uint16_t rawSwitch = frame.rawSwitch;
    const uint8_t switchSafe =
        (frame.manualOnline != 0u &&
         ShootSwitchIsStop(rawSwitch, &frame.semantics) != 0u) ? 1u : 0u;

    ShootFaultUpdate(switchSafe);
    ShootFaultSyncInhibit();

    // 函数地图：先处理娱乐模式；再跑射击状态机；最后分别输出拨弹和摩擦轮电流。
    if (robot_mode_is_entertain() != 0u)
    {
        if (entertain_entered == 0u)
        {
            entertain_entered = 1u;
            g_shoot.mode = SHOOT_STOP;
            ShootClearTriggerOutput();
            ShootClearFricOutput();
        }
        else
        {
            g_shoot.mode = SHOOT_STOP;
        }
        ShootWriteState();
        return 0;
    }
    entertain_entered = 0u;

    if (ShootFaultStopsDomain() != 0u)
    {
        g_shoot.mode = SHOOT_STOP;
        ShootClearTriggerOutput();
        ShootClearFricOutput();
        g_shoot.trigger_measure_ready = 0u;
        ShootWriteState();
        return 0;
    }

    ShootSetMode(&frame);        //设置状态机
    ShootFeedbackUpdate(&frame); // update feedback data
    const bool_t allow_fric = (robot_mode_allow_shoot_fric() != 0u) ? 1 : 0;
    const bool_t allow_trigger = (robot_mode_allow_shoot_trigger() != 0u) ? 1 : 0;
    const uint16_t ShootSw = frame.effectiveSwitch;
    const bool_t sw_ready = ShootSwitchIsReady(ShootSw, &frame.semantics);
    const bool_t sw_fire = ShootSwitchIsFire(ShootSw, &frame.semantics);


    if (g_shoot.mode == SHOOT_STOP)
    {
        //设置拨弹轮的速度
        g_shoot.speed_set = 0.0f;
    }
    else if (g_shoot.mode == SHOOT_READY_FRIC)
    {
        //设置拨弹轮的速度
        g_shoot.speed_set = 0.0f;
    }
    else if(g_shoot.mode ==SHOOT_READY_BULLET)
    {
        if(g_shoot.key == SWITCH_TRIGGER_OFF)
        {
            //设置拨弹轮的拨动速度,并开启堵转反转处理
            g_shoot.trigger_speed_set = READY_TRIGGER_SPEED;
            trigger_motor_turn_back();
        }
        else
        {
            g_shoot.trigger_speed_set = 0.0f;
            g_shoot.speed_set = 0.0f;
        }
        g_shoot.trigger_motor_pid.max_out = TRIGGER_READY_PID_MAX_OUT;
        g_shoot.trigger_motor_pid.max_iout = TRIGGER_READY_PID_MAX_IOUT;
    }
    else if (g_shoot.mode == SHOOT_READY)
    {
        //设置拨弹轮的速度
         g_shoot.speed_set = 0.0f;
    }
    else if (g_shoot.mode == SHOOT_BULLET)
    {
        g_shoot.trigger_motor_pid.max_out = TRIGGER_BULLET_PID_MAX_OUT;
        g_shoot.trigger_motor_pid.max_iout = TRIGGER_BULLET_PID_MAX_IOUT;
        ShootBulletControl();
    }
    else if (g_shoot.mode == SHOOT_CONTINUE_BULLET)
    {
        //设置拨弹轮的拨动速度,并开启堵转反转处理
        g_shoot.trigger_speed_set = CONTINUE_TRIGGER_SPEED;
        trigger_motor_turn_back();
    }
    else if(g_shoot.mode == SHOOT_DONE)
    {
        g_shoot.speed_set = 0.0f;
    }

    if (ShootFaultStopsTrigger() != 0u)
    {
        /* 故障隔离必须覆盖本周期前面可能计算出的旧 PID 输出。 */
        ShootClearTriggerOutput();
        g_shoot.trigger_measure_ready = 0u;
    }
    else if(g_shoot.mode == SHOOT_STOP)
    {
        // STOP overwrites LowCmd with zero current. Do not run the speed PID toward 0 RPM.
        ShootClearTriggerOutput();
        ShootClearFricOutput();
    }
    else
    {
        PID_calc(&g_shoot.trigger_motor_pid, g_shoot.speed, g_shoot.speed_set);
        g_shoot.given_current = (int16_t)(g_shoot.trigger_motor_pid.out);
        if(g_shoot.mode < SHOOT_READY_BULLET)
        {
            g_shoot.given_current = 0;
        }
    }

    if (g_shoot.mode != SHOOT_STOP)
    {
        /* 拨弹轴隔离后，健康摩擦轮仍可按原策略预热。 */
        const fp32 fric_ramp_step = sw_ready ?
            (SHOOT_FRIC_SPEED_STEP_RPM_S * 0.5f) : SHOOT_FRIC_SPEED_STEP_RPM_S;
        ramp_calc(&g_shoot.fric_speed_ramp, fric_ramp_step);
    }

    if(!allow_trigger || !sw_fire || ShootFaultStopsTrigger() != 0u)
    {
        ShootClearTriggerOutput();
    }

    if(!allow_fric)
    {
        ShootClearFricOutput();
    }

    if(!allow_fric && !allow_trigger)
    {
        g_shoot.mode = SHOOT_STOP;
    }

    // 速度环：每路用反馈转速闭环输出电流
    if (allow_fric && g_shoot.mode != SHOOT_STOP)
    {
        int16_t fric_current_cmd[FRIC_MOTOR_NUM] = {0};
        g_shoot.fric_speed_set = g_shoot.fric_speed_ramp.out;
        for (uint8_t i = 0; i < FRIC_MOTOR_NUM; i++)
        {
            const int8_t dir = SHOOT_FRIC_DIR(i);
            if (dir == 0)
            {
                g_shoot.fric_current_set[i] = 0;
                PID_clear(&g_shoot.fric_speed_pid[i]);
                fric_current_cmd[i] = 0;
                continue;
            }

            const motor_measure_t *m = get_friction_motor_measure_point(i);
            const fp32 speed_fdb = (m != NULL) ? (fp32)m->speed_rpm : 0.0f;
            const fp32 speed_set = g_shoot.fric_speed_set * (fp32)dir;
            const int16_t current_raw = (int16_t)PID_calc(&g_shoot.fric_speed_pid[i], speed_fdb, speed_set);
            const int16_t current = MotorCfgLimitCurrentNode(&g_config.motor.friction[i], current_raw);
            g_shoot.fric_current_set[i] = current;
            fric_current_cmd[i] = current;
        }

        (void)MotorInstSetCurrentBindsBestEffort(ShootFrictionCurrentBindings,
                                                                  fric_current_cmd,
                                                                  FRIC_MOTOR_NUM);
    }
    ShootWriteState();
    return g_shoot.given_current;
}

/**
  * @brief          射击状态机设置：上=停火；中=仅摩擦轮预热（拨盘不动）；下=允许拨盘进入预备/射击（等同原“中档”逻辑）
  * @param[in]      void
  * @retval         void
  */
static void ShootSetMode(const ShootFrameInput *frame)
{
    // 函数地图：先按拨杆定大状态，再叠加运行模式、鼠标/微动开关和完成/堵转条件。
    const ManualInputSemanticsConfig *semantics =
        ShootSemanticsOrDefault((frame != NULL) ? &frame->semantics : NULL);
    const bool_t allow_fric = (robot_mode_allow_shoot_fric() != 0u) ? 1 : 0;
    const bool_t allow_trigger = (robot_mode_allow_shoot_trigger() != 0u) ? 1 : 0;
    const uint16_t ShootSw = (frame != NULL) ?
                                 frame->rawSwitch :
                                 ShootSwitchRawFromPos(semantics->ShootStopPos);
    const bool_t sw_fire = ShootSwitchIsFire(ShootSw, semantics);

    // 拨杆位置优先控制：上=停火；中=预热摩擦轮（拨盘不动）；下=允许拨盘进入 READY_BULLET/READY 等逻辑
    if (ShootSwitchIsStop(ShootSw, semantics))
    {
        g_shoot.mode = SHOOT_STOP;
    }
    else if (ShootSwitchIsReady(ShootSw, semantics))
    {
        // 进入预热：摩擦轮加速至目标，等待 READY_BULLET
        g_shoot.mode = SHOOT_READY_FRIC;
    }
    else if (ShootSwitchIsFire(ShootSw, semantics))
    {
        // 启动射击：摩擦轮提速，准备好后持续拨盘
        g_shoot.mode = SHOOT_READY_FRIC;
    }

    const bool_t fric_set_done = (g_shoot.fric_speed_ramp.out == g_shoot.fric_speed_ramp.max_value);
    const bool_t fric_ready = fric_set_done && ShootFricSpeedReady();

    if(sw_fire && g_shoot.mode == SHOOT_READY_FRIC && (fric_ready || (!allow_fric && allow_trigger)))
    {
        g_shoot.mode = SHOOT_READY_BULLET;
    }
    else if(g_shoot.mode == SHOOT_READY_BULLET && g_shoot.key == SWITCH_TRIGGER_ON)
    {
        g_shoot.mode = SHOOT_READY;
    }
    else if(g_shoot.mode == SHOOT_READY && g_shoot.key == SWITCH_TRIGGER_OFF)
    {
        g_shoot.mode = SHOOT_READY_BULLET;
    }
    else if(g_shoot.mode == SHOOT_READY)
    {
        // 仅鼠标按键边沿启动射击（下档不再自动连发/触发）
        if (frame != NULL &&
            ((frame->mousePressL != 0u && g_shoot.press_l == 0) ||
             (frame->mousePressR != 0u && g_shoot.press_r == 0)))
        {
            g_shoot.mode = SHOOT_BULLET;
        }
    }
    else if(g_shoot.mode == SHOOT_DONE)
    {
        if(g_shoot.key == SWITCH_TRIGGER_OFF)
        {
            g_shoot.key_time = ShootU16AddSat(g_shoot.key_time, ShootTickMs(), SHOOT_DONE_KEY_OFF_TIME);
            if(g_shoot.key_time >= SHOOT_DONE_KEY_OFF_TIME)
            {
                g_shoot.key_time = 0;
                g_shoot.mode = SHOOT_READY_BULLET;
            }
        }
        else
        {
            g_shoot.key_time = 0;
            g_shoot.mode = SHOOT_BULLET;
        }
    }



    if(g_shoot.mode > SHOOT_READY_FRIC)
    {
        //鼠标长按一直进入射击状态 保持连发
        if ((g_shoot.press_l_time == PRESS_LONG_TIME) || (g_shoot.press_r_time == PRESS_LONG_TIME))
        {
            g_shoot.mode = SHOOT_CONTINUE_BULLET;
        }
        else if(g_shoot.mode == SHOOT_CONTINUE_BULLET)
        {
            g_shoot.mode =SHOOT_READY_BULLET;
        }
    }

    get_shoot_heat0_limit_and_heat0(&g_shoot.heat_limit, &g_shoot.heat);
    if(!toe_is_error(REFEREE_TOE) && (g_shoot.heat + SHOOT_HEAT_REMAIN_VALUE > g_shoot.heat_limit))
    {
        if(g_shoot.mode == SHOOT_BULLET ||
           g_shoot.mode == SHOOT_CONTINUE_BULLET ||
           g_shoot.mode == SHOOT_READY)
        {
            g_shoot.mode =SHOOT_READY_BULLET;
        }
    }
    //如果云台状态是 无力状态，就关闭射击
    if (ShootGimbalCmdToShootStop())
    {
        g_shoot.mode = SHOOT_STOP;
    }

    if(!allow_trigger && g_shoot.mode > SHOOT_READY_FRIC)
    {
        g_shoot.mode = SHOOT_READY_FRIC;
    }

    if(!allow_fric && !allow_trigger)
    {
        g_shoot.mode = SHOOT_STOP;
    }

}
/**
  * @brief          update shoot feedback data.
  * @param[in]      void
  * @retval         void
  */
static void ShootFeedbackUpdate(const ShootFrameInput *frame)
{
    // 函数地图：滤波拨弹速度；维护编码器圈数/输出角；读取微动开关；更新长按和发射完成状态。
    const ManualInputSemanticsConfig *semantics =
        ShootSemanticsOrDefault((frame != NULL) ? &frame->semantics : NULL);
    static const fp32 fliter_num[3] = {1.725709860247969f, -0.75594777109163436f, 0.030237910843665373f};
    static second_order_filter_type_t speed_filter;
    static bool_t speed_filter_inited = 0;
    const uint16_t tick_ms = ShootTickMs();
    const uint8_t trigger_online = ShootFaultTriggerUsable();

    //拨弹轮电机速度滤波一下（二阶低通）
    if (!speed_filter_inited)
    {
        second_order_filter_init(&speed_filter, fliter_num, 0.0f);
        speed_filter_inited = 1;
    }

    if (trigger_online == 0u)
    {
        g_shoot.trigger_measure_ready = 0u;
    }

    if (g_shoot.ShootMotorMeasure == NULL)
    {
        g_shoot.speed = second_order_filter_cali(&speed_filter, 0.0f);
    }
    else
    {
        g_shoot.speed = second_order_filter_cali(&speed_filter,
                                                       g_shoot.ShootMotorMeasure->speed_rpm * MOTOR_RPM_TO_SPEED);
    }

    //电机圈数重置， 因为输出轴旋转一圈， 电机轴旋转 36圈，将电机轴数据处理成输出轴数据，用于控制输出轴角度
    if (g_shoot.ShootMotorMeasure != NULL)
    {
        if (g_shoot.trigger_measure_ready == 0u)
        {
            g_shoot.ecd_count = 0;
            g_shoot.angle = g_shoot.ShootMotorMeasure->ecd * MOTOR_ECD_TO_ANGLE;
            g_shoot.set_angle = g_shoot.angle;
            g_shoot.move_flag = 0;
            g_shoot.trigger_measure_ready = trigger_online;
        }
        else
        {
            if (g_shoot.ShootMotorMeasure->ecd - g_shoot.ShootMotorMeasure->last_ecd > HALF_ECD_RANGE)
            {
                g_shoot.ecd_count--;
            }
            else if (g_shoot.ShootMotorMeasure->ecd - g_shoot.ShootMotorMeasure->last_ecd < -HALF_ECD_RANGE)
            {
                g_shoot.ecd_count++;
            }

            if (g_shoot.ecd_count == FULL_COUNT)
            {
                g_shoot.ecd_count = -(FULL_COUNT - 1);
            }
            else if (g_shoot.ecd_count == -FULL_COUNT)
            {
                g_shoot.ecd_count = FULL_COUNT - 1;
            }

            g_shoot.angle = (g_shoot.ecd_count * ECD_RANGE + g_shoot.ShootMotorMeasure->ecd) * MOTOR_ECD_TO_ANGLE;
        }
    }
    //微动开关
    uint8_t trig_level = 0u;
    if (BspShootTrigReadRaw(&trig_level) != 0u)
    {
        g_shoot.key = (bool_t)trig_level;
    }
    else
    {
        g_shoot.key = (bool_t)SWITCH_TRIGGER_OFF;
    }
    //榧犳爣鎸夐敭
    g_shoot.last_press_l = g_shoot.press_l;
    g_shoot.last_press_r = g_shoot.press_r;
    g_shoot.press_l = (frame != NULL) ? frame->mousePressL : 0u;
    g_shoot.press_r = (frame != NULL) ? frame->mousePressR : 0u;
    //长按计时
    if (g_shoot.press_l)
    {
        if (g_shoot.press_l_time < PRESS_LONG_TIME)
        {
            g_shoot.press_l_time = ShootU16AddSat(g_shoot.press_l_time, tick_ms, PRESS_LONG_TIME);
        }
    }
    else
    {
        g_shoot.press_l_time = 0;
    }

    if (g_shoot.press_r)
    {
        if (g_shoot.press_r_time < PRESS_LONG_TIME)
        {
            g_shoot.press_r_time = ShootU16AddSat(g_shoot.press_r_time, tick_ms, PRESS_LONG_TIME);
        }
    }
    else
    {
        g_shoot.press_r_time = 0;
    }

    //射击开关下档时间计时
    const uint16_t effective_sw = (frame != NULL) ?
                                      frame->effectiveSwitch :
                                      ShootSwitchRawFromPos(semantics->ShootStopPos);
    if (g_shoot.mode != SHOOT_STOP && ShootSwitchIsFire(effective_sw, semantics))
    {
        if (g_shoot.rc_s_time < RC_S_LONG_TIME)
        {
            g_shoot.rc_s_time = ShootU16AddSat(g_shoot.rc_s_time, tick_ms, RC_S_LONG_TIME);
        }
    }
    else
    {
        g_shoot.rc_s_time = 0;
    }

    // 摩擦轮只保留一个目标转速，不再区分左键/右键高速。
    g_shoot.fric_speed_ramp.max_value = SHOOT_FRIC_SPEED_RPM;


}

static void trigger_motor_turn_back(void)
{
    const uint16_t tick_ms = ShootTickMs();

    if( g_shoot.block_time < BLOCK_TIME)
    {
        g_shoot.speed_set = g_shoot.trigger_speed_set;
    }
    else
    {
        g_shoot.speed_set = -g_shoot.trigger_speed_set;
    }

    if(fabsf(g_shoot.speed) < BLOCK_TRIGGER_SPEED && g_shoot.block_time < BLOCK_TIME)
    {
        g_shoot.block_time = ShootU16AddSat(g_shoot.block_time, tick_ms, BLOCK_TIME);
        g_shoot.reverse_time = 0;
    }
    else if (g_shoot.block_time >= BLOCK_TIME && g_shoot.reverse_time < REVERSE_TIME)
    {
        g_shoot.reverse_time = ShootU16AddSat(g_shoot.reverse_time, tick_ms, REVERSE_TIME);
    }
    else
    {
        g_shoot.block_time = 0u;
    }
}

/**
  * @brief          射击控制，控制拨弹电机角度，完成一次发射
  * @param[in]      void
  * @retval         void
  */
static void ShootBulletControl(void)
{

    //每次拨动 1/4PI的角度
    if (g_shoot.move_flag == 0)
    {
        g_shoot.set_angle = rad_format(g_shoot.angle + PI_TEN);
        g_shoot.move_flag = 1;
    }
    if(g_shoot.key == SWITCH_TRIGGER_OFF)
    {

        g_shoot.mode = SHOOT_DONE;
    }
    // check whether target angle has been reached
    if (rad_format(g_shoot.set_angle - g_shoot.angle) > 0.05f)
    {
        //没到达一直设置旋转速度
        g_shoot.trigger_speed_set = TRIGGER_SPEED;
        trigger_motor_turn_back();
    }
    else
    {
        g_shoot.move_flag = 0;
    }
}

#endif
