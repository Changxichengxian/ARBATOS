/*
 * SPDX-FileCopyrightText: 2026 陈轩 <2811158416@qq.com>
 * SPDX-License-Identifier: Apache-2.0
 *
 * 步兵车 UART6 运行观测；只读取正式运行时快照，不参与控制与 CAN 收发。
 */

#include "BspCan.h"
#include "CanTxTask.h"
#include "ChassisState.h"
#include "GimbalState.h"
#include "LowCmd.h"
#include "ManualInputSnapshot.h"
#include "MotorInst.h"
#include "MotorConfig.h"
#include "RobotConfig.h"
#include "RobotLifecycle.h"
#include "RtProf.h"
#include "ShootState.h"

#include <stdint.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>

#define INFANTRY_DIAG_ROUTE_CAP 9u

/* A 板 USART6：PG14 发送、PG9 接收，设备树固定为 115200。 */
static const struct device *const InfantryDiagUart = DEVICE_DT_GET(DT_NODELABEL(usart6));

static void InfantryDiagWrite(const char *text)
{
    if (text == NULL) {
        return;
    }

    while (*text != '\0') {
        uart_poll_out(InfantryDiagUart, (unsigned char)*text++);
    }
}

static const char *InfantryDiagInputSourceName(uint8_t source)
{
    switch (source) {
    case MANUAL_INPUT_SRC_AUTO:
        return "AUTO";
    case MANUAL_INPUT_SRC_DBUS:
        return "DBUS";
    case MANUAL_INPUT_SRC_ELRS:
        return "ELRS";
    case MANUAL_INPUT_SRC_IMAGE:
        return "IMAGE";
    case MANUAL_INPUT_SRC_USB:
        return "USB";
    default:
        return "UNKNOWN";
    }
}

static void InfantryDiagCanReport(uint8_t bus)
{
    char line[192];

    snprintk(line, sizeof(line),
             "CAN%u rx=%u tx=%u fail=%u drop=%u err=0x%08x rxerr=%u txerr=%u off=%u\r\n",
             (unsigned)bus, (unsigned)BspCanRxGetCount(bus),
             (unsigned)BspCanGetTxCount(bus), (unsigned)BspCanGetTxFailCount(bus),
             (unsigned)BspCanRxGetDropCount(bus), (unsigned)BspCanGetLastError(bus),
             (unsigned)BspCanGetRxErrorCount(bus), (unsigned)BspCanGetTxErrorCount(bus),
             (unsigned)BspCanGetProtocolBusOff(bus));
    InfantryDiagWrite(line);
}

static void InfantryDiagMotorsReport(void)
{
    const MotorRoute *routes[INFANTRY_DIAG_ROUTE_CAP];
    MotorId ids[INFANTRY_DIAG_ROUTE_CAP];
    MotorState feedback[INFANTRY_DIAG_ROUTE_CAP];
    uint8_t count = 0u;
    char line[288];

    for (uint8_t index = 0u; index < MotorRouteCount() && count < INFANTRY_DIAG_ROUTE_CAP;
         index++) {
        const MotorRoute *route = MotorRouteGet(index);

        if (route == NULL || route->enabled == 0u) {
            continue;
        }
        routes[count] = route;
        ids[count] = route->motorId;
        count++;
    }

    if (MotorInstGetFeedbackIds(ids, feedback, count) == 0u) {
        InfantryDiagWrite("MOTOR snapshot=unavailable\r\n");
        return;
    }

    // 先复制反馈再取时刻，避免串口打印期间的新反馈导致无符号 age 下溢。
    const uint32_t now = k_uptime_get_32();
    for (uint8_t index = 0u; index < count; index++) {
        const MotorRoute *route = routes[index];
        const MotorState *state = &feedback[index];
        MotorCmd command = {0};
        uint16_t txGroup = 0u;
        uint8_t txSlot = 0u;
        const char *drive = (route->controlMode == MOTOR_CONTROL_MODE_VOLTAGE) ? "VOLTAGE" : "CURRENT";
        const uint32_t age = (state->rxCount != 0u) ? (now - state->lastRxTick) : UINT32_MAX;
        const uint8_t commandValid = LowCmdGetMotor(route->motorId, &command);
        (void)MotorCfgRmTxSlot(route->node, &txGroup, &txSlot);

        snprintk(line, sizeof(line),
                 "MOTOR %s id=%u bus=%u cfg_fb=0x%03x online=%u last_fb=0x%03x "
                 "count=%u age=%u rpm=%d fb_cur=%d cmd=%u mode=%u cmd_cur=%d drive=%s cfg_tx=0x%03x slot=%u\r\n",
                 (route->name != NULL) ? route->name : "unnamed", (unsigned)route->motorId,
                 (unsigned)route->bus, (unsigned)route->feedbackId, (unsigned)state->online,
                 (unsigned)state->rxId, (unsigned)state->rxCount, (unsigned)age,
                 (int)state->speedRpm, (int)state->current,
                 (unsigned)((commandValid != 0u) ? command.active : 0u),
                 (unsigned)((commandValid != 0u) ? command.mode : MotorModeNone),
                 (int)((commandValid != 0u) ? command.current : 0),
                 drive, (unsigned)txGroup, (unsigned)(txSlot + 1u));
        InfantryDiagWrite(line);
    }
}

static void InfantryDiagChassisReport(void)
{
    ChassisState chassis = {0};
    char line[320];

    if (ChassisStateRead(&chassis) == 0u || chassis.valid == 0u) {
        InfantryDiagWrite("CHASSIS snapshot=unavailable\r\n");
        return;
    }

    snprintk(line, sizeof(line),
             "CHASSIS mode=%u last=%u vx_vy_mms_wz_mrad=%d,%d,%d wheel_set_mms=%d,%d,%d,%d\r\n",
             (unsigned)chassis.mode, (unsigned)chassis.last_mode,
             (int)(chassis.vx_set * 1000.0f), (int)(chassis.vy_set * 1000.0f),
             (int)(chassis.wz_set * 1000.0f),
             (int)(chassis.motor[0].speed_set * 1000.0f),
             (int)(chassis.motor[1].speed_set * 1000.0f),
             (int)(chassis.motor[2].speed_set * 1000.0f),
             (int)(chassis.motor[3].speed_set * 1000.0f));
    InfantryDiagWrite(line);

    snprintk(line, sizeof(line),
             "CHASSIS_HEALTH cfg=%02x active=%02x block=%02x recover=%02x inhibit=%02x zero=%02x safe=%u "
             "age=%u,%u,%u,%u reason=%04x,%04x,%04x,%04x\r\n",
             (unsigned)chassis.fault_configured_mask, (unsigned)chassis.fault_active_mask,
             (unsigned)chassis.fault_blocking_mask, (unsigned)chassis.fault_recovery_mask,
             (unsigned)chassis.fault_inhibit_mask, (unsigned)chassis.fault_hold_zero_mask,
             (unsigned)chassis.fault_recovery_input_safe,
             (unsigned)chassis.motor_feedback_age_ms[0], (unsigned)chassis.motor_feedback_age_ms[1],
             (unsigned)chassis.motor_feedback_age_ms[2], (unsigned)chassis.motor_feedback_age_ms[3],
             (unsigned)chassis.motor_reason_mask[0], (unsigned)chassis.motor_reason_mask[1],
             (unsigned)chassis.motor_reason_mask[2], (unsigned)chassis.motor_reason_mask[3]);
    InfantryDiagWrite(line);
}

static void InfantryDiagGimbalReport(void)
{
    static GimbalState gimbal;
    char line[256];

    if (GimbalStateReadFresh(&gimbal, GIMBAL_STATE_FRESH_TIMEOUT_MS) == 0u) {
        InfantryDiagWrite("GIMBAL snapshot=unavailable\r\n");
        return;
    }
    snprintk(line, sizeof(line),
             "GIMBAL at=%u imu=%u age=%u feedback=%u block=%02x "
             "yaw_mrad=%d,%d yaw_mradps=%d,%d pitch_mrad=%d,%d pitch_mradps=%d,%d cmd=%d,%d\r\n",
             (unsigned)k_uptime_get_32(), (unsigned)gimbal.imu_online,
             (unsigned)gimbal.imu_age_ms, (unsigned)gimbal.feedback_mode,
             (unsigned)gimbal.feedback_block_mask,
             (int)(gimbal.yaw.angle_set * 1000.0f), (int)(gimbal.yaw.angle * 1000.0f),
             (int)(gimbal.yaw.motor_gyro_set * 1000.0f), (int)(gimbal.yaw.motor_gyro * 1000.0f),
             (int)(gimbal.pitch.angle_set * 1000.0f), (int)(gimbal.pitch.angle * 1000.0f),
             (int)(gimbal.pitch.motor_gyro_set * 1000.0f), (int)(gimbal.pitch.motor_gyro * 1000.0f),
             (int)gimbal.yaw.given_current, (int)gimbal.pitch.given_current);
    InfantryDiagWrite(line);
}

static void InfantryDiagTimingReport(void)
{
    static const RtProfId ids[] = {
        RtProfCanRxWake, RtProfCanTxLoop, RtProfGimbalLoop, RtProfChassisLoop,
    };
    static const char *const names[] = {"can_rx", "can_tx", "gimbal", "chassis"};
    char line[160];

    for (uint8_t index = 0u; index < ARRAY_SIZE(ids); index++) {
        RtProfStats stats = {0};
        RtProfGet(ids[index], &stats);
        snprintk(line, sizeof(line),
                 "TIMING %s count=%u last_us=%u avg_us=%u max_us=%u budget_us=%u overruns=%u\r\n",
                 names[index], (unsigned)stats.count, (unsigned)stats.last_us,
                 (unsigned)stats.avg_us, (unsigned)stats.max_us, (unsigned)stats.budget_us,
                 (unsigned)stats.overrun_count);
        InfantryDiagWrite(line);
    }
}

static void InfantryDiagMotionReport(void)
{
    /* 各控制域分别有自己的采样时刻；先复制全部数据，再打印，避免把串口耗时当成同一帧。 */
    static GimbalState gimbal;
    static ChassisState chassis;
    static ShootState shoot;
    static MotorState motors[5];
    static MotorCmd commands[5];
    static MotorTxReceipt receipts[5];
    static MotorApplied applied[5];
    uint32_t rejects[5];
    MotorId ids[5];
    state_info_t gi = {0}, ci = {0}, si = {0};
    ManualInputSnapshot input = {0};
    RobotLifecycleSnapshot lifecycle = {0};
    uint8_t count = 0u;
    const uint32_t begin = k_uptime_get_32();
    char line[384];

    const uint8_t gv = StateStoreReadSnapshot(STATE_GIMBAL, &gimbal, sizeof(gimbal), &gi);
    const uint8_t cv = StateStoreReadSnapshot(STATE_CHASSIS, &chassis, sizeof(chassis), &ci);
    const uint8_t sv = StateStoreReadSnapshot(STATE_SHOOT, &shoot, sizeof(shoot), &si);
    const uint8_t iv = ManualInputSnapshotRead(&input);
    (void)RobotLifecycleGetSnapshot(&lifecycle);
    if (cv != 0u && chassis.mode == CHASSIS_STATE_MODE_NO_FOLLOW_YAW &&
        (chassis.wz_set > 0.5f || chassis.wz_set < -0.5f)) {
        // 小陀螺用紧凑记录缩短串口阻塞；轮速和电流来自同一份底盘快照。
        snprintk(line, sizeof(line),
                 "SPIN t=%u gt=%u age=%u valid=%u,%u,%u out=%u ch=%d,%d "
                 "ecd=%u,%u yaw=%d vs=%d,%d v=%d,%d wz=%d,%d "
                 "ws=%d,%d,%d,%d rpm=%d,%d,%d,%d cur=%d,%d,%d,%d zero=%u\r\n",
                 (unsigned)ci.write_tick_ms, (unsigned)gi.write_tick_ms, (unsigned)input.sourceAgeMs,
                 (unsigned)cv, (unsigned)gv, (unsigned)iv, (unsigned)lifecycle.output_allowed,
                 (int)input.manual.rc.ch[3], (int)input.manual.rc.ch[2],
                 (unsigned)gimbal.yaw.measure.ecd, (unsigned)gimbal.yaw.offset_ecd,
                 (int)(gimbal.yaw.angle * 1000.0f),
                 (int)(chassis.vx_set * 1000.0f), (int)(chassis.vy_set * 1000.0f),
                 (int)(chassis.vx * 1000.0f), (int)(chassis.vy * 1000.0f),
                 (int)(chassis.wz_set * 1000.0f), (int)(chassis.wz * 1000.0f),
                 (int)(chassis.motor[0].speed_set * 1000.0f), (int)(chassis.motor[1].speed_set * 1000.0f),
                 (int)(chassis.motor[2].speed_set * 1000.0f), (int)(chassis.motor[3].speed_set * 1000.0f),
                 (int)chassis.motor[0].measure.speed_rpm, (int)chassis.motor[1].measure.speed_rpm,
                 (int)chassis.motor[2].measure.speed_rpm, (int)chassis.motor[3].measure.speed_rpm,
                 (int)chassis.motor[0].give_current, (int)chassis.motor[1].give_current,
                 (int)chassis.motor[2].give_current, (int)chassis.motor[3].give_current,
                 (unsigned)chassis.fault_hold_zero_mask);
        InfantryDiagWrite(line);
        return;
    }
    for (uint8_t index = 0u; index < MotorRouteCount() && count < ARRAY_SIZE(ids); index++) {
        const MotorRoute *route = MotorRouteGet(index);
        if (route != NULL && route->enabled != 0u &&
            (route->role == MotorRoleYaw || route->role == MotorRolePitch ||
             route->role == MotorRoleTrigger || route->role == MotorRoleFriction)) {
            ids[count++] = route->motorId;
        }
    }
    const uint8_t mv = MotorInstGetFeedbackIds(ids, motors, count);
    for (uint8_t index = 0u; index < count; index++) {
        commands[index] = (MotorCmd){0};
        receipts[index] = (MotorTxReceipt){0};
        applied[index] = (MotorApplied){0};
        (void)LowCmdGetMotor(ids[index], &commands[index]);
        (void)LowStateGetTxReceipt(ids[index], &receipts[index]);
        (void)LowStateGetApplied(ids[index], &applied[index]);
        rejects[index] = CanTxAuthorityRejectCount((uint8_t)ids[index]);
    }
    const uint32_t end = k_uptime_get_32();

    snprintk(line, sizeof(line),
             "OBS at=%u span=%u state=%s output=%u valid=%u,%u,%u,%u,%u "
             "input_age=%u sw=%u,%u ch=%d,%d,%d,%d\r\n",
             (unsigned)begin, (unsigned)(end - begin), RobotLifecycleName(lifecycle.state),
             (unsigned)lifecycle.output_allowed, (unsigned)gv, (unsigned)cv, (unsigned)sv,
             (unsigned)iv, (unsigned)mv, (unsigned)input.sourceAgeMs,
             (unsigned)input.manual.rc.s[0], (unsigned)input.manual.rc.s[1],
             (int)input.manual.rc.ch[0], (int)input.manual.rc.ch[1],
             (int)input.manual.rc.ch[2], (int)input.manual.rc.ch[3]);
    InfantryDiagWrite(line);
    if (gv != 0u) {
        /* y/p：目标角、实际角、目标角速度、实际角速度、速度环输出、附加补偿后的输出。
         * 角度统一用 mrad；这里的输出尚未乘电机方向，最终发送值看 OBS_M。 */
        snprintk(line, sizeof(line),
                 "OBS_G t=%u seq=%u mode=%u block=%u zero=%u imu_age=%u "
                 "y=%d,%d,%d,%d,%d,%d p=%d,%d,%d,%d,%d,%d\r\n",
                 (unsigned)gi.write_tick_ms, (unsigned)gi.seq, (unsigned)gimbal.feedback_mode,
                 (unsigned)gimbal.feedback_block_mask, (unsigned)gimbal.fault_hold_zero_mask,
                 (unsigned)gimbal.imu_age_ms,
                 (int)(gimbal.yaw.angle_set * 1000.0f), (int)(gimbal.yaw.angle * 1000.0f),
                 (int)(gimbal.yaw.motor_gyro_set * 1000.0f), (int)(gimbal.yaw.motor_gyro * 1000.0f),
                 (int)gimbal.yaw.gyro_pid.out, (int)gimbal.yaw.given_current,
                 (int)(gimbal.pitch.angle_set * 1000.0f), (int)(gimbal.pitch.angle * 1000.0f),
                 (int)(gimbal.pitch.motor_gyro_set * 1000.0f), (int)(gimbal.pitch.motor_gyro * 1000.0f),
                 (int)gimbal.pitch.gyro_pid.out, (int)gimbal.pitch.given_current);
        InfantryDiagWrite(line);
    }
    if (cv != 0u) {
        snprintk(line, sizeof(line),
                 "OBS_C t=%u seq=%u mode=%u wz=%d,%d err=%d pd=%d,%d zero=%u vset=%d,%d v=%d,%d\r\n",
                 (unsigned)ci.write_tick_ms, (unsigned)ci.seq, (unsigned)chassis.mode,
                 (int)(chassis.wz_set * 1000.0f), (int)(chassis.wz * 1000.0f),
                 (int)(chassis.angle_pid.error[0] * 1000.0f),
                 (int)(chassis.angle_pid.Pout * 1000.0f), (int)(chassis.angle_pid.Dout * 1000.0f),
                 (unsigned)chassis.fault_hold_zero_mask,
                 (int)(chassis.vx_set * 1000.0f), (int)(chassis.vy_set * 1000.0f),
                 (int)(chassis.vx * 1000.0f), (int)(chassis.vy * 1000.0f));
        InfantryDiagWrite(line);
    }
    if (sv != 0u) {
        /* f0/f1：目标 RPM、反馈 RPM、输出；tr：目标/实际 mrad/s、输出。 */
        snprintk(line, sizeof(line),
                 "OBS_S t=%u seq=%u mode=%u zero=%u block=%u "
                 "f0=%d,%d,%d f1=%d,%d,%d tr=%d,%d,%d\r\n",
                 (unsigned)si.write_tick_ms, (unsigned)si.seq, (unsigned)shoot.mode,
                 (unsigned)shoot.fault_hold_zero_mask, (unsigned)shoot.fault_blocking_mask,
                 (int)shoot.fric_speed_pid[0].set, (int)shoot.fric_speed_pid[0].fdb,
                 (int)shoot.fric_current_set[0],
                 (int)shoot.fric_speed_pid[1].set, (int)shoot.fric_speed_pid[1].fdb,
                 (int)shoot.fric_current_set[1],
                 (int)(shoot.speed_set * 1000.0f), (int)(shoot.speed * 1000.0f),
                 (int)shoot.given_current);
        InfantryDiagWrite(line);
    }
    if (mv != 0u) {
        for (uint8_t index = 0u; index < count; index++) {
            /* fb：编码器/RPM/反馈电流/收帧时刻；cmd：输出/时刻；tx：输出/完成时刻/回执有效。 */
            snprintk(line, sizeof(line),
                     "OBS_M id=%u fb=%u,%d,%d,%u cmd=%d,%u tx=%d,%u,%u app=%d,%u,%u reject=%u\r\n",
                     (unsigned)ids[index], (unsigned)motors[index].ecd,
                     (int)motors[index].speedRpm, (int)motors[index].current,
                     (unsigned)motors[index].lastRxTick,
                     (int)commands[index].current, (unsigned)commands[index].tick,
                     (int)receipts[index].current, (unsigned)receipts[index].completedTick,
                     (unsigned)receipts[index].valid,
                     (int)applied[index].current, (unsigned)applied[index].flags,
                     (unsigned)applied[index].tick, (unsigned)rejects[index]);
            InfantryDiagWrite(line);
        }
    }
}

static void InfantryDiagRun(void *a, void *b, void *c)
{
    ARG_UNUSED(a);
    ARG_UNUSED(b);
    ARG_UNUSED(c);

    uint32_t lastReport = 0u;
    uint8_t lastOutput = UINT8_MAX;
    for (;;) {
        ManualInputSnapshot input = {0};
        RobotLifecycleSnapshot lifecycle = {0};
        const uint32_t now = k_uptime_get_32();
        char line[192];

        if (device_is_ready(InfantryDiagUart) == 0) {
            k_msleep(1000);
            continue;
        }

        (void)RobotLifecycleGetSnapshot(&lifecycle);
        const uint8_t report = (uint8_t)(lastOutput != lifecycle.output_allowed ||
                                         (now - lastReport) >= 3000u);
        lastOutput = lifecycle.output_allowed;
        if (report == 0u) {
            if (lifecycle.output_allowed != 0u) {
                InfantryDiagMotionReport();
            }
            k_msleep(100);
            continue;
        }
        lastReport = now;
        snprintk(line, sizeof(line),
                 "INFANTRY-TIAN-A uptime=%u run_mode=%u state=%s reason=%u output=%u task=%u variant=%u\r\n",
                 (unsigned)now, (unsigned)g_config.operation.mode,
                 RobotLifecycleName(lifecycle.state), (unsigned)lifecycle.reason,
                 (unsigned)lifecycle.output_allowed,
                 (unsigned)g_config.operation.target_task, (unsigned)g_config.operation.variant);
        InfantryDiagWrite(line);

        if (ManualInputSnapshotRead(&input) != 0u) {
            snprintk(line, sizeof(line),
                     "INPUT online=%u source=%s(%u) age=%u sw=%u,%u ch=%d,%d,%d,%d\r\n",
                     (unsigned)input.online, InfantryDiagInputSourceName(input.activeSource),
                     (unsigned)input.activeSource, (unsigned)input.sourceAgeMs,
                     (unsigned)input.manual.rc.s[0], (unsigned)input.manual.rc.s[1],
                     (int)input.manual.rc.ch[0], (int)input.manual.rc.ch[1],
                     (int)input.manual.rc.ch[2], (int)input.manual.rc.ch[3]);
        }
        else {
            snprintk(line, sizeof(line), "INPUT snapshot=unavailable\r\n");
        }
        InfantryDiagWrite(line);

        InfantryDiagCanReport(1u);
        InfantryDiagCanReport(2u);
        if (lifecycle.output_allowed == 0u) {
            InfantryDiagChassisReport();
            InfantryDiagGimbalReport();
            InfantryDiagMotorsReport();
            InfantryDiagTimingReport();
        }
        else {
            InfantryDiagMotionReport();
        }
        k_msleep(100);
    }
}

K_THREAD_DEFINE(InfantryDiagThread, 3072, InfantryDiagRun, NULL, NULL, NULL, 14, 0, 1500);
