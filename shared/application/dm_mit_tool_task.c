/*
 * SPDX-License-Identifier: PolyForm-Noncommercial-1.0.0
 */

#include "dm_mit_tool_task.h"

#include <stdio.h>
#include <string.h>

#include "FreeRTOS.h"
#include "task.h"

#include "bsp_buzzer.h"
#include "bsp_can.h"
#include "bsp_usart.h"
#include "can_mit_motor_driver.h"
#include "manual_input.h"

#if defined(DM_MIT_TOOL_PROJECT) || defined(DM_MIT_TOOL_HAS_CONFIG)
#include "dm_mit_tool_config.h"
#endif

#define DM_MIT_TOOL_CMD_READ_PARAM  0x33u
#define DM_MIT_TOOL_CMD_WRITE_PARAM 0x55u
#define DM_MIT_TOOL_CMD_SAVE_PARAM  0xAAu
#define DM_MIT_TOOL_PARAM_CAN_ID    8u
#define DM_MIT_TOOL_PARAM_MASTER_ID 7u
#define DM_MIT_TOOL_PARAM_CAN_BR    35u
#define DM_MIT_TOOL_PARAM_CAN_STDID 0x7FFu
#define DM_MIT_TOOL_WRITE_MAGIC     0xA5u
#define DM_MIT_TOOL_CAN_BR_1M       4u

#ifndef DM_MIT_TOOL_BUS
#define DM_MIT_TOOL_BUS 1u
#endif

#ifndef DM_MIT_TOOL_SCAN_MIN_ID
#define DM_MIT_TOOL_SCAN_MIN_ID 1u
#endif

#ifndef DM_MIT_TOOL_SCAN_MAX_ID
#define DM_MIT_TOOL_SCAN_MAX_ID 15u
#endif

#ifndef DM_MIT_TOOL_TARGET_COMMAND_ID
#define DM_MIT_TOOL_TARGET_COMMAND_ID 1u
#endif

#ifndef DM_MIT_TOOL_TARGET_MASTER_ID
#define DM_MIT_TOOL_TARGET_MASTER_ID 0x11u
#endif

#ifndef DM_MIT_TOOL_AUTO_WRITE
#define DM_MIT_TOOL_AUTO_WRITE 0u
#endif

#ifndef DM_MIT_TOOL_WRITE_ARM
#define DM_MIT_TOOL_WRITE_ARM 0u
#endif

#ifndef DM_MIT_TOOL_MODEL
#define DM_MIT_TOOL_MODEL 4310u
#endif

#ifndef DM_MIT_TOOL_TEST_ENABLE
#define DM_MIT_TOOL_TEST_ENABLE 1u
#endif

#ifndef DM_MIT_TOOL_REBOOT_AFTER_WRITE
#define DM_MIT_TOOL_REBOOT_AFTER_WRITE 1u
#endif

#ifndef DM_MIT_TOOL_BEEP_ENABLE
#define DM_MIT_TOOL_BEEP_ENABLE 1u
#endif

#ifndef DM_MIT_TOOL_BEEP_VOLUME
#define DM_MIT_TOOL_BEEP_VOLUME 160u
#endif

#ifndef DM_MIT_TOOL_RC_TIMEOUT_MS
#define DM_MIT_TOOL_RC_TIMEOUT_MS 200u
#endif

#ifndef DM_MIT_TOOL_RC_UNLOCK_S0
#define DM_MIT_TOOL_RC_UNLOCK_S0 RC_SW_UP
#endif

#ifndef DM_MIT_TOOL_RC_UNLOCK_S1
#define DM_MIT_TOOL_RC_UNLOCK_S1 RC_SW_UP
#endif

#ifndef DM_MIT_TOOL_SPEED_CH
#define DM_MIT_TOOL_SPEED_CH 1u
#endif

#ifndef DM_MIT_TOOL_TORQUE_CH
#define DM_MIT_TOOL_TORQUE_CH 3u
#endif

#ifndef DM_MIT_TOOL_MAX_SPEED_RAD_S
#define DM_MIT_TOOL_MAX_SPEED_RAD_S 0.8f
#endif

#ifndef DM_MIT_TOOL_MAX_TORQUE_NM
#define DM_MIT_TOOL_MAX_TORQUE_NM 0.5f
#endif

#ifndef DM_MIT_TOOL_RC_DEADBAND
#define DM_MIT_TOOL_RC_DEADBAND 20
#endif

#ifndef DM_MIT_TOOL_REQUIRE_FEEDBACK
#define DM_MIT_TOOL_REQUIRE_FEEDBACK 1u
#endif

#ifndef DM_MIT_TOOL_SCAN_ENABLE_FALLBACK
#define DM_MIT_TOOL_SCAN_ENABLE_FALLBACK 1u
#endif

#ifndef DM_MIT_TOOL_RX_TIMEOUT_MS
#define DM_MIT_TOOL_RX_TIMEOUT_MS 150u
#endif

#ifndef DM_MIT_TOOL_BLIND_RECOVER_ENABLE
#define DM_MIT_TOOL_BLIND_RECOVER_ENABLE 0u
#endif

#ifndef DM_MIT_TOOL_BLIND_RECOVER_ONLY
#define DM_MIT_TOOL_BLIND_RECOVER_ONLY 0u
#endif

#ifndef DM_MIT_TOOL_BLIND_RECOVER_REPEAT
#define DM_MIT_TOOL_BLIND_RECOVER_REPEAT 3u
#endif

#ifndef DM_MIT_TOOL_BLIND_RECOVER_SAVE_DELAY_MS
#define DM_MIT_TOOL_BLIND_RECOVER_SAVE_DELAY_MS 40u
#endif

#ifndef DM_MIT_TOOL_AUTO_FD_ENABLE
#define DM_MIT_TOOL_AUTO_FD_ENABLE 0u
#endif

#ifndef DM_MIT_TOOL_OWN_CAN_EXTRA_HOOK
#define DM_MIT_TOOL_OWN_CAN_EXTRA_HOOK 1u
#endif

#define DM_MIT_TOOL_SCAN_MIN_CLAMPED ((DM_MIT_TOOL_SCAN_MIN_ID < 1u) ? 1u : DM_MIT_TOOL_SCAN_MIN_ID)
#define DM_MIT_TOOL_SCAN_MAX_CLAMPED ((DM_MIT_TOOL_SCAN_MAX_ID > 0x7FEu) ? 0x7FEu : DM_MIT_TOOL_SCAN_MAX_ID)

typedef struct
{
    uint32_t seq;
    uint8_t bus;
    uint8_t cmd;
    uint8_t rid;
    uint16_t slave_id;
    uint32_t value;
} dm_mit_param_rsp_t;

typedef enum
{
    DM_TOOL_ID_RESULT_NOT_ARMED = 0u,
    DM_TOOL_ID_RESULT_ALREADY_TARGET,
    DM_TOOL_ID_RESULT_CHANGED,
    DM_TOOL_ID_RESULT_FAILED,
} dm_tool_id_result_e;

typedef struct
{
    uint32_t data_bitrate;
} dm_tool_fd_candidate_t;

#if (DM_MIT_TOOL_MODEL == 6215u)
static const can_mit_motor_limits_t k_dm6215_limits =
{
    12.0f,
    45.0f,
    500.0f,
    5.0f,
    18.0f,
};
#elif (DM_MIT_TOOL_MODEL == 3510u)
static const can_mit_motor_limits_t k_dmh3510_limits =
{
    12.5f,
    100.0f,
    500.0f,
    5.0f,
    0.45f,
};
#else
static const can_mit_motor_limits_t k_dm4310_limits =
{
    12.5f,
    30.0f,
    500.0f,
    5.0f,
    10.0f,
};
#endif

static dm_mit_tool_state_t g_dm_mit_tool_state;
static volatile dm_mit_param_rsp_t g_param_rsp;
static can_mit_motor_feedback_t g_feedback;
static uint32_t g_last_sbus_count;
static uint32_t g_last_sbus_tick_ms;

static uint32_t dm_tool_now_ms(void)
{
    return (uint32_t)(xTaskGetTickCount() * portTICK_PERIOD_MS);
}

static const can_mit_motor_limits_t *dm_tool_limits(void)
{
#if (DM_MIT_TOOL_MODEL == 6215u)
    return &k_dm6215_limits;
#elif (DM_MIT_TOOL_MODEL == 3510u)
    return &k_dmh3510_limits;
#else
    return &k_dm4310_limits;
#endif
}

static void dm_tool_log(const char *text)
{
    if (text == NULL || bsp_aux_link_tx_ready() == 0u)
    {
        return;
    }
    (void)bsp_aux_link_tx_dma((const uint8_t *)text, (uint16_t)strlen(text));
}

static void dm_tool_logf(const char *fmt, uint32_t a, uint32_t b, uint32_t c)
{
    char buf[96];
    int n;

    if (fmt == NULL || bsp_aux_link_tx_ready() == 0u)
    {
        return;
    }
    n = snprintf(buf, sizeof(buf), fmt, (unsigned long)a, (unsigned long)b, (unsigned long)c);
    if (n <= 0)
    {
        return;
    }
    if (n > (int)sizeof(buf))
    {
        n = (int)sizeof(buf);
    }
    (void)bsp_aux_link_tx_dma((const uint8_t *)buf, (uint16_t)n);
}

static void dm_tool_beep_tone(uint32_t freq_hz, uint16_t duration_ms, uint16_t gap_ms)
{
#if (DM_MIT_TOOL_BEEP_ENABLE != 0u)
    buzzer_set_enable(1u);
    (void)buzzer_tone_start_hz(freq_hz, (uint8_t)DM_MIT_TOOL_BEEP_VOLUME);
    vTaskDelay(pdMS_TO_TICKS(duration_ms));
    buzzer_tone_stop();
    if (gap_ms != 0u)
    {
        vTaskDelay(pdMS_TO_TICKS(gap_ms));
    }
#else
    (void)freq_hz;
    (void)duration_ms;
    (void)gap_ms;
#endif
}

static void dm_tool_beep_id_success(void)
{
    dm_tool_beep_tone(1400u, 90u, 60u);
    dm_tool_beep_tone(2200u, 140u, 0u);
}

static void dm_tool_beep_id_failed(void)
{
    dm_tool_beep_tone(500u, 120u, 80u);
    dm_tool_beep_tone(500u, 120u, 80u);
    dm_tool_beep_tone(500u, 220u, 0u);
}

static void dm_tool_beep_id_already_target(void)
{
    dm_tool_beep_tone(1200u, 220u, 0u);
}

static uint8_t dm_tool_bus(void)
{
    if (DM_MIT_TOOL_BUS == 2u)
    {
        return 2u;
    }
#if defined(HAL_FDCAN_MODULE_ENABLED)
    if (DM_MIT_TOOL_BUS == 3u)
    {
        return 3u;
    }
#endif
    return 1u;
}

static void dm_tool_send_param_frame(uint16_t slave_id, uint8_t cmd, uint8_t rid, uint32_t value)
{
    uint8_t data[8];

    data[0] = (uint8_t)(slave_id & 0xFFu);
    data[1] = (uint8_t)((slave_id >> 8) & 0xFFu);
    data[2] = cmd;
    data[3] = rid;
    data[4] = (uint8_t)(value & 0xFFu);
    data[5] = (uint8_t)((value >> 8) & 0xFFu);
    data[6] = (uint8_t)((value >> 16) & 0xFFu);
    data[7] = (uint8_t)((value >> 24) & 0xFFu);

    if (bsp_can_tx(dm_tool_bus(), (uint16_t)DM_MIT_TOOL_PARAM_CAN_STDID, data, 8u) == 0)
    {
        g_dm_mit_tool_state.tx_count++;
    }
}

static void dm_tool_send_enable(uint16_t command_id)
{
    can_mit_motor_send_enable(dm_tool_bus(), command_id);
    g_dm_mit_tool_state.tx_count++;
}

static void dm_tool_send_zero(uint16_t command_id)
{
    can_mit_motor_cmd_t cmd;

    (void)memset(&cmd, 0, sizeof(cmd));
    can_mit_motor_send_cmd(dm_tool_bus(), command_id, dm_tool_limits(), &cmd);
    g_dm_mit_tool_state.tx_count++;
}

static void dm_tool_send_disable(uint16_t command_id)
{
    static const uint8_t data[8] = {0xFFu, 0xFFu, 0xFFu, 0xFFu, 0xFFu, 0xFFu, 0xFFu, 0xFDu};

    if (command_id == 0u)
    {
        return;
    }
    if (bsp_can_tx(dm_tool_bus(), command_id, data, 8u) == 0)
    {
        g_dm_mit_tool_state.tx_count++;
    }
}

static int16_t dm_tool_abs_i16(int16_t x)
{
    if (x == (int16_t)-32768)
    {
        return 32767;
    }
    return (x < 0) ? (int16_t)(-x) : x;
}

static int16_t dm_tool_rc_channel(const manual_input_state_t *rc, uint8_t index)
{
    if (rc == NULL || index >= 5u)
    {
        return 0;
    }
    return rc->rc.ch[index];
}

static fp32 dm_tool_scale_rc_axis(int16_t raw, fp32 max_abs)
{
    int32_t value = raw;
    int32_t deadband = (int32_t)DM_MIT_TOOL_RC_DEADBAND;
    const int32_t input_abs = (int32_t)RC_CH_VALUE_ABS_LEGACY;

    if (max_abs <= 0.0f || input_abs <= 0)
    {
        return 0.0f;
    }
    if (deadband < 0)
    {
        deadband = 0;
    }
    if (deadband >= input_abs)
    {
        return 0.0f;
    }
    if (value > input_abs)
    {
        value = input_abs;
    }
    else if (value < -input_abs)
    {
        value = -input_abs;
    }

    if (dm_tool_abs_i16((int16_t)value) <= (int16_t)deadband)
    {
        return 0.0f;
    }
    if (value > 0)
    {
        value -= deadband;
    }
    else
    {
        value += deadband;
    }

    return ((fp32)value * max_abs) / (fp32)(input_abs - deadband);
}

static uint8_t dm_tool_switch_match(uint8_t actual, uint16_t expected)
{
    if (expected == 0u)
    {
        return 1u;
    }
    return ((uint16_t)actual == expected) ? 1u : 0u;
}

static uint8_t dm_tool_rc_online(uint32_t now_ms)
{
    const uint32_t sbus_count = manual_input_get_sbus_frame_count();

    if (sbus_count != g_last_sbus_count)
    {
        g_last_sbus_count = sbus_count;
        g_last_sbus_tick_ms = now_ms;
    }

    g_dm_mit_tool_state.last_rc_tick_ms = g_last_sbus_tick_ms;
    if (sbus_count == 0u || g_last_sbus_tick_ms == 0u)
    {
        return 0u;
    }

    return ((uint32_t)(now_ms - g_last_sbus_tick_ms) <= (uint32_t)DM_MIT_TOOL_RC_TIMEOUT_MS) ? 1u : 0u;
}

static uint8_t dm_tool_feedback_fresh(uint32_t now_ms)
{
    if (g_dm_mit_tool_state.feedback_online == 0u || g_dm_mit_tool_state.last_rx_tick_ms == 0u)
    {
        return 0u;
    }
    if ((uint32_t)(now_ms - g_dm_mit_tool_state.last_rx_tick_ms) > (uint32_t)DM_MIT_TOOL_RX_TIMEOUT_MS)
    {
        g_dm_mit_tool_state.feedback_online = 0u;
        return 0u;
    }
    return 1u;
}

static uint8_t dm_tool_update_test_inputs(uint32_t now_ms, fp32 *velocity, fp32 *torque)
{
    const manual_input_state_t *rc;
    uint8_t arm_allowed;
    uint8_t drive_allowed;
    uint8_t feedback_ok;
    int16_t speed_raw;
    int16_t torque_raw;

    manual_input_refresh();
    rc = get_remote_control_point();

    speed_raw = dm_tool_rc_channel(rc, (uint8_t)DM_MIT_TOOL_SPEED_CH);
    torque_raw = dm_tool_rc_channel(rc, (uint8_t)DM_MIT_TOOL_TORQUE_CH);
    g_dm_mit_tool_state.speed_ch = speed_raw;
    g_dm_mit_tool_state.torque_ch = torque_raw;
    g_dm_mit_tool_state.rc_online = dm_tool_rc_online(now_ms);
    g_dm_mit_tool_state.rc_unlocked = 0u;

    if (rc != NULL && g_dm_mit_tool_state.rc_online != 0u &&
        dm_tool_switch_match((uint8_t)rc->rc.s[0], (uint16_t)DM_MIT_TOOL_RC_UNLOCK_S0) != 0u &&
        dm_tool_switch_match((uint8_t)rc->rc.s[1], (uint16_t)DM_MIT_TOOL_RC_UNLOCK_S1) != 0u)
    {
        g_dm_mit_tool_state.rc_unlocked = 1u;
    }

    feedback_ok = dm_tool_feedback_fresh(now_ms);
    arm_allowed = (g_dm_mit_tool_state.rc_online != 0u &&
                   g_dm_mit_tool_state.rc_unlocked != 0u) ? 1u : 0u;
    drive_allowed = arm_allowed;
#if (DM_MIT_TOOL_REQUIRE_FEEDBACK != 0u)
    if (feedback_ok == 0u)
    {
        drive_allowed = 0u;
    }
#endif

    if (velocity != NULL)
    {
        *velocity = (drive_allowed != 0u) ?
                        dm_tool_scale_rc_axis(speed_raw, (fp32)DM_MIT_TOOL_MAX_SPEED_RAD_S) :
                        0.0f;
    }
    if (torque != NULL)
    {
        *torque = (drive_allowed != 0u) ?
                      dm_tool_scale_rc_axis(torque_raw, (fp32)DM_MIT_TOOL_MAX_TORQUE_NM) :
                      0.0f;
    }

    g_dm_mit_tool_state.key_down = g_dm_mit_tool_state.rc_unlocked;
    g_dm_mit_tool_state.drive_allowed = drive_allowed;
    return arm_allowed;
}

static void dm_tool_send_test_cmd(uint16_t command_id, fp32 velocity, fp32 torque)
{
    can_mit_motor_cmd_t cmd;

    (void)memset(&cmd, 0, sizeof(cmd));
    cmd.velocity = velocity;
    cmd.torque = torque;

    g_dm_mit_tool_state.cmd_velocity = cmd.velocity;
    g_dm_mit_tool_state.cmd_torque = cmd.torque;
    can_mit_motor_send_cmd(dm_tool_bus(), command_id, dm_tool_limits(), &cmd);
    g_dm_mit_tool_state.tx_count++;
}

static uint8_t dm_tool_param_matches(uint32_t start_seq,
                                     uint8_t cmd,
                                     uint8_t rid,
                                     uint16_t slave_id,
                                     dm_mit_param_rsp_t *out)
{
    dm_mit_param_rsp_t rsp;

    rsp.seq = g_param_rsp.seq;
    if (rsp.seq == start_seq)
    {
        return 0u;
    }
    rsp.bus = g_param_rsp.bus;
    rsp.cmd = g_param_rsp.cmd;
    rsp.rid = g_param_rsp.rid;
    rsp.slave_id = g_param_rsp.slave_id;
    rsp.value = g_param_rsp.value;

    if (rsp.bus != dm_tool_bus() || rsp.cmd != cmd || rsp.rid != rid || rsp.slave_id != slave_id)
    {
        return 0u;
    }
    if (out != NULL)
    {
        *out = rsp;
    }
    return 1u;
}

static void dm_tool_handle_param_rsp(uint8_t bus, const uint8_t data[8])
{
    const uint16_t slave_id = (uint16_t)data[0] | ((uint16_t)data[1] << 8);
    const uint32_t value = (uint32_t)data[4] |
                           ((uint32_t)data[5] << 8) |
                           ((uint32_t)data[6] << 16) |
                           ((uint32_t)data[7] << 24);

    g_param_rsp.bus = bus;
    g_param_rsp.cmd = data[2];
    g_param_rsp.rid = data[3];
    g_param_rsp.slave_id = slave_id;
    g_param_rsp.value = value;
    g_param_rsp.seq++;

    g_dm_mit_tool_state.param_rx_count++;
    g_dm_mit_tool_state.last_rx_tick_ms = dm_tool_now_ms();
}

static uint8_t dm_tool_handle_can_frame(const bsp_can_frame_t *f)
{
    uint8_t motor_id;

    if (f == NULL || f->bus != dm_tool_bus())
    {
        return 0u;
    }

    g_dm_mit_tool_state.last_rx_flags = f->flags;
    if ((f->flags & BSP_CAN_FLAG_FD) != 0u)
    {
        g_dm_mit_tool_state.can_fd_seen = 1u;
    }
    if ((f->flags & BSP_CAN_FLAG_BRS) != 0u)
    {
        g_dm_mit_tool_state.can_brs_seen = 1u;
    }

    if (f->dlc == 8u &&
        (f->data[2] == DM_MIT_TOOL_CMD_READ_PARAM || f->data[2] == DM_MIT_TOOL_CMD_WRITE_PARAM))
    {
        dm_tool_handle_param_rsp(f->bus, f->data);
        return 1u;
    }

    if (f->dlc != 8u)
    {
        return 0u;
    }

    motor_id = (uint8_t)(f->data[0] & 0x0Fu);
    if (motor_id == 0u)
    {
        return 0u;
    }

    if (can_mit_motor_update_feedback(f->std_id,
                                      motor_id,
                                      dm_tool_limits(),
                                      f->dlc,
                                      f->data,
                                      &g_feedback) != 0u)
    {
        g_dm_mit_tool_state.feedback_online = 1u;
        g_dm_mit_tool_state.feedback_state = g_feedback.state;
        g_dm_mit_tool_state.last_rx_id = f->std_id;
        g_dm_mit_tool_state.rx_count = g_feedback.rx_count;
        g_dm_mit_tool_state.last_rx_tick_ms = g_feedback.last_rx_tick;
        g_dm_mit_tool_state.feedback_position = g_feedback.position;
        g_dm_mit_tool_state.feedback_velocity = g_feedback.velocity;
        g_dm_mit_tool_state.feedback_torque = g_feedback.torque;
        return 1u;
    }

    return 0u;
}

static void dm_tool_poll_can(void)
{
    bsp_can_frame_t f;

    while (bsp_can_rx_pop(&f) != 0)
    {
        (void)dm_tool_handle_can_frame(&f);
    }
}

static uint8_t dm_tool_wait_param(uint16_t slave_id,
                                  uint8_t cmd,
                                  uint8_t rid,
                                  uint16_t timeout_ms,
                                  dm_mit_param_rsp_t *out)
{
    const uint32_t start_ms = dm_tool_now_ms();
    const uint32_t start_seq = g_param_rsp.seq;

    while ((uint32_t)(dm_tool_now_ms() - start_ms) < (uint32_t)timeout_ms)
    {
        dm_tool_poll_can();
        if (dm_tool_param_matches(start_seq, cmd, rid, slave_id, out) != 0u)
        {
            return 1u;
        }
        (void)ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(2u));
    }

    return 0u;
}

static uint8_t dm_tool_read_param(uint16_t slave_id,
                                  uint8_t rid,
                                  uint16_t timeout_ms,
                                  dm_mit_param_rsp_t *out)
{
    dm_tool_send_param_frame(slave_id, DM_MIT_TOOL_CMD_READ_PARAM, rid, 0u);
    return dm_tool_wait_param(slave_id, DM_MIT_TOOL_CMD_READ_PARAM, rid, timeout_ms, out);
}

static uint8_t dm_tool_write_param(uint16_t slave_id,
                                   uint8_t rid,
                                   uint32_t value,
                                   uint16_t timeout_ms)
{
    dm_mit_param_rsp_t rsp;

    dm_tool_send_param_frame(slave_id, DM_MIT_TOOL_CMD_WRITE_PARAM, rid, value);
    if (dm_tool_wait_param(slave_id, DM_MIT_TOOL_CMD_WRITE_PARAM, rid, timeout_ms, &rsp) == 0u)
    {
        return 0u;
    }
    return (rsp.value == value) ? 1u : 0u;
}

static void dm_tool_save_param(uint16_t slave_id)
{
    dm_tool_send_param_frame(slave_id, DM_MIT_TOOL_CMD_SAVE_PARAM, 1u, 0u);
}

static void dm_tool_blind_recover_can_1m(void)
{
#if (DM_MIT_TOOL_BLIND_RECOVER_ENABLE != 0u)
    uint16_t id;
    uint16_t repeat;
    const uint16_t min_id = (uint16_t)DM_MIT_TOOL_SCAN_MIN_CLAMPED;
    const uint16_t max_id = (uint16_t)DM_MIT_TOOL_SCAN_MAX_CLAMPED;

    g_dm_mit_tool_state.phase = (uint8_t)DM_MIT_TOOL_PHASE_SAVE;
    g_dm_mit_tool_state.blind_recover_done = 1u;
    dm_tool_log("[dm-tool] blind recover: write can_br=4 and save\r\n");

    for (repeat = 0u; repeat < (uint16_t)DM_MIT_TOOL_BLIND_RECOVER_REPEAT; repeat++)
    {
        for (id = min_id; id <= max_id; id++)
        {
            g_dm_mit_tool_state.scan_id = (uint8_t)(id & 0xFFu);
            dm_tool_send_disable(id);
            vTaskDelay(pdMS_TO_TICKS(2u));
            dm_tool_send_param_frame(id,
                                     DM_MIT_TOOL_CMD_WRITE_PARAM,
                                     DM_MIT_TOOL_PARAM_CAN_BR,
                                     (uint32_t)DM_MIT_TOOL_CAN_BR_1M);
            vTaskDelay(pdMS_TO_TICKS(2u));
            dm_tool_save_param(id);
            vTaskDelay(pdMS_TO_TICKS(DM_MIT_TOOL_BLIND_RECOVER_SAVE_DELAY_MS));
        }
    }

    dm_tool_log("[dm-tool] blind recover sent; power-cycle motor and board\r\n");
#else
    dm_tool_log("[dm-tool] blind recover disabled\r\n");
#endif
}

static uint8_t dm_tool_scan_by_param(uint16_t slave_id,
                                     uint16_t *out_command_id,
                                     uint16_t *out_master_id)
{
    dm_mit_param_rsp_t rsp;

    if (dm_tool_read_param(slave_id, DM_MIT_TOOL_PARAM_CAN_ID, 30u, &rsp) == 0u)
    {
        return 0u;
    }

    if (out_command_id != NULL)
    {
        *out_command_id = (rsp.value <= 0x7FEu) ? (uint16_t)rsp.value : slave_id;
    }
    if (out_master_id != NULL)
    {
        *out_master_id = 0u;
        if (dm_tool_read_param(slave_id, DM_MIT_TOOL_PARAM_MASTER_ID, 30u, &rsp) != 0u &&
            rsp.value <= 0x7FEu)
        {
            *out_master_id = (uint16_t)rsp.value;
        }
    }

    return 1u;
}

static uint8_t dm_tool_scan_by_enable(uint16_t slave_id,
                                      uint16_t *out_command_id,
                                      uint16_t *out_master_id)
{
    const uint32_t start_rx = g_dm_mit_tool_state.rx_count;
    const uint32_t start_ms = dm_tool_now_ms();

    dm_tool_send_enable(slave_id);
    while ((uint32_t)(dm_tool_now_ms() - start_ms) < 20u)
    {
        dm_tool_poll_can();
        if (g_dm_mit_tool_state.rx_count != start_rx &&
            (uint16_t)g_feedback.motor_id == slave_id)
        {
            dm_tool_send_zero(slave_id);
            if (out_command_id != NULL)
            {
                *out_command_id = slave_id;
            }
            if (out_master_id != NULL)
            {
                *out_master_id = g_dm_mit_tool_state.last_rx_id;
            }
            return 1u;
        }
        vTaskDelay(pdMS_TO_TICKS(2u));
    }
    dm_tool_send_zero(slave_id);
    return 0u;
}

static void dm_tool_discard_can(void)
{
    bsp_can_frame_t f;

    while (bsp_can_rx_pop(&f) != 0)
    {
    }
}

static uint8_t dm_tool_scan_once(uint16_t *out_command_id, uint16_t *out_master_id)
{
    uint16_t id;
    const uint16_t min_id = (uint16_t)DM_MIT_TOOL_SCAN_MIN_CLAMPED;
    const uint16_t max_id = (uint16_t)DM_MIT_TOOL_SCAN_MAX_CLAMPED;

    for (id = min_id; id <= max_id; id++)
    {
        g_dm_mit_tool_state.scan_id = (uint8_t)(id & 0xFFu);
        dm_tool_logf("[dm-tool] scan id=%lu bus=%lu\r\n", id, dm_tool_bus(), 0u);

        if (dm_tool_scan_by_param(id, out_command_id, out_master_id) != 0u)
        {
            return 1u;
        }

#if (DM_MIT_TOOL_SCAN_ENABLE_FALLBACK != 0u)
        if (dm_tool_scan_by_enable(id, out_command_id, out_master_id) != 0u)
        {
            return 1u;
        }
#endif
    }

    return 0u;
}

static uint8_t dm_tool_scan(uint16_t *out_command_id, uint16_t *out_master_id)
{
#if (DM_MIT_TOOL_AUTO_FD_ENABLE != 0u) && defined(HAL_FDCAN_MODULE_ENABLED)
    static const dm_tool_fd_candidate_t candidates[] =
    {
        {5000000u},
        {4000000u},
        {3200000u},
        {2500000u},
        {2000000u},
        {1000000u},
    };
    uint8_t i;

    for (i = 0u; i < (uint8_t)(sizeof(candidates) / sizeof(candidates[0])); i++)
    {
        g_dm_mit_tool_state.fd_data_bitrate = candidates[i].data_bitrate;
        (void)bsp_can_fd_set_data_bitrate(dm_tool_bus(), candidates[i].data_bitrate);
        dm_tool_discard_can();
        dm_tool_logf("[dm-tool] fd data scan=%lu bus=%lu\r\n",
                     candidates[i].data_bitrate,
                     dm_tool_bus(),
                     0u);
        if (dm_tool_scan_once(out_command_id, out_master_id) != 0u)
        {
            return 1u;
        }
    }
    return 0u;
#else
    g_dm_mit_tool_state.fd_data_bitrate = 0u;
    return dm_tool_scan_once(out_command_id, out_master_id);
#endif
}

static dm_tool_id_result_e dm_tool_write_ids(uint16_t current_command_id, uint16_t current_master_id)
{
    const uint16_t original_command_id = current_command_id;
    const uint16_t target_command_id = (uint16_t)DM_MIT_TOOL_TARGET_COMMAND_ID;
    const uint16_t target_master_id = (uint16_t)DM_MIT_TOOL_TARGET_MASTER_ID;
    uint8_t changed = 0u;
    uint8_t failed = 0u;

    g_dm_mit_tool_state.write_armed =
        (DM_MIT_TOOL_AUTO_WRITE != 0u && DM_MIT_TOOL_WRITE_ARM == DM_MIT_TOOL_WRITE_MAGIC) ? 1u : 0u;
    if (g_dm_mit_tool_state.write_armed == 0u)
    {
        if ((target_command_id == 0u || target_command_id == current_command_id) &&
            (target_master_id == 0u || target_master_id == current_master_id))
        {
            dm_tool_log("[dm-tool] id already matches; write not armed\r\n");
            return DM_TOOL_ID_RESULT_ALREADY_TARGET;
        }
        dm_tool_log("[dm-tool] id write not armed; skip write\r\n");
        return DM_TOOL_ID_RESULT_NOT_ARMED;
    }

    if (target_master_id != 0u && target_master_id != current_master_id)
    {
        g_dm_mit_tool_state.phase = (uint8_t)DM_MIT_TOOL_PHASE_WRITE_MASTER_ID;
        dm_tool_logf("[dm-tool] write master id: %lu -> %lu\r\n",
                     current_master_id,
                     target_master_id,
                     0u);
        if (dm_tool_write_param(current_command_id, DM_MIT_TOOL_PARAM_MASTER_ID, target_master_id, 80u) == 0u)
        {
            g_dm_mit_tool_state.last_error = 1u;
            failed = 1u;
        }
        else
        {
            current_master_id = target_master_id;
            changed = 1u;
        }
    }

    if (target_command_id != 0u && target_command_id != current_command_id)
    {
        g_dm_mit_tool_state.phase = (uint8_t)DM_MIT_TOOL_PHASE_WRITE_CAN_ID;
        dm_tool_logf("[dm-tool] write can id: %lu -> %lu\r\n",
                     current_command_id,
                     target_command_id,
                     0u);
        if (dm_tool_write_param(current_command_id, DM_MIT_TOOL_PARAM_CAN_ID, target_command_id, 80u) == 0u)
        {
            g_dm_mit_tool_state.last_error = 2u;
            failed = 1u;
        }
        else
        {
            current_command_id = target_command_id;
            changed = 1u;
        }
    }

    if (changed != 0u)
    {
        g_dm_mit_tool_state.phase = (uint8_t)DM_MIT_TOOL_PHASE_SAVE;
        dm_tool_save_param(original_command_id);
        if (target_command_id != 0u && target_command_id != original_command_id)
        {
            dm_tool_save_param(target_command_id);
        }
        g_dm_mit_tool_state.write_done = 1u;
        dm_tool_log("[dm-tool] save sent; power-cycle motor and board before test\r\n");
    }
    else
    {
        dm_tool_log("[dm-tool] id already matches; skip write\r\n");
    }

    g_dm_mit_tool_state.active_command_id = current_command_id;
    g_dm_mit_tool_state.active_master_id = current_master_id;
    if (failed != 0u)
    {
        return DM_TOOL_ID_RESULT_FAILED;
    }
    if (changed != 0u)
    {
        return DM_TOOL_ID_RESULT_CHANGED;
    }
    return DM_TOOL_ID_RESULT_ALREADY_TARGET;
}

static void dm_tool_test_loop(void)
{
    uint8_t enable_sent = 0u;
    uint32_t last_tx_ms = 0u;

    g_dm_mit_tool_state.phase = (uint8_t)DM_MIT_TOOL_PHASE_TEST;
    dm_tool_log("[dm-tool] test mode: unlock RC switches; speed ch[1], torque ch[3]\r\n");

    for (;;)
    {
        const uint32_t now_ms = dm_tool_now_ms();
        fp32 velocity = 0.0f;
        fp32 torque = 0.0f;
        const uint8_t arm_allowed = dm_tool_update_test_inputs(now_ms, &velocity, &torque);

        dm_tool_poll_can();

        if (arm_allowed != 0u)
        {
            if (enable_sent < 5u)
            {
                dm_tool_send_enable(g_dm_mit_tool_state.active_command_id);
                enable_sent++;
            }
            g_dm_mit_tool_state.motor_enabled = 1u;
        }
        else
        {
            enable_sent = 0u;
            g_dm_mit_tool_state.motor_enabled = 0u;
        }

        if ((uint32_t)(now_ms - last_tx_ms) >= 5u)
        {
            dm_tool_send_test_cmd(g_dm_mit_tool_state.active_command_id, velocity, torque);
            last_tx_ms = now_ms;
        }

        (void)ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(2u));
    }
}

static void dm_tool_done_loop(void)
{
    g_dm_mit_tool_state.phase = (uint8_t)DM_MIT_TOOL_PHASE_DONE;
    g_dm_mit_tool_state.motor_enabled = 0u;
    g_dm_mit_tool_state.drive_allowed = 0u;
    g_dm_mit_tool_state.cmd_velocity = 0.0f;
    g_dm_mit_tool_state.cmd_torque = 0.0f;

    for (;;)
    {
        dm_tool_poll_can();
        if (g_dm_mit_tool_state.active_command_id != 0u)
        {
            dm_tool_send_zero(g_dm_mit_tool_state.active_command_id);
        }
        vTaskDelay(pdMS_TO_TICKS(20u));
    }
}

#if (DM_MIT_TOOL_OWN_CAN_EXTRA_HOOK != 0u)
uint8_t CAN_rx_process_extra_frame(uint8_t bus, uint16_t std_id, uint8_t dlc, const uint8_t data[8])
{
    bsp_can_frame_t f;

    if (data == NULL)
    {
        return 0u;
    }

    f.bus = bus;
    f.std_id = std_id;
    f.dlc = dlc;
    f.flags = 0u;
    (void)memcpy(f.data, data, sizeof(f.data));
    return dm_tool_handle_can_frame(&f);
}
#endif

void dm_mit_tool_task(void const *argument)
{
    uint16_t found_command_id = 0u;
    uint16_t found_master_id = 0u;
    dm_tool_id_result_e id_result = DM_TOOL_ID_RESULT_NOT_ARMED;

    (void)argument;
    (void)memset(&g_dm_mit_tool_state, 0, sizeof(g_dm_mit_tool_state));
    (void)memset(&g_feedback, 0, sizeof(g_feedback));

    g_dm_mit_tool_state.phase = (uint8_t)DM_MIT_TOOL_PHASE_BOOT;
    g_dm_mit_tool_state.bus = dm_tool_bus();
    g_dm_mit_tool_state.target_command_id = (uint16_t)DM_MIT_TOOL_TARGET_COMMAND_ID;
    g_dm_mit_tool_state.target_master_id = (uint16_t)DM_MIT_TOOL_TARGET_MASTER_ID;

    bsp_can_rx_attach_task(xTaskGetCurrentTaskHandle());
    vTaskDelay(pdMS_TO_TICKS(500u));

    dm_tool_log("[dm-tool] boot\r\n");

#if (DM_MIT_TOOL_BLIND_RECOVER_ONLY != 0u)
    dm_tool_blind_recover_can_1m();
    dm_tool_beep_id_success();
    dm_tool_done_loop();
#endif

    g_dm_mit_tool_state.phase = (uint8_t)DM_MIT_TOOL_PHASE_SCAN;

    if (dm_tool_scan(&found_command_id, &found_master_id) == 0u)
    {
#if (DM_MIT_TOOL_BLIND_RECOVER_ENABLE != 0u)
        dm_tool_log("[dm-tool] no motor found; try blind recover\r\n");
        dm_tool_blind_recover_can_1m();
        dm_tool_beep_id_success();
        dm_tool_done_loop();
#else
        g_dm_mit_tool_state.phase = (uint8_t)DM_MIT_TOOL_PHASE_ERROR;
        g_dm_mit_tool_state.last_error = 3u;
        dm_tool_log("[dm-tool] no motor found\r\n");
        dm_tool_beep_id_failed();
        for (;;)
        {
            dm_tool_poll_can();
            vTaskDelay(pdMS_TO_TICKS(20u));
        }
#endif
    }

    g_dm_mit_tool_state.found = 1u;
    g_dm_mit_tool_state.found_command_id = found_command_id;
    g_dm_mit_tool_state.found_master_id = found_master_id;
    g_dm_mit_tool_state.active_command_id = found_command_id;
    g_dm_mit_tool_state.active_master_id = found_master_id;
    g_dm_mit_tool_state.phase = (uint8_t)DM_MIT_TOOL_PHASE_FOUND;
    dm_tool_logf("[dm-tool] found command=%lu master=%lu\r\n",
                 found_command_id,
                 found_master_id,
                 0u);
    dm_tool_logf("[dm-tool] rx fd=%lu brs=%lu data=%lu\r\n",
                 g_dm_mit_tool_state.can_fd_seen,
                 g_dm_mit_tool_state.can_brs_seen,
                 g_dm_mit_tool_state.fd_data_bitrate);

    id_result = dm_tool_write_ids(found_command_id, found_master_id);
    if (id_result == DM_TOOL_ID_RESULT_CHANGED)
    {
        dm_tool_beep_id_success();
    }
    else if (id_result == DM_TOOL_ID_RESULT_FAILED)
    {
        dm_tool_beep_id_failed();
        dm_tool_done_loop();
    }
    else if (id_result == DM_TOOL_ID_RESULT_ALREADY_TARGET)
    {
        dm_tool_beep_id_already_target();
    }
    else
    {
        /* Not armed: keep using the scanned ID for a plain test run. */
    }

#if (DM_MIT_TOOL_REBOOT_AFTER_WRITE != 0u)
    if (g_dm_mit_tool_state.write_done != 0u)
    {
        dm_tool_log("[dm-tool] id changed; power-cycle motor and board before test\r\n");
        dm_tool_done_loop();
    }
#endif

#if (DM_MIT_TOOL_TEST_ENABLE != 0u)
    dm_tool_test_loop();
#else
    dm_tool_done_loop();
#endif
}

const dm_mit_tool_state_t *dm_mit_tool_get_state(void)
{
    return &g_dm_mit_tool_state;
}
