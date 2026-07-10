/*
 * SPDX-FileCopyrightText: 2026 陈轩 <2811158416@qq.com>
 * SPDX-License-Identifier: Apache-2.0
 *
 * First published in this repository: 2026-04-06
 * Use of this file is governed by the LICENSE file in the repository root.
 */

#include "ManualInput.h"
#include "ManualInputDbus.h"
#include "ManualInputSnapshot.h"

#include "FreeRTOS.h"
#include "task.h"
#include "timers.h"
#include "main.h"

#include <string.h>

#include "RobotConfig.h"
#include "ControlInput.h"
#include "BspKey.h"
#include "Watch.h"

#include "DetectTask.h"
#include "SdLog.h"

#ifndef RC_CHANNEL_ERROR_VALUE
#define RC_CHANNEL_ERROR_VALUE ((int16_t)(RC_CH_VALUE_ABS_MAX + 64u))
#endif

#define MANUAL_INPUT_SNAPSHOT_BANK_COUNT 2u
#define MANUAL_INPUT_REFRESH_ATTEMPT_MAX 2u
#define MANUAL_INPUT_AGE_INVALID         0xFFFFFFFFu

typedef char ManualInputDbusFrameLengthCheck[
    (RC_FRAME_LENGTH == MANUAL_INPUT_DBUS_FRAME_LENGTH) ? 1 : -1];
typedef char ManualInputDbusChannelOffsetCheck[
    (RC_CH_VALUE_OFFSET == MANUAL_INPUT_DBUS_CHANNEL_OFFSET) ? 1 : -1];
typedef char ManualInputDbusSwitchValueCheck[
    (RC_SW_UP == MANUAL_INPUT_DBUS_SWITCH_UP &&
     RC_SW_MID == MANUAL_INPUT_DBUS_SWITCH_MID &&
     RC_SW_DOWN == MANUAL_INPUT_DBUS_SWITCH_DOWN) ? 1 : -1];
static int16_t RC_abs(int16_t value);
/**
  * @brief          remote control protocol resolution
  * @param[in]      sbus_buf: raw data point
  * @param[out]     rc_ctrl: remote control data struct point
  * @retval         none
  */
static void ManualInputDbusToRc(const ManualInputDbusData *dbus, ManualInputState *rc);

//remote control data
static ManualInputState rc_ctrl;

typedef struct
{
    ManualInputState rc;
    TickType_t last_update_tick;
    uint32_t update_seq;
    uint8_t valid;
} ManualInputSrcState;

typedef struct
{
    ManualInputSnapshot bank[MANUAL_INPUT_SNAPSHOT_BANK_COUNT];
    uint16_t readers[MANUAL_INPUT_SNAPSHOT_BANK_COUNT];
    uint8_t expired[MANUAL_INPUT_SNAPSHOT_BANK_COUNT];
    uint8_t active_bank;
    uint8_t ready;
} ManualInputSnapshotStore;

typedef struct
{
    ManualInputSrcState source[MANUAL_INPUT_SRC_MAX + 1u];
    ManualInputConfig manualConfig;
    input_config_t inputConfig;
    ManualInputSnapshot candidate;
} ManualInputRefreshWorkspace;

// Per-source raw snapshots (index == MANUAL_INPUT_SRC_*).
static ManualInputSrcState manual_src[MANUAL_INPUT_SRC_MAX + 1u];
static uint8_t manual_active_src = MANUAL_INPUT_SRC_AUTO;
static TickType_t ManualInputRefreshTick = 0u;
static uint8_t ManualInputRefreshDirty = 1u;
static uint8_t ManualInputRefreshBusy = 0u;
static uint32_t ManualInputDirtySeq = 1u;
static uint32_t ManualInputRefreshSeq = 0u;
static uint32_t ManualInputSourceSeq = 0u;
static uint32_t ManualInputPublishSeq = 0u;
static uint32_t ManualInputSwitchSeq = 0u;
static ManualInputSnapshotStore ManualInputStore;
static ManualInputRefreshWorkspace ManualInputWorkspace;
static uint32_t g_remote_control_sbus_frame_cnt = 0u;
static uint32_t g_remote_control_sbus_reject_cnt = 0u;
static uint32_t g_remote_control_set_source_cnt = 0u;
static TimerHandle_t ManualInputRefreshTimer = NULL;
static StaticTimer_t ManualInputRefreshTimerBuffer;

typedef struct
{
    uint8_t from_isr;
    UBaseType_t saved_mask;
} ManualInputCriticalState;

static void ManualInputResetRc(ManualInputState *rc);
static uint8_t ManualInputStateValid(const ManualInputState *rc);
static void ManualInputSanitizeSwitch(ManualInputState *rc);
static void ManualInputApplyBoardKey(ManualInputState *rc, uint16_t key_mask);
static uint8_t ManualInputSrcIsActive(const ManualInputSrcState *src_state,
                                      uint8_t src,
                                      TickType_t now_tick,
                                      uint32_t timeout_ms);
static uint8_t ManualInputPickLatest(const ManualInputSrcState *src_state,
                                     TickType_t now_tick,
                                     uint32_t timeout_ms);
static uint32_t ManualInputSeqNext(uint32_t seq);
static uint8_t ManualInputSeqNewer(uint32_t lhs, uint32_t rhs);
static uint32_t ManualInputTickMs(TickType_t tick);
static uint32_t ManualInputAgeMs(TickType_t now_tick, TickType_t source_tick);
static uint32_t ManualInputTimeoutMs(const ManualInputConfig *cfg);
static uint8_t ManualInputMixMode(const ManualInputConfig *cfg);
static uint32_t ManualInputSourceMask(uint8_t source);
static void ManualInputStoreInit(void);
static void ManualInputCopySources(ManualInputSrcState *out);
static void ManualInputCopyConfig(ManualInputConfig *manual_cfg, input_config_t *input_cfg);
static void ManualInputExpireSourcesLocked(const ManualInputSrcState *src_state,
                                           TickType_t now_tick,
                                           uint32_t timeout_ms);
static void ManualInputBuildSnapshot(const ManualInputSrcState *src_state,
                                     const ManualInputConfig *manual_cfg,
                                     const input_config_t *input_cfg,
                                     TickType_t now_tick,
                                     ManualInputSnapshot *out);
static void ManualInputMarkDirty(void);
static void ManualInputRefreshIfNeeded(uint8_t force);
static void ManualInputRefreshTimerCallback(TimerHandle_t timer);
static void remote_control_log_source_switch(uint8_t prev_src, uint8_t next_src);
static void remote_control_log_sbus_raw_frame(const uint8_t frame[RC_FRAME_LENGTH]);

static ManualInputCriticalState ManualInputEnterCritical(void)
{
    ManualInputCriticalState state;

    state.from_isr = (__get_IPSR() != 0U) ? 1u : 0u;
    state.saved_mask = 0u;
    if (state.from_isr != 0u)
    {
        state.saved_mask = taskENTER_CRITICAL_FROM_ISR();
    }
    else
    {
        taskENTER_CRITICAL();
    }
    return state;
}

static void ManualInputExitCritical(ManualInputCriticalState state)
{
    if (state.from_isr != 0u)
    {
        taskEXIT_CRITICAL_FROM_ISR(state.saved_mask);
    }
    else
    {
        taskEXIT_CRITICAL();
    }
}

/**
  * @brief          remote control init
  * @param[in]      none
  * @retval         none
  */
void ManualInputInit(void)
{
    TickType_t refresh_period_tick = pdMS_TO_TICKS(5u);

    ManualInputResetRc(&rc_ctrl);
    for (uint8_t i = 0u; i <= (uint8_t)MANUAL_INPUT_SRC_MAX; i++)
    {
        ManualInputResetRc(&manual_src[i].rc);
        manual_src[i].last_update_tick = 0u;
        manual_src[i].update_seq = 0u;
        manual_src[i].valid = 0u;
    }
    manual_active_src = MANUAL_INPUT_SRC_AUTO;
    ManualInputRefreshTick = 0u;
    ManualInputRefreshDirty = 1u;
    ManualInputRefreshBusy = 0u;
    ManualInputDirtySeq = 1u;
    ManualInputRefreshSeq = 0u;
    ManualInputSourceSeq = 0u;
    ManualInputPublishSeq = 0u;
    ManualInputSwitchSeq = 0u;
    memset(&ManualInputStore, 0, sizeof(ManualInputStore));
    memset(&ManualInputWorkspace, 0, sizeof(ManualInputWorkspace));
    g_remote_control_sbus_frame_cnt = 0u;
    g_remote_control_sbus_reject_cnt = 0u;
    g_remote_control_set_source_cnt = 0u;
    ManualInputStoreInit();
    if (refresh_period_tick == 0u)
    {
        refresh_period_tick = 1u;
    }

    ManualInputRefreshTimer = xTimerCreateStatic("ManualInputRefresh",
                                                    refresh_period_tick,
                                                    pdTRUE,
                                                    NULL,
                                                    ManualInputRefreshTimerCallback,
                                                    &ManualInputRefreshTimerBuffer);
    if (ManualInputRefreshTimer != NULL)
    {
        (void)xTimerStart(ManualInputRefreshTimer, 0u);
    }

    BspRcSbusInit();
}

void remote_control_init(void)
{
    ManualInputInit();
}
/**
  * @brief          get remote control data point
  * @param[in]      none
  * @retval         remote control data point
  */
const ManualInputState *ManualInputGetCurrentRc(void)
{
    return &rc_ctrl;
}

uint8_t ManualInputGetCurrentCopy(ManualInputState *out)
{
    ManualInputSnapshot snapshot;

    if (out == NULL)
    {
        return 0u;
    }

    if (ManualInputSnapshotRead(&snapshot) == 0u)
    {
        ManualInputResetRc(out);
        return 0u;
    }
    *out = snapshot.manual;
    return 1u;
}

const ManualInputState *get_remote_control_point(void)
{
    return ManualInputGetCurrentRc();
}

void remote_control_set_rc(const ManualInputState *rc)
{
    ManualInputUpdateSource(MANUAL_INPUT_SRC_ELRS, rc);
}

void ManualInputUpdateSource(uint8_t source, const ManualInputState *rc)
{
    const TickType_t now_tick = xTaskGetTickCount();

    if (rc == NULL)
    {
        return;
    }
    if (source == 0u || source > (uint8_t)MANUAL_INPUT_SRC_MAX)
    {
        return;
    }
    ManualInputCriticalState critical = ManualInputEnterCritical();
    ManualInputSourceSeq = ManualInputSeqNext(ManualInputSourceSeq);
    manual_src[source].rc = *rc;
    manual_src[source].last_update_tick = now_tick;
    manual_src[source].update_seq = ManualInputSourceSeq;
    manual_src[source].valid = 1u;
    g_remote_control_set_source_cnt++;
    ManualInputRefreshDirty = 1u;
    ManualInputDirtySeq = ManualInputSeqNext(ManualInputDirtySeq);
    ManualInputExitCritical(critical);
    /* 第二提交迁完旧 toe 消费者前，暂时维持所有手动来源共享 DBUS_TOE 的兼容行为。 */
    DetectHook(DBUS_TOE);
    ManualInputRefreshIfNeeded(1u);
}

void remote_control_set_rc_source(uint8_t source, const ManualInputState *rc)
{
    ManualInputUpdateSource(source, rc);
}

void ManualInputOnSbusFrame(const uint8_t frame[RC_FRAME_LENGTH])
{
    ManualInputDbusData dbus;

    if (frame == NULL)
    {
        return;
    }

    if (ManualInputDbusDecode(frame, &dbus) == 0u || ManualInputDbusValid(&dbus) == 0u)
    {
        ManualInputCriticalState critical = ManualInputEnterCritical();
        g_remote_control_sbus_reject_cnt++;
        ManualInputExitCritical(critical);
        return;
    }

    ManualInputState rc = {0};
    ManualInputDbusToRc(&dbus, &rc);
    remote_control_log_sbus_raw_frame(frame);
    ManualInputCriticalState critical = ManualInputEnterCritical();
    g_remote_control_sbus_frame_cnt++;
    ManualInputExitCritical(critical);
    ManualInputUpdateSource(MANUAL_INPUT_SRC_DBUS, &rc);
}

void remote_control_on_sbus_frame(const uint8_t frame[RC_FRAME_LENGTH])
{
    ManualInputOnSbusFrame(frame);
}

void remote_control_log_raw_source(uint8_t source,
                                   uint8_t proto,
                                   uint8_t range_mode,
                                   uint8_t channel_count,
                                   const int16_t *ch_raw,
                                   const uint8_t sw_raw[2],
                                   const ManualInputState *decoded)
{
    if (SdLogIsActive() == 0u || ch_raw == NULL || channel_count == 0u)
    {
        return;
    }
    if (channel_count > 16u)
    {
        channel_count = 16u;
    }

    sdlog_manual_input_raw_t pkt = {0};
    pkt.source = source;
    pkt.proto = proto;
    pkt.range_mode = range_mode;
    pkt.channel_count = channel_count;

    for (uint8_t i = 0u; i < channel_count; i++)
    {
        pkt.ch_raw[i] = ch_raw[i];
    }

    if (sw_raw != NULL)
    {
        pkt.sw[0] = sw_raw[0];
        pkt.sw[1] = sw_raw[1];
    }
    else if (decoded != NULL)
    {
        pkt.sw[0] = (uint8_t)decoded->rc.s[0];
        pkt.sw[1] = (uint8_t)decoded->rc.s[1];
    }

    if (decoded != NULL)
    {
        pkt.mouse_x = decoded->mouse.x;
        pkt.mouse_y = decoded->mouse.y;
        pkt.mouse_z = decoded->mouse.z;
        pkt.key_value = decoded->key.v;
        if (decoded->mouse.press_l != 0u)
        {
            pkt.mouse_btns |= 0x01u;
        }
        if (decoded->mouse.press_r != 0u)
        {
            pkt.mouse_btns |= 0x02u;
        }
    }

    SdLogWrite(SDLOG_TAG_MANUAL_INPUT_RAW, &pkt, (uint16_t)sizeof(pkt));
}

uint8_t ManualInputGetActiveSource(void)
{
    ManualInputSnapshot snapshot;
    return (ManualInputSnapshotRead(&snapshot) != 0u) ?
               snapshot.activeSource :
               (uint8_t)MANUAL_INPUT_SRC_AUTO;
}

uint8_t remote_control_get_active_source(void)
{
    return ManualInputGetActiveSource();
}

void ManualInputRefresh(void)
{
    ManualInputMarkDirty();
    ManualInputRefreshIfNeeded(1u);
}

void remote_control_refresh(void)
{
    ManualInputRefresh();
}

uint32_t ManualInputGetSbusFrameCount(void)
{
    uint32_t count;

    ManualInputCriticalState critical = ManualInputEnterCritical();
    count = g_remote_control_sbus_frame_cnt;
    ManualInputExitCritical(critical);
    return count;
}

uint32_t ManualInputGetSbusRejectCount(void)
{
    uint32_t count;

    ManualInputCriticalState critical = ManualInputEnterCritical();
    count = g_remote_control_sbus_reject_cnt;
    ManualInputExitCritical(critical);
    return count;
}

uint32_t remote_control_get_sbus_frame_count(void)
{
    return ManualInputGetSbusFrameCount();
}

uint32_t remote_control_get_sbus_reject_count(void)
{
    return ManualInputGetSbusRejectCount();
}

uint32_t ManualInputGetSetSourceCount(void)
{
    uint32_t count;

    ManualInputCriticalState critical = ManualInputEnterCritical();
    count = g_remote_control_set_source_cnt;
    ManualInputExitCritical(critical);
    return count;
}

uint32_t remote_control_get_set_source_count(void)
{
    return ManualInputGetSetSourceCount();
}

static void ManualInputResetRc(ManualInputState *rc)
{
    if (rc == NULL)
    {
        return;
    }

    memset(rc, 0, sizeof(*rc));
    // 无有效来源时固定使用原始上档；当前所有目标都把它配置为安全、停火档。
    rc->rc.s[0] = (char)RC_SW_UP;
    rc->rc.s[1] = (char)RC_SW_UP;
}

static uint8_t ManualInputStateValid(const ManualInputState *rc)
{
    uint8_t i;

    if (rc == NULL)
    {
        return 0u;
    }

    for (i = 0u; i < 5u; i++)
    {
        if (rc->rc.ch[i] < -RC_CHANNEL_ERROR_VALUE || rc->rc.ch[i] > RC_CHANNEL_ERROR_VALUE)
        {
            return 0u;
        }
    }

    for (i = 0u; i < 2u; i++)
    {
        const uint8_t sw = (uint8_t)rc->rc.s[i];
        if (!(sw == RC_SW_UP || sw == RC_SW_MID || sw == RC_SW_DOWN))
        {
            return 0u;
        }
    }

    if (rc->mouse.press_l > 1u || rc->mouse.press_r > 1u)
    {
        return 0u;
    }

    return 1u;
}

static void ManualInputSanitizeSwitch(ManualInputState *rc)
{
    if (rc == NULL)
    {
        return;
    }

    const uint8_t s0 = (uint8_t)rc->rc.s[0];
    const uint8_t s1 = (uint8_t)rc->rc.s[1];
    if (!(s0 == RC_SW_UP || s0 == RC_SW_MID || s0 == RC_SW_DOWN))
    {
        rc->rc.s[0] = (char)RC_SW_UP;
    }
    if (!(s1 == RC_SW_UP || s1 == RC_SW_MID || s1 == RC_SW_DOWN))
    {
        rc->rc.s[1] = (char)RC_SW_UP;
    }
}

static void ManualInputApplyBoardKey(ManualInputState *rc, uint16_t key_mask)
{
    if (rc == NULL)
    {
        return;
    }

    if (key_mask == 0u)
    {
        return;
    }
    if (BspKeyReadRawDown() != 0u)
    {
        rc->key.v |= key_mask;
    }
}

static uint32_t ManualInputSeqNext(uint32_t seq)
{
    seq++;
    return (seq != 0u) ? seq : 1u;
}

static uint8_t ManualInputSeqNewer(uint32_t lhs, uint32_t rhs)
{
    return (uint8_t)(lhs != rhs && (int32_t)(lhs - rhs) > 0);
}

static uint32_t ManualInputTickMs(TickType_t tick)
{
    uint32_t tick_ms = (uint32_t)portTICK_PERIOD_MS;
    if (tick_ms == 0u)
    {
        tick_ms = 1u;
    }
    return (uint32_t)tick * tick_ms;
}

static uint32_t ManualInputAgeMs(TickType_t now_tick, TickType_t source_tick)
{
    const TickType_t age_tick = (TickType_t)(now_tick - source_tick);
    uint32_t tick_ms = (uint32_t)portTICK_PERIOD_MS;

    if (tick_ms == 0u)
    {
        tick_ms = 1u;
    }
    if ((uint32_t)age_tick > (MANUAL_INPUT_AGE_INVALID / tick_ms))
    {
        return MANUAL_INPUT_AGE_INVALID;
    }
    return (uint32_t)age_tick * tick_ms;
}

static uint32_t ManualInputTimeoutMs(const ManualInputConfig *cfg)
{
    if (cfg == NULL || cfg->source_timeout_ms == 0u)
    {
        return MANUAL_INPUT_DEFAULT_TIMEOUT_MS;
    }
    return cfg->source_timeout_ms;
}

static uint8_t ManualInputMixMode(const ManualInputConfig *cfg)
{
    return (uint8_t)(cfg != NULL && cfg->mix_mode == MANUAL_INPUT_MIX_MERGE ?
                         MANUAL_INPUT_MIX_MERGE :
                         MANUAL_INPUT_MIX_SELECT_LATEST);
}

static uint32_t ManualInputSourceMask(uint8_t source)
{
    if (source == 0u || source > 32u)
    {
        return 0u;
    }
    return (uint32_t)1u << (source - 1u);
}

static uint8_t ManualInputSrcIsActive(const ManualInputSrcState *src_state,
                                      uint8_t src,
                                      TickType_t now_tick,
                                      uint32_t timeout_ms)
{
    if (src_state == NULL || src == 0u || src > (uint8_t)MANUAL_INPUT_SRC_MAX)
    {
        return 0u;
    }
    if (src_state[src].valid == 0u || src_state[src].update_seq == 0u)
    {
        return 0u;
    }
    return (ManualInputAgeMs(now_tick, src_state[src].last_update_tick) <= timeout_ms) ? 1u : 0u;
}

static uint8_t ManualInputPickLatest(const ManualInputSrcState *src_state,
                                     TickType_t now_tick,
                                     uint32_t timeout_ms)
{
    uint8_t best = 0u;
    uint32_t best_age = MANUAL_INPUT_AGE_INVALID;
    uint32_t best_seq = 0u;

    for (uint8_t src = (uint8_t)MANUAL_INPUT_SRC_DBUS; src <= (uint8_t)MANUAL_INPUT_SRC_MAX; src++)
    {
        if (ManualInputSrcIsActive(src_state, src, now_tick, timeout_ms) == 0u)
        {
            continue;
        }

        const uint32_t age = ManualInputAgeMs(now_tick, src_state[src].last_update_tick);
        if (best == 0u || age < best_age ||
            (age == best_age && ManualInputSeqNewer(src_state[src].update_seq, best_seq) != 0u))
        {
            best_age = age;
            best_seq = src_state[src].update_seq;
            best = src;
        }
    }

    return best;
}

static void ManualInputStoreInit(void)
{
    const TickType_t now_tick = xTaskGetTickCount();

    ManualInputWorkspace.manualConfig = g_config.manual_input;
    ManualInputWorkspace.inputConfig = g_config.input;
    ManualInputBuildSnapshot(manual_src,
                             &ManualInputWorkspace.manualConfig,
                             &ManualInputWorkspace.inputConfig,
                             now_tick,
                             &ManualInputWorkspace.candidate);
    ManualInputPublishSeq = ManualInputSeqNext(ManualInputPublishSeq);
    ManualInputWorkspace.candidate.publishSeq = ManualInputPublishSeq;
    ManualInputWorkspace.candidate.switchSeq = ManualInputSwitchSeq;

    ManualInputStore.bank[0] = ManualInputWorkspace.candidate;
    ManualInputStore.bank[1] = ManualInputWorkspace.candidate;
    ManualInputStore.readers[0] = 0u;
    ManualInputStore.readers[1] = 0u;
    ManualInputStore.expired[0] = 0u;
    ManualInputStore.expired[1] = 0u;
    ManualInputStore.active_bank = 0u;
    ManualInputStore.ready = 1u;
    rc_ctrl = ManualInputWorkspace.candidate.manual;
    manual_active_src = ManualInputWorkspace.candidate.activeSource;
    ManualInputRefreshTick = now_tick;
    ManualInputRefreshSeq = ManualInputDirtySeq;
    ManualInputRefreshDirty = 0u;
}

static void ManualInputCopySources(ManualInputSrcState *out)
{
    if (out == NULL)
    {
        return;
    }

    for (uint8_t source = 0u; source <= (uint8_t)MANUAL_INPUT_SRC_MAX; source++)
    {
        ManualInputCriticalState critical = ManualInputEnterCritical();
        out[source] = manual_src[source];
        ManualInputExitCritical(critical);
    }
}

static void ManualInputCopyConfig(ManualInputConfig *manual_cfg, input_config_t *input_cfg)
{
    if (manual_cfg == NULL || input_cfg == NULL)
    {
        return;
    }

    ManualInputCriticalState critical = ManualInputEnterCritical();
    *manual_cfg = g_config.manual_input;
    *input_cfg = g_config.input;
    ManualInputExitCritical(critical);
}

/* 调用者必须已进入 ManualInput 临界区并确认配置代与 dirty generation 未变化。 */
static void ManualInputExpireSourcesLocked(const ManualInputSrcState *src_state,
                                           TickType_t now_tick,
                                           uint32_t timeout_ms)
{
    if (src_state == NULL)
    {
        return;
    }

    for (uint8_t source = (uint8_t)MANUAL_INPUT_SRC_DBUS;
         source <= (uint8_t)MANUAL_INPUT_SRC_MAX;
         source++)
    {
        if (src_state[source].valid == 0u || src_state[source].update_seq == 0u ||
            ManualInputAgeMs(now_tick, src_state[source].last_update_tick) <= timeout_ms)
        {
            continue;
        }
        if (manual_src[source].valid != 0u &&
            manual_src[source].update_seq == src_state[source].update_seq)
        {
            /* 旧帧一旦确认超时便永久失效，避免完整 tick 回绕后复活。 */
            manual_src[source].valid = 0u;
        }
    }
}

static void ManualInputBuildSnapshot(const ManualInputSrcState *src_state,
                                     const ManualInputConfig *manual_cfg,
                                     const input_config_t *input_cfg,
                                     TickType_t now_tick,
                                     ManualInputSnapshot *out)
{
    uint8_t latest;
    uint8_t selected;
    const uint32_t timeout_ms = ManualInputTimeoutMs(manual_cfg);
    const uint8_t mix_mode = ManualInputMixMode(manual_cfg);

    if (src_state == NULL || out == NULL)
    {
        return;
    }

    memset(out, 0, sizeof(*out));
    ManualInputResetRc(&out->manual);
    out->sourceAgeMs = MANUAL_INPUT_AGE_INVALID;
    out->sourceTimeoutMs = timeout_ms;
    out->publishTickMs = ManualInputTickMs(now_tick);
    out->readTickMs = out->publishTickMs;
    out->mixMode = mix_mode;

    for (uint8_t source = (uint8_t)MANUAL_INPUT_SRC_DBUS;
         source <= (uint8_t)MANUAL_INPUT_SRC_MAX;
         source++)
    {
        if (ManualInputSrcIsActive(src_state, source, now_tick, timeout_ms) != 0u)
        {
            out->activeMask |= ManualInputSourceMask(source);
        }
    }

    latest = ManualInputPickLatest(src_state, now_tick, timeout_ms);
    selected = latest;

    if (mix_mode == MANUAL_INPUT_MIX_MERGE)
    {
        if (latest != 0u)
        {
            out->manual.rc.s[0] = src_state[latest].rc.rc.s[0];
            out->manual.rc.s[1] = src_state[latest].rc.rc.s[1];

            for (uint8_t source = (uint8_t)MANUAL_INPUT_SRC_DBUS;
                 source <= (uint8_t)MANUAL_INPUT_SRC_MAX;
                 source++)
            {
                if (ManualInputSrcIsActive(src_state, source, now_tick, timeout_ms) == 0u)
                {
                    continue;
                }

                for (uint8_t channel = 0u; channel < 5u; channel++)
                {
                    const int16_t value = src_state[source].rc.rc.ch[channel];
                    if (RC_abs(value) > RC_abs(out->manual.rc.ch[channel]))
                    {
                        out->manual.rc.ch[channel] = value;
                    }
                }

                const int16_t mouse_x = src_state[source].rc.mouse.x;
                const int16_t mouse_y = src_state[source].rc.mouse.y;
                const int16_t mouse_z = src_state[source].rc.mouse.z;
                if (RC_abs(mouse_x) > RC_abs(out->manual.mouse.x)) out->manual.mouse.x = mouse_x;
                if (RC_abs(mouse_y) > RC_abs(out->manual.mouse.y)) out->manual.mouse.y = mouse_y;
                if (RC_abs(mouse_z) > RC_abs(out->manual.mouse.z)) out->manual.mouse.z = mouse_z;

                out->manual.key.v |= src_state[source].rc.key.v;
                out->manual.mouse.press_l |= src_state[source].rc.mouse.press_l;
                out->manual.mouse.press_r |= src_state[source].rc.mouse.press_r;
            }
        }
    }
    else
    {
        selected = (manual_cfg != NULL) ? manual_cfg->active_source : MANUAL_INPUT_SRC_AUTO;
        if (selected == MANUAL_INPUT_SRC_AUTO)
        {
            selected = latest;
        }
        else if (ManualInputSrcIsActive(src_state, selected, now_tick, timeout_ms) == 0u)
        {
            selected = latest;
        }

        if (selected != 0u)
        {
            out->manual = src_state[selected].rc;
        }
    }

    ManualInputSanitizeSwitch(&out->manual);
    ManualInputApplyBoardKey(&out->manual,
                             (manual_cfg != NULL) ? manual_cfg->BoardKeyKeyMask : 0u);
    if (selected != 0u)
    {
        ControlInputBuild(&out->manual, input_cfg, &out->control);
    }
    else
    {
        ControlInputBuild(NULL, NULL, &out->control);
    }

    if (selected != 0u)
    {
        out->activeSource = selected;
        out->sourceTickMs = ManualInputTickMs(src_state[selected].last_update_tick);
        out->sourceAgeMs = ManualInputAgeMs(now_tick, src_state[selected].last_update_tick);
        out->sourceSeq = src_state[selected].update_seq;
        out->online = (out->sourceAgeMs <= timeout_ms) ? 1u : 0u;
    }
    else
    {
        out->activeSource = MANUAL_INPUT_SRC_AUTO;
        out->sourceTickMs = 0u;
        out->sourceAgeMs = MANUAL_INPUT_AGE_INVALID;
        out->sourceSeq = 0u;
        out->online = 0u;
    }
}

uint8_t ManualInputSnapshotRead(ManualInputSnapshot *out)
{
    uint8_t bank;
    uint8_t expired;
    uint8_t has_source;
    uint8_t stale = 0u;

    if (out == NULL)
    {
        return 0u;
    }

    ManualInputCriticalState critical = ManualInputEnterCritical();
    bank = ManualInputStore.active_bank;
    if (ManualInputStore.ready == 0u || bank >= MANUAL_INPUT_SNAPSHOT_BANK_COUNT ||
        ManualInputStore.readers[bank] == 0xFFFFu)
    {
        ManualInputExitCritical(critical);
        memset(out, 0, sizeof(*out));
        ManualInputResetRc(&out->manual);
        ControlInputBuild(NULL, NULL, &out->control);
        out->sourceAgeMs = MANUAL_INPUT_AGE_INVALID;
        out->sourceTimeoutMs = MANUAL_INPUT_DEFAULT_TIMEOUT_MS;
        return 0u;
    }
    ManualInputStore.readers[bank]++;
    expired = ManualInputStore.expired[bank];
    ManualInputExitCritical(critical);

    __DMB();
    *out = ManualInputStore.bank[bank];
    __DMB();

    out->readTickMs = ManualInputTickMs(xTaskGetTickCount());
    if (out->sourceTimeoutMs == 0u)
    {
        out->sourceTimeoutMs = MANUAL_INPUT_DEFAULT_TIMEOUT_MS;
    }
    has_source = (uint8_t)(out->activeSource != MANUAL_INPUT_SRC_AUTO && out->sourceSeq != 0u);
    if (has_source == 0u)
    {
        out->sourceAgeMs = MANUAL_INPUT_AGE_INVALID;
        out->online = 0u;
    }
    else
    {
        out->sourceAgeMs = out->readTickMs - out->sourceTickMs;
        stale = (uint8_t)(expired != 0u || out->sourceAgeMs > out->sourceTimeoutMs);
        out->online = (stale == 0u) ? 1u : 0u;
    }

    /* stale 判定和 latch 都发生在释放 reader pin 之前，writer 不会误复用这一代。 */
    critical = ManualInputEnterCritical();
    if (ManualInputStore.expired[bank] != 0u)
    {
        stale = 1u;
    }
    if (stale != 0u && has_source != 0u)
    {
        ManualInputStore.expired[bank] = 1u;
    }
    if (ManualInputStore.readers[bank] != 0u)
    {
        ManualInputStore.readers[bank]--;
    }
    ManualInputExitCritical(critical);

    if (stale != 0u)
    {
        /* 即使定时器守护任务停滞或 tick 完整回绕，旧接口也只能看到安全帧。 */
        ManualInputResetRc(&out->manual);
        ControlInputBuild(NULL, NULL, &out->control);
        out->activeMask = 0u;
        out->sourceTickMs = 0u;
        out->sourceAgeMs = MANUAL_INPUT_AGE_INVALID;
        out->sourceSeq = 0u;
        out->activeSource = MANUAL_INPUT_SRC_AUTO;
        out->online = 0u;
    }
    return 1u;
}

static void ManualInputMarkDirty(void)
{
    ManualInputCriticalState critical = ManualInputEnterCritical();
    ManualInputRefreshDirty = 1u;
    ManualInputDirtySeq = ManualInputSeqNext(ManualInputDirtySeq);
    ManualInputExitCritical(critical);
}

static void ManualInputRefreshIfNeeded(uint8_t force)
{
    uint8_t inactive_bank;
    uint8_t published = 0u;
    uint8_t prev_source = MANUAL_INPUT_SRC_AUTO;
    TickType_t now_tick = xTaskGetTickCount();

    ManualInputCriticalState critical = ManualInputEnterCritical();
    if (force == 0u && ManualInputRefreshDirty == 0u &&
        ManualInputDirtySeq == ManualInputRefreshSeq && ManualInputRefreshTick == now_tick)
    {
        ManualInputExitCritical(critical);
        return;
    }
    if (ManualInputRefreshBusy != 0u || ManualInputStore.ready == 0u)
    {
        ManualInputExitCritical(critical);
        return;
    }

    inactive_bank = (uint8_t)(ManualInputStore.active_bank ^ 1u);
    if (inactive_bank >= MANUAL_INPUT_SNAPSHOT_BANK_COUNT ||
        ManualInputStore.readers[inactive_bank] != 0u)
    {
        ManualInputRefreshDirty = 1u;
        ManualInputExitCritical(critical);
        return;
    }
    ManualInputRefreshBusy = 1u;
    ManualInputExitCritical(critical);

    for (uint8_t attempt = 0u; attempt < MANUAL_INPUT_REFRESH_ATTEMPT_MAX; attempt++)
    {
        uint32_t dirty_seq;
        uint32_t publish_seq;
        uint32_t switch_seq;
        uint8_t current_source;

        now_tick = xTaskGetTickCount();
        critical = ManualInputEnterCritical();
        dirty_seq = ManualInputDirtySeq;
        publish_seq = ManualInputPublishSeq;
        switch_seq = ManualInputSwitchSeq;
        current_source = ManualInputStore.bank[ManualInputStore.active_bank].activeSource;
        ManualInputExitCritical(critical);

        ManualInputCopySources(ManualInputWorkspace.source);
        ManualInputCopyConfig(&ManualInputWorkspace.manualConfig,
                              &ManualInputWorkspace.inputConfig);
        ManualInputBuildSnapshot(ManualInputWorkspace.source,
                                 &ManualInputWorkspace.manualConfig,
                                 &ManualInputWorkspace.inputConfig,
                                 now_tick,
                                 &ManualInputWorkspace.candidate);
        ManualInputWorkspace.candidate.publishSeq = ManualInputSeqNext(publish_seq);
        ManualInputWorkspace.candidate.switchSeq =
            (ManualInputWorkspace.candidate.activeSource != current_source) ?
                ManualInputSeqNext(switch_seq) :
                switch_seq;

        /* 非活动 bank 没有读者，整块候选在临界区外写入。 */
        ManualInputStore.bank[inactive_bank] = ManualInputWorkspace.candidate;
        __DMB();

        critical = ManualInputEnterCritical();
        if (ManualInputDirtySeq == dirty_seq &&
            memcmp(&g_config.manual_input,
                   &ManualInputWorkspace.manualConfig,
                   sizeof(ManualInputWorkspace.manualConfig)) == 0 &&
            memcmp(&g_config.input,
                   &ManualInputWorkspace.inputConfig,
                   sizeof(ManualInputWorkspace.inputConfig)) == 0)
        {
            ManualInputExpireSourcesLocked(ManualInputWorkspace.source,
                                           now_tick,
                                           ManualInputWorkspace.candidate.sourceTimeoutMs);
            prev_source = manual_active_src;
            ManualInputPublishSeq = ManualInputWorkspace.candidate.publishSeq;
            ManualInputSwitchSeq = ManualInputWorkspace.candidate.switchSeq;
            ManualInputRefreshTick = now_tick;
            ManualInputRefreshSeq = dirty_seq;
            ManualInputRefreshDirty = 0u;
            rc_ctrl = ManualInputWorkspace.candidate.manual;
            manual_active_src = ManualInputWorkspace.candidate.activeSource;
            ManualInputStore.expired[inactive_bank] = 0u;
            __DMB();
            ManualInputStore.active_bank = inactive_bank;
            published = 1u;
        }
        else
        {
            /* 计算期间已有新输入或配置；旧候选绝不翻成当前帧。 */
            ManualInputRefreshDirty = 1u;
        }
        ManualInputExitCritical(critical);

        if (published != 0u)
        {
            break;
        }
    }

    if (published != 0u)
    {
        /* busy 覆盖观察与日志副作用，保证发布顺序不会被下一 writer 反转。 */
        WatchUpdateRcSnapshot(&ManualInputWorkspace.candidate.manual);
        if (prev_source != ManualInputWorkspace.candidate.activeSource)
        {
            remote_control_log_source_switch(prev_source,
                                             ManualInputWorkspace.candidate.activeSource);
        }
    }

    critical = ManualInputEnterCritical();
    ManualInputRefreshBusy = 0u;
    ManualInputExitCritical(critical);
}

static void ManualInputRefreshTimerCallback(TimerHandle_t timer)
{
    (void)timer;
    ManualInputRefreshIfNeeded(0u);
}

uint8_t RC_data_is_error(void)
{
    ManualInputSnapshot snapshot;

    if (ManualInputSnapshotRead(&snapshot) == 0u || snapshot.online == 0u)
    {
        return 1u;
    }
    return (uint8_t)(ManualInputStateValid(&snapshot.manual) == 0u);
}

void slove_RC_lost(void)
{
    RC_restart(SBUS_RX_BUF_NUM);
}
void slove_data_error(void)
{
    RC_restart(SBUS_RX_BUF_NUM);
}

static int16_t RC_abs(int16_t value)
{
    if (value > 0)
    {
        return value;
    }
    else
    {
        return -value;
    }
}

static void remote_control_log_source_switch(uint8_t prev_src, uint8_t next_src)
{
    uint32_t set_source_cnt;

    if (SdLogIsActive() == 0u)
    {
        return;
    }

    ManualInputCriticalState critical = ManualInputEnterCritical();
    set_source_cnt = g_remote_control_set_source_cnt;
    ManualInputExitCritical(critical);

    sdlog_event_t evt = {0};
    evt.event_id = SDLOG_EVT_MANUAL_SOURCE_SWITCH;
    evt.arg0_u16 = next_src;
    evt.arg1_u32 = prev_src;
    evt.arg2_u32 = set_source_cnt;
    SdLogWrite(SDLOG_TAG_EVENT, &evt, (uint16_t)sizeof(evt));
}

static void remote_control_log_sbus_raw_frame(const uint8_t frame[RC_FRAME_LENGTH])
{
    if (frame == NULL)
    {
        return;
    }

    int16_t ch_raw[5] = {0};
    const uint8_t sw_raw[2] = {
        (uint8_t)((frame[5] >> 4) & 0x03u),
        (uint8_t)(((frame[5] >> 4) & 0x0Cu) >> 2),
    };
    ManualInputState decoded = {0};

    ch_raw[0] = (int16_t)((frame[0] | (frame[1] << 8)) & 0x07FFu);
    ch_raw[1] = (int16_t)(((frame[1] >> 3) | (frame[2] << 5)) & 0x07FFu);
    ch_raw[2] = (int16_t)(((frame[2] >> 6) | (frame[3] << 2) | (frame[4] << 10)) & 0x07FFu);
    ch_raw[3] = (int16_t)(((frame[4] >> 1) | (frame[5] << 7)) & 0x07FFu);
    ch_raw[4] = (int16_t)(frame[16] | (frame[17] << 8));

    decoded.rc.s[0] = (char)sw_raw[0];
    decoded.rc.s[1] = (char)sw_raw[1];
    decoded.mouse.x = (int16_t)(frame[6] | (frame[7] << 8));
    decoded.mouse.y = (int16_t)(frame[8] | (frame[9] << 8));
    decoded.mouse.z = (int16_t)(frame[10] | (frame[11] << 8));
    decoded.mouse.press_l = frame[12];
    decoded.mouse.press_r = frame[13];
    decoded.key.v = (uint16_t)(frame[14] | (frame[15] << 8));

    remote_control_log_raw_source(MANUAL_INPUT_SRC_DBUS,
                                  SDLOG_MANUAL_INPUT_PROTO_DBUS,
                                  SDLOG_MANUAL_INPUT_RANGE_RAW_11BIT,
                                  5u,
                                  ch_raw,
                                  sw_raw,
                                  &decoded);
}
/**
  * @brief          remote control protocol resolution
  * @param[in]      sbus_buf: raw data point
  * @param[out]     rc_ctrl: remote control data struct point
  * @retval         none
  */
static void ManualInputDbusToRc(const ManualInputDbusData *dbus, ManualInputState *rc)
{
    uint8_t i;

    if (dbus == NULL || rc == NULL)
    {
        return;
    }

    for (i = 0u; i < 5u; i++)
    {
        const int16_t centered = (int16_t)((int32_t)dbus->channel[i] - (int32_t)RC_CH_VALUE_OFFSET);
        rc->rc.ch[i] = rc_scale_axis_by_abs(centered,
                                            (int16_t)RC_CH_VALUE_ABS_LEGACY,
                                            (int16_t)RC_CH_VALUE_ABS_MAX);
    }

    rc->rc.s[0] = (char)dbus->sw[0];
    rc->rc.s[1] = (char)dbus->sw[1];
    rc->mouse.x = dbus->mouse[0];
    rc->mouse.y = dbus->mouse[1];
    rc->mouse.z = dbus->mouse[2];
    rc->mouse.press_l = dbus->mouseButton[0];
    rc->mouse.press_r = dbus->mouseButton[1];
    rc->key.v = dbus->key;
}
