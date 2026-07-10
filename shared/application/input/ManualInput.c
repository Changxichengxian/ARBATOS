/*
 * SPDX-FileCopyrightText: 2026 陈轩 <2811158416@qq.com>
 * SPDX-License-Identifier: Apache-2.0
 *
 * First published in this repository: 2026-04-06
 * Use of this file is governed by the LICENSE file in the repository root.
 */

#include "ManualInput.h"
#include "ManualInputCrsf.h"
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
/**
  * @brief          remote control protocol resolution
  * @param[in]      sbus_buf: raw data point
  * @param[out]     rc_ctrl: remote control data struct point
  * @retval         none
  */
static void ManualInputDbusToRc(const ManualInputDbusData *dbus, ManualInputState *rc);

typedef struct
{
    ManualInputState rc;
    TickType_t last_update_tick;
    uint32_t update_seq;
    uint8_t protocol;
    uint8_t raw_flags;
    uint8_t raw_switch1;
    uint8_t valid;
    /* 每次确认失效推进一代；bank 只有冻结到同一代才可再次使用该来源。 */
    uint32_t invalidate_gen;
} ManualInputSrcState;

typedef struct
{
    uint16_t channel[MANUAL_INPUT_CRSF_CHANNEL_COUNT];
    uint8_t valid;
} ManualInputCrsfState;

typedef struct
{
    ManualInputSnapshot bank[MANUAL_INPUT_SNAPSHOT_BANK_COUNT];
    ManualInputSrcState source[MANUAL_INPUT_SNAPSHOT_BANK_COUNT][MANUAL_INPUT_SRC_MAX + 1u];
    ManualInputCrsfState crsf[MANUAL_INPUT_SNAPSHOT_BANK_COUNT];
    ManualInputConfig manual_config[MANUAL_INPUT_SNAPSHOT_BANK_COUNT];
    input_config_t input_config[MANUAL_INPUT_SNAPSHOT_BANK_COUNT];
    uint32_t excluded_mask[MANUAL_INPUT_SNAPSHOT_BANK_COUNT];
    uint8_t board_key_down[MANUAL_INPUT_SNAPSHOT_BANK_COUNT];
    uint8_t active_bank;
    uint8_t ready;
} ManualInputSnapshotStore;

typedef struct
{
    ManualInputSrcState source[MANUAL_INPUT_SRC_MAX + 1u];
    ManualInputCrsfState crsf;
    ManualInputConfig manualConfig;
    input_config_t inputConfig;
    ManualInputSnapshot candidate;
    uint8_t boardKeyDown;
} ManualInputRefreshWorkspace;

typedef struct
{
    uint32_t contributorMask;
    uint32_t semanticsSeq;
    uint32_t invalidateGen[MANUAL_INPUT_SRC_MAX];
    uint8_t protocol[MANUAL_INPUT_SRC_MAX];
    uint8_t mixMode;
    uint8_t activeSource;
    uint8_t ready;
} ManualInputActionAuthority;

typedef struct
{
    uint32_t invalidateGen;
    uint8_t activeSource;
    uint8_t protocol;
    uint8_t mixMode;
    uint8_t ready;
} ManualInputControlAuthority;

// Per-source raw snapshots (index == MANUAL_INPUT_SRC_*).
static ManualInputSrcState manual_src[MANUAL_INPUT_SRC_MAX + 1u];
static ManualInputCrsfState manual_crsf;
static uint8_t manual_active_src = MANUAL_INPUT_SRC_AUTO;
static uint8_t ManualInputRefreshDirty = 1u;
static uint8_t ManualInputRefreshBusy = 0u;
static uint32_t ManualInputDirtySeq = 1u;
static uint32_t ManualInputRefreshSeq = 0u;
static uint32_t ManualInputSourceSeq = 0u;
static uint32_t ManualInputPublishSeq = 0u;
static uint32_t ManualInputSwitchSeq = 0u;
static uint32_t ManualInputActionSeq = 1u;
static uint32_t ManualInputAuthoritySeq = 1u;
static ManualInputSnapshotStore ManualInputStore;
static ManualInputRefreshWorkspace ManualInputWorkspace;
static ManualInputActionAuthority ManualInputActionCurrent;
static ManualInputControlAuthority ManualInputAuthorityCurrent;
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
static void ManualInputApplyBoardKey(ManualInputState *rc,
                                     uint16_t key_mask,
                                     uint8_t board_key_down);
static uint8_t ManualInputSrcIsActive(const ManualInputSrcState *src_state,
                                      uint8_t src,
                                      TickType_t now_tick,
                                      uint32_t timeout_ms,
                                      uint32_t excluded_mask);
static uint8_t ManualInputPickLatest(const ManualInputSrcState *src_state,
                                     TickType_t now_tick,
                                     uint32_t timeout_ms,
                                     uint32_t excluded_mask);
static uint8_t ManualInputSelectSource(const ManualInputSrcState *src_state,
                                       const ManualInputConfig *manual_cfg,
                                       TickType_t now_tick,
                                       uint32_t timeout_ms,
                                       uint32_t excluded_mask,
                                       uint8_t preferred_source,
                                       uint8_t mix_mode);
static void ManualInputActionAuthorityBuild(const ManualInputSrcState *source,
                                            const ManualInputSnapshot *snapshot,
                                            ManualInputActionAuthority *out);
static uint32_t ManualInputActionAuthoritySyncLocked(const ManualInputSrcState *source,
                                                     const ManualInputSnapshot *snapshot);
static uint32_t ManualInputControlAuthoritySyncLocked(const ManualInputSrcState *source,
                                                      const ManualInputSnapshot *snapshot);
static uint32_t ManualInputSeqNext(uint32_t seq);
static uint8_t ManualInputSeqNewer(uint32_t lhs, uint32_t rhs);
static uint32_t ManualInputTickMs(TickType_t tick);
static uint32_t ManualInputAgeMs(TickType_t now_tick, TickType_t source_tick);
static uint32_t ManualInputTimeoutMs(const ManualInputConfig *cfg);
static uint8_t ManualInputMixMode(const ManualInputConfig *cfg);
static uint32_t ManualInputSourceMask(uint8_t source);
static uint8_t ManualInputSourceFlags(const ManualInputSrcState *source,
                                      const ManualInputConfig *config);
static void ManualInputApplySourceMapping(ManualInputState *rc,
                                          const ManualInputSrcState *source,
                                          const ManualInputConfig *config);
static void ManualInputResolveSource(const ManualInputSrcState *source,
                                     const ManualInputCrsfState *crsf,
                                     const input_config_t *input_config,
                                     ManualInputState *out);
static void ManualInputUpdateSourceDetail(uint8_t source,
                                          const ManualInputState *rc,
                                          uint8_t protocol,
                                          uint8_t raw_flags,
                                          uint8_t raw_switch1,
                                          const uint16_t *crsf_raw);
static void ManualInputStoreInit(void);
static void ManualInputCopySources(ManualInputSrcState *out,
                                   ManualInputCrsfState *crsf);
static void ManualInputCopyConfig(ManualInputConfig *manual_cfg, input_config_t *input_cfg);
static void ManualInputExpireSourcesLocked(const ManualInputSrcState *src_state,
                                           TickType_t now_tick,
                                           uint32_t timeout_ms);
static void ManualInputBuildSnapshot(const ManualInputSrcState *src_state,
                                     const ManualInputCrsfState *crsf,
                                     const ManualInputConfig *manual_cfg,
                                     const input_config_t *input_cfg,
                                     TickType_t now_tick,
                                     uint32_t excluded_mask,
                                     uint8_t preferred_source,
                                     uint8_t board_key_down,
                                     ManualInputSnapshot *out);
static uint32_t ManualInputExpiredMask(const ManualInputSrcState *src_state,
                                       TickType_t now_tick,
                                       uint32_t timeout_ms);
static uint32_t ManualInputInvalidatedMaskLocked(uint8_t bank);
static void ManualInputPreservePublishedGeneration(const ManualInputSnapshot *published,
                                                   ManualInputSnapshot *derived);
static void ManualInputMarkDirty(void);
static void ManualInputRefreshIfNeeded(uint8_t force);
static void ManualInputRefreshTimerCallback(TimerHandle_t timer);
static void ManualInputLogSourceSwitch(uint8_t prev_src, uint8_t next_src);
static void ManualInputLogSbusRawFrame(const uint8_t frame[RC_FRAME_LENGTH]);

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

    for (uint8_t i = 0u; i <= (uint8_t)MANUAL_INPUT_SRC_MAX; i++)
    {
        ManualInputResetRc(&manual_src[i].rc);
        manual_src[i].last_update_tick = 0u;
        manual_src[i].update_seq = 0u;
        manual_src[i].protocol = MANUAL_INPUT_PROTOCOL_NONE;
        manual_src[i].raw_flags = 0u;
        manual_src[i].raw_switch1 = 0u;
        manual_src[i].valid = 0u;
        manual_src[i].invalidate_gen = 0u;
    }
    memset(&manual_crsf, 0, sizeof(manual_crsf));
    manual_active_src = MANUAL_INPUT_SRC_AUTO;
    ManualInputRefreshDirty = 1u;
    ManualInputRefreshBusy = 0u;
    ManualInputDirtySeq = 1u;
    ManualInputRefreshSeq = 0u;
    ManualInputSourceSeq = 0u;
    ManualInputPublishSeq = 0u;
    ManualInputSwitchSeq = 0u;
    ManualInputActionSeq = 1u;
    ManualInputAuthoritySeq = 1u;
    memset(&ManualInputStore, 0, sizeof(ManualInputStore));
    memset(&ManualInputWorkspace, 0, sizeof(ManualInputWorkspace));
    memset(&ManualInputActionCurrent, 0, sizeof(ManualInputActionCurrent));
    memset(&ManualInputAuthorityCurrent, 0, sizeof(ManualInputAuthorityCurrent));
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

void ManualInputUpdateImageSource(const ManualInputState *rc,
                                  uint8_t protocol,
                                  uint8_t rawFlags,
                                  uint8_t rawSwitch1)
{
    if (rc == NULL ||
        (protocol != MANUAL_INPUT_PROTOCOL_IMAGE_CUSTOM &&
         protocol != MANUAL_INPUT_PROTOCOL_IMAGE_VT13))
    {
        ManualInputInvalidateSource(MANUAL_INPUT_SRC_IMAGE);
        return;
    }
    ManualInputUpdateSourceDetail(MANUAL_INPUT_SRC_IMAGE,
                                  rc,
                                  protocol,
                                  rawFlags,
                                  rawSwitch1,
                                  NULL);
}

void ManualInputUpdateElrsChannels(
    const ManualInputState *decoded,
    const uint16_t raw[MANUAL_INPUT_CRSF_CHANNEL_COUNT])
{
    if (decoded == NULL || raw == NULL)
    {
        ManualInputInvalidateSource(MANUAL_INPUT_SRC_ELRS);
        return;
    }
    ManualInputUpdateSourceDetail(MANUAL_INPUT_SRC_ELRS,
                                  decoded,
                                  MANUAL_INPUT_PROTOCOL_CRSF,
                                  0u,
                                  0u,
                                  raw);
}

static void ManualInputUpdateSourceDetail(uint8_t source,
                                          const ManualInputState *rc,
                                          uint8_t protocol,
                                          uint8_t raw_flags,
                                          uint8_t raw_switch1,
                                          const uint16_t *crsf_raw)
{
    const TickType_t now_tick = xTaskGetTickCount();
    ManualInputState source_copy;
    uint8_t source_valid;

    if (rc == NULL)
    {
        return;
    }
    if (source == 0u || source > (uint8_t)MANUAL_INPUT_SRC_MAX)
    {
        return;
    }
    /* 校验和入库必须使用同一份值，不能再次解引用调用者的可变缓冲。 */
    source_copy = *rc;
    source_valid = ManualInputStateValid(&source_copy);
    ManualInputCriticalState critical = ManualInputEnterCritical();
    const uint8_t old_valid = manual_src[source].valid;
    const uint8_t protocol_changed = (uint8_t)(old_valid != 0u && source_valid != 0u &&
                                                manual_src[source].protocol != protocol);
    const uint8_t session_timed_out =
        (uint8_t)(old_valid != 0u && source_valid != 0u &&
                  ManualInputAgeMs(now_tick, manual_src[source].last_update_tick) >
                      ManualInputTimeoutMs(&g_config.manual_input));

    if ((source_valid == 0u && old_valid != 0u) ||
        protocol_changed != 0u || session_timed_out != 0u)
    {
        manual_src[source].invalidate_gen =
            ManualInputSeqNext(manual_src[source].invalidate_gen);
    }
    ManualInputSourceSeq = ManualInputSeqNext(ManualInputSourceSeq);
    manual_src[source].rc = source_copy;
    manual_src[source].last_update_tick = now_tick;
    manual_src[source].update_seq = ManualInputSourceSeq;
    manual_src[source].protocol = protocol;
    manual_src[source].raw_flags = raw_flags;
    manual_src[source].raw_switch1 = raw_switch1;
    if (source == MANUAL_INPUT_SRC_ELRS)
    {
        manual_crsf.valid = (uint8_t)(crsf_raw != NULL);
        if (manual_crsf.valid != 0u)
        {
            memcpy(manual_crsf.channel,
                   crsf_raw,
                   sizeof(manual_crsf.channel));
        }
    }
    /* 非法来源立即退出候选，其他健康来源仍可被同一发布代选中。 */
    manual_src[source].valid = source_valid;
    g_remote_control_set_source_cnt++;
    ManualInputRefreshDirty = 1u;
    ManualInputDirtySeq = ManualInputSeqNext(ManualInputDirtySeq);
    ManualInputExitCritical(critical);
    /* DBUS_TOE 只描述物理 DBUS；统一来源在线状态由发布快照单独给出。 */
    if (source == MANUAL_INPUT_SRC_DBUS && source_valid != 0u)
    {
        DetectHook(DBUS_TOE);
    }
    ManualInputRefreshIfNeeded(1u);
}

void ManualInputInvalidateSource(uint8_t source)
{
    const TickType_t now_tick = xTaskGetTickCount();

    if (source == 0u || source > (uint8_t)MANUAL_INPUT_SRC_MAX)
    {
        return;
    }

    ManualInputCriticalState critical = ManualInputEnterCritical();
    if (manual_src[source].valid == 0u)
    {
        ManualInputExitCritical(critical);
        return;
    }
    manual_src[source].invalidate_gen =
        ManualInputSeqNext(manual_src[source].invalidate_gen);
    ManualInputSourceSeq = ManualInputSeqNext(ManualInputSourceSeq);
    manual_src[source].last_update_tick = now_tick;
    manual_src[source].update_seq = ManualInputSourceSeq;
    manual_src[source].valid = 0u;
    if (source == MANUAL_INPUT_SRC_ELRS)
    {
        manual_crsf.valid = 0u;
    }
    g_remote_control_set_source_cnt++;
    ManualInputRefreshDirty = 1u;
    ManualInputDirtySeq = ManualInputSeqNext(ManualInputDirtySeq);
    ManualInputExitCritical(critical);

    ManualInputRefreshIfNeeded(1u);
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
    ManualInputLogSbusRawFrame(frame);
    ManualInputCriticalState critical = ManualInputEnterCritical();
    g_remote_control_sbus_frame_cnt++;
    ManualInputExitCritical(critical);
    ManualInputUpdateSourceDetail(MANUAL_INPUT_SRC_DBUS,
                                  &rc,
                                  MANUAL_INPUT_PROTOCOL_DBUS,
                                  0u,
                                  0u,
                                  NULL);
}

void ManualInputLogRawSource(uint8_t source,
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

void ManualInputRefresh(void)
{
    ManualInputMarkDirty();
    ManualInputRefreshIfNeeded(1u);
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

uint32_t ManualInputGetSetSourceCount(void)
{
    uint32_t count;

    ManualInputCriticalState critical = ManualInputEnterCritical();
    count = g_remote_control_set_source_cnt;
    ManualInputExitCritical(critical);
    return count;
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

static void ManualInputApplyBoardKey(ManualInputState *rc,
                                     uint16_t key_mask,
                                     uint8_t board_key_down)
{
    if (rc == NULL)
    {
        return;
    }

    if (key_mask == 0u)
    {
        return;
    }
    if (board_key_down != 0u)
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
    if (cfg == NULL)
    {
        return MANUAL_INPUT_MIX_SELECT_STICKY;
    }
    if (cfg->mix_mode == MANUAL_INPUT_MIX_MERGE ||
        cfg->mix_mode == MANUAL_INPUT_MIX_SELECT_LATEST)
    {
        return cfg->mix_mode;
    }
    return MANUAL_INPUT_MIX_SELECT_STICKY;
}

static uint32_t ManualInputSourceMask(uint8_t source)
{
    if (source == 0u || source > 32u)
    {
        return 0u;
    }
    return (uint32_t)1u << (source - 1u);
}

static uint8_t ManualInputSourceFlags(const ManualInputSrcState *source,
                                      const ManualInputConfig *config)
{
    uint8_t flags = 0u;

    if (source == NULL || config == NULL ||
        (source->protocol != (uint8_t)MANUAL_INPUT_PROTOCOL_IMAGE_CUSTOM &&
         source->protocol != (uint8_t)MANUAL_INPUT_PROTOCOL_IMAGE_VT13))
    {
        return 0u;
    }
    if (((config->vt13.auto_aim_pause_enable != 0u) &&
         ((source->raw_flags & MANUAL_INPUT_SOURCE_RAW_PAUSE) != 0u)) ||
        ((config->vt13.auto_aim_mouse_r_enable != 0u) &&
         ((source->raw_flags & MANUAL_INPUT_SOURCE_RAW_MOUSE_R) != 0u)))
    {
        flags |= MANUAL_INPUT_SOURCE_FLAG_AUTO_AIM;
    }
    if (((config->vt13.AuxFireBtnLEnable != 0u) &&
         ((source->raw_flags & MANUAL_INPUT_SOURCE_RAW_BTN_L) != 0u)) ||
        ((config->vt13.AuxFireMouseLEnable != 0u) &&
         ((source->raw_flags & MANUAL_INPUT_SOURCE_RAW_MOUSE_L) != 0u)))
    {
        flags |= MANUAL_INPUT_SOURCE_FLAG_AUX_FIRE;
    }
    return flags;
}

static uint8_t ManualInputMapVt13Switch1(uint8_t value,
                                         const ManualInputConfig *config)
{
    if (config == NULL)
    {
        return RC_SW_UP;
    }
    if (value == config->vt13.switch1_safe_value)
    {
        return ControlInputSwitchPosToRaw(config->semantics.GimbalSafePos);
    }
    if (value == config->vt13.switch1_normal_value || value == 3u)
    {
        return ControlInputSwitchPosToRaw(config->semantics.ChassisFollowPos);
    }
    if (value == config->vt13.switch1_spin_value)
    {
        return ControlInputSwitchPosToRaw(config->semantics.ChassisSpinPos);
    }
    return ControlInputSwitchPosToRaw(config->semantics.GimbalSafePos);
}

static uint8_t ManualInputMapVt13Switch2(uint8_t raw_flags,
                                         const ManualInputConfig *config)
{
    if (config == NULL)
    {
        return RC_SW_UP;
    }
    if ((raw_flags & MANUAL_INPUT_SOURCE_RAW_PAUSE) != 0u)
    {
        return ControlInputSwitchPosToRaw(config->vt13.switch2_pause_pos);
    }
    if ((raw_flags & MANUAL_INPUT_SOURCE_RAW_BTN_L) != 0u)
    {
        return ControlInputSwitchPosToRaw(config->vt13.switch2_btn_l_pos);
    }
    if ((raw_flags & MANUAL_INPUT_SOURCE_RAW_BTN_R) != 0u)
    {
        return ControlInputSwitchPosToRaw(config->vt13.switch2_btn_r_pos);
    }
    return ControlInputSwitchPosToRaw(config->semantics.ShootStopPos);
}

static void ManualInputApplySourceMapping(ManualInputState *rc,
                                          const ManualInputSrcState *source,
                                          const ManualInputConfig *config)
{
    if (rc == NULL || source == NULL || config == NULL ||
        source->protocol != (uint8_t)MANUAL_INPUT_PROTOCOL_IMAGE_VT13)
    {
        return;
    }

    rc->rc.s[0] = (char)ManualInputMapVt13Switch1(source->raw_switch1, config);
    rc->rc.s[1] = (char)ManualInputMapVt13Switch2(source->raw_flags, config);
}

static void ManualInputResolveSource(const ManualInputSrcState *source,
                                     const ManualInputCrsfState *crsf,
                                     const input_config_t *input_config,
                                     ManualInputState *out)
{
    if (source == NULL || out == NULL)
    {
        return;
    }

    *out = source->rc;
    if (source->protocol == MANUAL_INPUT_PROTOCOL_CRSF &&
        crsf != NULL && crsf->valid != 0u && input_config != NULL)
    {
        ManualInputCrsfDecode(crsf->channel, input_config, out);
    }
}

static uint8_t ManualInputSrcIsActive(const ManualInputSrcState *src_state,
                                      uint8_t src,
                                      TickType_t now_tick,
                                      uint32_t timeout_ms,
                                      uint32_t excluded_mask)
{
    if (src_state == NULL || src == 0u || src > (uint8_t)MANUAL_INPUT_SRC_MAX)
    {
        return 0u;
    }
    if ((excluded_mask & ManualInputSourceMask(src)) != 0u ||
        src_state[src].valid == 0u || src_state[src].update_seq == 0u)
    {
        return 0u;
    }
    return (ManualInputAgeMs(now_tick, src_state[src].last_update_tick) <= timeout_ms) ? 1u : 0u;
}

static uint8_t ManualInputPickLatest(const ManualInputSrcState *src_state,
                                     TickType_t now_tick,
                                     uint32_t timeout_ms,
                                     uint32_t excluded_mask)
{
    uint8_t best = 0u;
    uint32_t best_age = MANUAL_INPUT_AGE_INVALID;
    uint32_t best_seq = 0u;

    for (uint8_t src = (uint8_t)MANUAL_INPUT_SRC_DBUS; src <= (uint8_t)MANUAL_INPUT_SRC_MAX; src++)
    {
        if (ManualInputSrcIsActive(src_state,
                                   src,
                                   now_tick,
                                   timeout_ms,
                                   excluded_mask) == 0u)
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

static uint8_t ManualInputSelectSource(const ManualInputSrcState *src_state,
                                       const ManualInputConfig *manual_cfg,
                                       TickType_t now_tick,
                                       uint32_t timeout_ms,
                                       uint32_t excluded_mask,
                                       uint8_t preferred_source,
                                       uint8_t mix_mode)
{
    uint8_t configured_source = (manual_cfg != NULL) ?
                                    manual_cfg->active_source :
                                    MANUAL_INPUT_SRC_AUTO;

    /* 显式指定的来源恢复后重新接管；离线期间仍允许健康来源维持基本控制。 */
    if (configured_source != MANUAL_INPUT_SRC_AUTO &&
        ManualInputSrcIsActive(src_state,
                               configured_source,
                               now_tick,
                               timeout_ms,
                               excluded_mask) != 0u)
    {
        return configured_source;
    }

    if (mix_mode != MANUAL_INPUT_MIX_SELECT_LATEST &&
        ManualInputSrcIsActive(src_state,
                               preferred_source,
                               now_tick,
                               timeout_ms,
                               excluded_mask) != 0u)
    {
        return preferred_source;
    }

    return ManualInputPickLatest(src_state, now_tick, timeout_ms, excluded_mask);
}

static void ManualInputActionAuthorityBuild(const ManualInputSrcState *source,
                                            const ManualInputSnapshot *snapshot,
                                            ManualInputActionAuthority *out)
{
    uint32_t contributor_mask;

    if (out == NULL)
    {
        return;
    }
    memset(out, 0, sizeof(*out));
    if (source == NULL || snapshot == NULL)
    {
        return;
    }

    contributor_mask = ManualInputSourceMask(snapshot->activeSource);
    if (snapshot->mixMode == MANUAL_INPUT_MIX_MERGE)
    {
        /* 非代表来源只能通过类型明确的 Image 业务位贡献动作。 */
        contributor_mask |= snapshot->activeMask &
                            ManualInputSourceMask(MANUAL_INPUT_SRC_IMAGE);
    }

    out->contributorMask = contributor_mask;
    out->semanticsSeq = snapshot->semanticsSeq;
    out->mixMode = snapshot->mixMode;
    out->activeSource = snapshot->activeSource;
    for (uint8_t source_id = (uint8_t)MANUAL_INPUT_SRC_DBUS;
         source_id <= (uint8_t)MANUAL_INPUT_SRC_MAX;
         source_id++)
    {
        if ((contributor_mask & ManualInputSourceMask(source_id)) == 0u)
        {
            continue;
        }
        out->invalidateGen[source_id - 1u] = source[source_id].invalidate_gen;
        out->protocol[source_id - 1u] = source[source_id].protocol;
    }
    out->ready = 1u;
}

/* 调用者必须持有 ManualInput 临界区。 */
static uint32_t ManualInputActionAuthoritySyncLocked(const ManualInputSrcState *source,
                                                     const ManualInputSnapshot *snapshot)
{
    ManualInputActionAuthority next;

    ManualInputActionAuthorityBuild(source, snapshot, &next);
    if (next.ready == 0u)
    {
        return ManualInputActionSeq;
    }
    if (ManualInputActionCurrent.ready != 0u &&
        memcmp(&ManualInputActionCurrent, &next, sizeof(next)) != 0)
    {
        ManualInputActionSeq = ManualInputSeqNext(ManualInputActionSeq);
    }
    ManualInputActionCurrent = next;
    return ManualInputActionSeq;
}

/* 调用者必须持有 ManualInput 临界区。 */
static uint32_t ManualInputControlAuthoritySyncLocked(const ManualInputSrcState *source,
                                                      const ManualInputSnapshot *snapshot)
{
    ManualInputControlAuthority next = {0};

    if (source == NULL || snapshot == NULL)
    {
        return ManualInputAuthoritySeq;
    }
    next.activeSource = snapshot->activeSource;
    next.protocol = snapshot->sourceProtocol;
    next.mixMode = snapshot->mixMode;
    if (next.activeSource != MANUAL_INPUT_SRC_AUTO &&
        next.activeSource <= (uint8_t)MANUAL_INPUT_SRC_MAX)
    {
        next.invalidateGen = source[next.activeSource].invalidate_gen;
    }
    next.ready = 1u;

    if (ManualInputAuthorityCurrent.ready != 0u &&
        memcmp(&ManualInputAuthorityCurrent, &next, sizeof(next)) != 0)
    {
        ManualInputAuthoritySeq = ManualInputSeqNext(ManualInputAuthoritySeq);
    }
    ManualInputAuthorityCurrent = next;
    return ManualInputAuthoritySeq;
}

static void ManualInputStoreInit(void)
{
    const TickType_t now_tick = xTaskGetTickCount();

    ManualInputWorkspace.manualConfig = g_config.manual_input;
    ManualInputWorkspace.inputConfig = g_config.input;
    ManualInputWorkspace.boardKeyDown = BspKeyReadRawDown();
    ManualInputBuildSnapshot(manual_src,
                             &manual_crsf,
                             &ManualInputWorkspace.manualConfig,
                             &ManualInputWorkspace.inputConfig,
                             now_tick,
                             0u,
                             MANUAL_INPUT_SRC_AUTO,
                             ManualInputWorkspace.boardKeyDown,
                             &ManualInputWorkspace.candidate);
    ManualInputPublishSeq = ManualInputSeqNext(ManualInputPublishSeq);
    ManualInputWorkspace.candidate.publishSeq = ManualInputPublishSeq;
    ManualInputWorkspace.candidate.switchSeq = ManualInputSwitchSeq;
    ManualInputWorkspace.candidate.semanticsSeq = 1u;
    ManualInputWorkspace.candidate.actionSeq =
        ManualInputActionAuthoritySyncLocked(manual_src,
                                             &ManualInputWorkspace.candidate);
    ManualInputWorkspace.candidate.authoritySeq =
        ManualInputControlAuthoritySyncLocked(manual_src,
                                              &ManualInputWorkspace.candidate);

    ManualInputStore.bank[0] = ManualInputWorkspace.candidate;
    ManualInputStore.bank[1] = ManualInputWorkspace.candidate;
    for (uint8_t source = 0u; source <= (uint8_t)MANUAL_INPUT_SRC_MAX; source++)
    {
        ManualInputStore.source[0][source] = manual_src[source];
        ManualInputStore.source[1][source] = manual_src[source];
    }
    ManualInputStore.crsf[0] = manual_crsf;
    ManualInputStore.crsf[1] = manual_crsf;
    ManualInputStore.manual_config[0] = ManualInputWorkspace.manualConfig;
    ManualInputStore.manual_config[1] = ManualInputWorkspace.manualConfig;
    ManualInputStore.input_config[0] = ManualInputWorkspace.inputConfig;
    ManualInputStore.input_config[1] = ManualInputWorkspace.inputConfig;
    ManualInputStore.excluded_mask[0] = 0u;
    ManualInputStore.excluded_mask[1] = 0u;
    ManualInputStore.board_key_down[0] = ManualInputWorkspace.boardKeyDown;
    ManualInputStore.board_key_down[1] = ManualInputWorkspace.boardKeyDown;
    ManualInputStore.active_bank = 0u;
    ManualInputStore.ready = 1u;
    manual_active_src = ManualInputWorkspace.candidate.activeSource;
    ManualInputRefreshSeq = ManualInputDirtySeq;
    ManualInputRefreshDirty = 0u;
}

static void ManualInputCopySources(ManualInputSrcState *out,
                                   ManualInputCrsfState *crsf)
{
    if (out == NULL || crsf == NULL)
    {
        return;
    }

    for (uint8_t source = 0u; source <= (uint8_t)MANUAL_INPUT_SRC_MAX; source++)
    {
        ManualInputCriticalState critical = ManualInputEnterCritical();
        out[source] = manual_src[source];
        if (source == MANUAL_INPUT_SRC_ELRS)
        {
            *crsf = manual_crsf;
        }
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
            manual_src[source].invalidate_gen =
                ManualInputSeqNext(manual_src[source].invalidate_gen);
        }
    }
}

static uint32_t ManualInputExpiredMask(const ManualInputSrcState *src_state,
                                       TickType_t now_tick,
                                       uint32_t timeout_ms)
{
    uint32_t mask = 0u;

    if (src_state == NULL)
    {
        return 0u;
    }

    for (uint8_t source = (uint8_t)MANUAL_INPUT_SRC_DBUS;
         source <= (uint8_t)MANUAL_INPUT_SRC_MAX;
         source++)
    {
        if (src_state[source].valid != 0u && src_state[source].update_seq != 0u &&
            ManualInputAgeMs(now_tick, src_state[source].last_update_tick) > timeout_ms)
        {
            mask |= ManualInputSourceMask(source);
        }
    }
    return mask;
}

/* 调用者必须持有 ManualInput 临界区。 */
static uint32_t ManualInputInvalidatedMaskLocked(uint8_t bank)
{
    uint32_t mask = 0u;

    if (bank >= MANUAL_INPUT_SNAPSHOT_BANK_COUNT)
    {
        return 0u;
    }

    for (uint8_t source = (uint8_t)MANUAL_INPUT_SRC_DBUS;
         source <= (uint8_t)MANUAL_INPUT_SRC_MAX;
         source++)
    {
        if (manual_src[source].invalidate_gen !=
            ManualInputStore.source[bank][source].invalidate_gen)
        {
            mask |= ManualInputSourceMask(source);
        }
    }
    return mask;
}

static void ManualInputPreservePublishedGeneration(const ManualInputSnapshot *published,
                                                   ManualInputSnapshot *derived)
{
    if (published == NULL || derived == NULL)
    {
        return;
    }

    derived->publishTickMs = published->publishTickMs;
    derived->publishSeq = published->publishSeq;
    derived->switchSeq = published->switchSeq;
    derived->semanticsSeq = published->semanticsSeq;
    derived->actionSeq = published->actionSeq;
    derived->authoritySeq = published->authoritySeq;
}

static void ManualInputBuildSnapshot(const ManualInputSrcState *src_state,
                                     const ManualInputCrsfState *crsf,
                                     const ManualInputConfig *manual_cfg,
                                     const input_config_t *input_cfg,
                                     TickType_t now_tick,
                                     uint32_t excluded_mask,
                                     uint8_t preferred_source,
                                     uint8_t board_key_down,
                                     ManualInputSnapshot *out)
{
    uint8_t selected;
    uint8_t merged_flags = 0u;
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
    if (manual_cfg != NULL)
    {
        out->semantics = manual_cfg->semantics;
    }

    for (uint8_t source = (uint8_t)MANUAL_INPUT_SRC_DBUS;
         source <= (uint8_t)MANUAL_INPUT_SRC_MAX;
         source++)
    {
        if (ManualInputSrcIsActive(src_state,
                                   source,
                                   now_tick,
                                   timeout_ms,
                                   excluded_mask) != 0u)
        {
            out->activeMask |= ManualInputSourceMask(source);
        }
    }

    selected = ManualInputSelectSource(src_state,
                                       manual_cfg,
                                       now_tick,
                                       timeout_ms,
                                       excluded_mask,
                                       preferred_source,
                                       mix_mode);

    if (selected != 0u)
    {
        /* 连续轴、通用键鼠和拨杆始终只来自稳定代表来源，避免拼出不存在的操纵帧。 */
        ManualInputResolveSource(&src_state[selected], crsf, input_cfg, &out->manual);
        if (mix_mode == MANUAL_INPUT_MIX_MERGE)
        {
            for (uint8_t source = (uint8_t)MANUAL_INPUT_SRC_DBUS;
                 source <= (uint8_t)MANUAL_INPUT_SRC_MAX;
                 source++)
            {
                if (ManualInputSrcIsActive(src_state,
                                           source,
                                           now_tick,
                                           timeout_ms,
                                           excluded_mask) == 0u)
                {
                    continue;
                }
                merged_flags |= ManualInputSourceFlags(&src_state[source], manual_cfg);
            }
        }
    }

    if (selected != 0u)
    {
        /* 协议原始拨杆和业务位都用本次候选冻结的同一份配置解释。 */
        ManualInputApplySourceMapping(&out->manual, &src_state[selected], manual_cfg);
    }
    ManualInputSanitizeSwitch(&out->manual);
    ManualInputApplyBoardKey(&out->manual,
                             (manual_cfg != NULL) ? manual_cfg->BoardKeyKeyMask : 0u,
                             board_key_down);
    out->dataValid = (uint8_t)(selected != 0u && ManualInputStateValid(&out->manual) != 0u);
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
        out->sourceProtocol = src_state[selected].protocol;
        /* MERGE 的代表来源负责开关/协议，业务请求按所有实际贡献者做并集。 */
        out->sourceFlags = (mix_mode == MANUAL_INPUT_MIX_MERGE) ?
                               merged_flags :
                               ManualInputSourceFlags(&src_state[selected], manual_cfg);
        /* online 是控制许可的一部分，因此必须蕴含本代数据合法。 */
        out->online = (uint8_t)(out->sourceAgeMs <= timeout_ms && out->dataValid != 0u);
    }
    else
    {
        out->activeSource = MANUAL_INPUT_SRC_AUTO;
        out->sourceTickMs = 0u;
        out->sourceAgeMs = MANUAL_INPUT_AGE_INVALID;
        out->sourceSeq = 0u;
        out->sourceProtocol = 0u;
        out->sourceFlags = 0u;
        out->online = 0u;
    }
}

uint8_t ManualInputSnapshotRead(ManualInputSnapshot *out)
{
    uint8_t bank;
    uint8_t expired_global = 0u;
    TickType_t now_tick;
    uint32_t now_ms;
    uint32_t timeout_ms;
    uint32_t timeout_mask;
    uint32_t excluded_mask;

    if (out == NULL)
    {
        return 0u;
    }

    ManualInputCriticalState critical = ManualInputEnterCritical();
    bank = ManualInputStore.active_bank;
    if (ManualInputStore.ready == 0u || bank >= MANUAL_INPUT_SNAPSHOT_BANK_COUNT)
    {
        ManualInputExitCritical(critical);
        memset(out, 0, sizeof(*out));
        ManualInputResetRc(&out->manual);
        ControlInputBuild(NULL, NULL, &out->control);
        out->sourceAgeMs = MANUAL_INPUT_AGE_INVALID;
        out->sourceTimeoutMs = MANUAL_INPUT_DEFAULT_TIMEOUT_MS;
        return 0u;
    }
    /* SnapshotRead 只允许任务调用；临界区内取 tick，返回值不会比线性化时刻更旧。 */
    now_tick = xTaskGetTickCount();
    now_ms = ManualInputTickMs(now_tick);
    timeout_ms = ManualInputStore.bank[bank].sourceTimeoutMs;
    if (timeout_ms == 0u)
    {
        timeout_ms = MANUAL_INPUT_DEFAULT_TIMEOUT_MS;
    }
    timeout_mask = ManualInputExpiredMask(ManualInputStore.source[bank],
                                          now_tick,
                                          timeout_ms);
    ManualInputStore.excluded_mask[bank] |= timeout_mask;
    ManualInputStore.excluded_mask[bank] |= ManualInputInvalidatedMaskLocked(bank);
    excluded_mask = ManualInputStore.excluded_mask[bank];

    *out = ManualInputStore.bank[bank];
    out->readTickMs = now_ms;
    out->sourceTimeoutMs = timeout_ms;

    if ((excluded_mask & out->activeMask) != 0u)
    {
        /* 失效边界才纯重建；函数不访问日志、BSP 或 RTOS，临界区长度固定有界。 */
        ManualInputBuildSnapshot(ManualInputStore.source[bank],
                                 &ManualInputStore.crsf[bank],
                                 &ManualInputStore.manual_config[bank],
                                 &ManualInputStore.input_config[bank],
                                 now_tick,
                                 excluded_mask,
                                 ManualInputStore.bank[bank].activeSource,
                                 ManualInputStore.board_key_down[bank],
                                 out);
        ManualInputPreservePublishedGeneration(&ManualInputStore.bank[bank], out);
        out->readTickMs = now_ms;
        out->actionSeq =
            ManualInputActionAuthoritySyncLocked(ManualInputStore.source[bank], out);
        out->authoritySeq =
            ManualInputControlAuthoritySyncLocked(ManualInputStore.source[bank], out);
    }
    else if (out->activeSource == MANUAL_INPUT_SRC_AUTO || out->sourceSeq == 0u)
    {
        out->sourceAgeMs = MANUAL_INPUT_AGE_INVALID;
        out->online = 0u;
    }
    else
    {
        out->sourceAgeMs = out->readTickMs - out->sourceTickMs;
        out->online = (uint8_t)(out->sourceAgeMs <= out->sourceTimeoutMs &&
                                out->dataValid != 0u);
    }

    for (uint8_t source = (uint8_t)MANUAL_INPUT_SRC_DBUS;
         source <= (uint8_t)MANUAL_INPUT_SRC_MAX;
         source++)
    {
        const uint32_t bit = ManualInputSourceMask(source);
        if ((timeout_mask & bit) != 0u &&
            manual_src[source].valid != 0u &&
            manual_src[source].update_seq == ManualInputStore.source[bank][source].update_seq)
        {
            manual_src[source].valid = 0u;
            manual_src[source].invalidate_gen =
                ManualInputSeqNext(manual_src[source].invalidate_gen);
            expired_global = 1u;
        }
    }
    if (expired_global != 0u)
    {
        ManualInputRefreshDirty = 1u;
        ManualInputDirtySeq = ManualInputSeqNext(ManualInputDirtySeq);
    }
    ManualInputExitCritical(critical);
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
    uint8_t tail_retry = 0u;
    uint8_t prev_source = MANUAL_INPUT_SRC_AUTO;
    uint8_t frozen_board_key = 0u;
    TickType_t now_tick = xTaskGetTickCount();

    ManualInputCriticalState critical = ManualInputEnterCritical();
    if (ManualInputRefreshBusy != 0u || ManualInputStore.ready == 0u)
    {
        ManualInputExitCritical(critical);
        return;
    }
    if (force == 0u && ManualInputRefreshDirty == 0u &&
        ManualInputDirtySeq == ManualInputRefreshSeq)
    {
        const uint8_t active_bank = ManualInputStore.active_bank;
        if (active_bank >= MANUAL_INPUT_SNAPSHOT_BANK_COUNT)
        {
            ManualInputRefreshDirty = 1u;
            ManualInputExitCritical(critical);
            return;
        }
        frozen_board_key = ManualInputStore.board_key_down[active_bank];
        ManualInputExitCritical(critical);
        if (BspKeyReadRawDown() == frozen_board_key)
        {
            return;
        }

        /* 空闲时只轮询板载键；来源年龄由读端同代裁决，不再固定 200Hz 翻 bank。 */
        ManualInputMarkDirty();
        critical = ManualInputEnterCritical();
        if (ManualInputRefreshBusy != 0u || ManualInputStore.ready == 0u)
        {
            ManualInputExitCritical(critical);
            return;
        }
    }

    inactive_bank = (uint8_t)(ManualInputStore.active_bank ^ 1u);
    if (inactive_bank >= MANUAL_INPUT_SNAPSHOT_BANK_COUNT)
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
        uint32_t semantics_seq;
        uint8_t current_bank;
        uint8_t current_source;

        now_tick = xTaskGetTickCount();
        critical = ManualInputEnterCritical();
        dirty_seq = ManualInputDirtySeq;
        publish_seq = ManualInputPublishSeq;
        switch_seq = ManualInputSwitchSeq;
        current_bank = ManualInputStore.active_bank;
        current_source = ManualInputStore.bank[current_bank].activeSource;
        semantics_seq = ManualInputStore.bank[current_bank].semanticsSeq;
        ManualInputExitCritical(critical);

        ManualInputCopySources(ManualInputWorkspace.source,
                               &ManualInputWorkspace.crsf);
        ManualInputCopyConfig(&ManualInputWorkspace.manualConfig,
                              &ManualInputWorkspace.inputConfig);
        ManualInputWorkspace.boardKeyDown = BspKeyReadRawDown();
        ManualInputBuildSnapshot(ManualInputWorkspace.source,
                                 &ManualInputWorkspace.crsf,
                                 &ManualInputWorkspace.manualConfig,
                                 &ManualInputWorkspace.inputConfig,
                                 now_tick,
                                 0u,
                                 current_source,
                                 ManualInputWorkspace.boardKeyDown,
                                 &ManualInputWorkspace.candidate);
        ManualInputWorkspace.candidate.publishSeq = ManualInputSeqNext(publish_seq);
        ManualInputWorkspace.candidate.switchSeq =
            (ManualInputWorkspace.candidate.activeSource != current_source) ?
                ManualInputSeqNext(switch_seq) :
                switch_seq;
        ManualInputWorkspace.candidate.semanticsSeq =
            (memcmp(&ManualInputWorkspace.manualConfig,
                    &ManualInputStore.manual_config[current_bank],
                    sizeof(ManualInputWorkspace.manualConfig)) != 0 ||
             memcmp(&ManualInputWorkspace.inputConfig,
                    &ManualInputStore.input_config[current_bank],
                    sizeof(ManualInputWorkspace.inputConfig)) != 0) ?
                ManualInputSeqNext(semantics_seq) :
                semantics_seq;
        ManualInputWorkspace.candidate.actionSeq =
            ManualInputStore.bank[current_bank].actionSeq;
        ManualInputWorkspace.candidate.authoritySeq =
            ManualInputStore.bank[current_bank].authoritySeq;

        /* reader 全程持有临界区，只会读取活动 bank；非活动 bank 可在外部完整构造。 */
        ManualInputStore.bank[inactive_bank] = ManualInputWorkspace.candidate;
        for (uint8_t source = 0u;
             source <= (uint8_t)MANUAL_INPUT_SRC_MAX;
             source++)
        {
            ManualInputStore.source[inactive_bank][source] = ManualInputWorkspace.source[source];
        }
        ManualInputStore.manual_config[inactive_bank] = ManualInputWorkspace.manualConfig;
        ManualInputStore.input_config[inactive_bank] = ManualInputWorkspace.inputConfig;
        ManualInputStore.crsf[inactive_bank] = ManualInputWorkspace.crsf;
        ManualInputStore.board_key_down[inactive_bank] = ManualInputWorkspace.boardKeyDown;
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
            ManualInputWorkspace.candidate.actionSeq =
                ManualInputActionAuthoritySyncLocked(ManualInputWorkspace.source,
                                                     &ManualInputWorkspace.candidate);
            ManualInputStore.bank[inactive_bank].actionSeq =
                ManualInputWorkspace.candidate.actionSeq;
            ManualInputWorkspace.candidate.authoritySeq =
                ManualInputControlAuthoritySyncLocked(ManualInputWorkspace.source,
                                                      &ManualInputWorkspace.candidate);
            ManualInputStore.bank[inactive_bank].authoritySeq =
                ManualInputWorkspace.candidate.authoritySeq;
            prev_source = manual_active_src;
            ManualInputPublishSeq = ManualInputWorkspace.candidate.publishSeq;
            ManualInputSwitchSeq = ManualInputWorkspace.candidate.switchSeq;
            ManualInputRefreshSeq = dirty_seq;
            ManualInputRefreshDirty = 0u;
            manual_active_src = ManualInputWorkspace.candidate.activeSource;
            ManualInputStore.excluded_mask[inactive_bank] = 0u;
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
        /* busy 覆盖切源日志副作用，保证发布顺序不会被下一 writer 反转。 */
        if (prev_source != ManualInputWorkspace.candidate.activeSource)
        {
            ManualInputLogSourceSwitch(prev_source,
                                       ManualInputWorkspace.candidate.activeSource);
        }
    }

    critical = ManualInputEnterCritical();
    ManualInputRefreshBusy = 0u;
    tail_retry = (uint8_t)(ManualInputRefreshDirty != 0u ||
                           ManualInputDirtySeq != ManualInputRefreshSeq);
    ManualInputExitCritical(critical);

    /* 日志副作用期间到达的新帧最多立即补刷一次，持续流量仍交给 5ms 定时器。 */
    if (tail_retry != 0u && force != 2u)
    {
        ManualInputRefreshIfNeeded(2u);
    }
}

static void ManualInputRefreshTimerCallback(TimerHandle_t timer)
{
    (void)timer;
    ManualInputRefreshIfNeeded(0u);
}

static void ManualInputLogSourceSwitch(uint8_t prev_src, uint8_t next_src)
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

static void ManualInputLogSbusRawFrame(const uint8_t frame[RC_FRAME_LENGTH])
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

    ManualInputLogRawSource(MANUAL_INPUT_SRC_DBUS,
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
