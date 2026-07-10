/*
 * SPDX-FileCopyrightText: 2026 陈轩 <2811158416@qq.com>
 * SPDX-License-Identifier: Apache-2.0
 *
 * First published in this repository: 2026-04-06
 * Use of this file is governed by the LICENSE file in the repository root.
 */


#ifndef MANUAL_INPUT_H
#define MANUAL_INPUT_H
#include <stddef.h>

#include "ManualInputProtocol.h"
#include "Types.h"
#include "BspRc.h"
#include "RobotConfig.h"

#define SBUS_RX_BUF_NUM BSP_RC_SBUS_RX_BUF_NUM

#define RC_FRAME_LENGTH BSP_RC_SBUS_FRAME_LENGTH

#define RC_CH_VALUE_MIN         ((uint16_t)364)
#define RC_CH_VALUE_OFFSET      ((uint16_t)1024)
#define RC_CH_VALUE_MAX         ((uint16_t)1684)
#define RC_CH_VALUE_ABS_LEGACY  ((uint16_t)(RC_CH_VALUE_MAX - RC_CH_VALUE_OFFSET))
#define RC_CH_VALUE_ABS_MAX     ((uint16_t)1024)

#ifndef MANUAL_INPUT_SRC_IMAGE
// Targets without a dedicated image-link input source reuse the reserved USB slot.
#define MANUAL_INPUT_SRC_IMAGE MANUAL_INPUT_SRC_USB
#endif

#ifndef MANUAL_INPUT_SRC_MAX
#define MANUAL_INPUT_SRC_MAX MANUAL_INPUT_SRC_USB
#endif

static __inline int16_t rc_scale_axis_by_abs(int16_t raw, int16_t in_abs_max, int16_t out_abs_max)
{
    int32_t value = raw;
    int32_t in_max = in_abs_max;
    int32_t out_max = out_abs_max;

    if (in_max <= 0 || out_max <= 0)
    {
        return 0;
    }

    if (value > in_max)
    {
        value = in_max;
    }
    else if (value < -in_max)
    {
        value = -in_max;
    }

    value *= out_max;
    if (value >= 0)
    {
        value += in_max / 2;
    }
    else
    {
        value -= in_max / 2;
    }

    return (int16_t)(value / in_max);
}

static __inline uint16_t rc_scale_u16_by_abs(uint16_t value, uint16_t in_abs_max, uint16_t out_abs_max)
{
    uint32_t in_max = in_abs_max;
    uint32_t out_max = out_abs_max;
    uint32_t scaled = value;

    if (in_max == 0u || out_max == 0u)
    {
        return 0u;
    }

    scaled = (scaled * out_max + (in_max / 2u)) / in_max;
    if (scaled > 65535u)
    {
        scaled = 65535u;
    }

    return (uint16_t)scaled;
}

static __inline fp32 rc_scale_fp32_by_abs(fp32 value, fp32 in_abs_max, fp32 out_abs_max)
{
    if (in_abs_max <= 0.0f || out_abs_max <= 0.0f)
    {
        return 0.0f;
    }

    return value * (out_abs_max / in_abs_max);
}

/* ----------------------- RC Switch Definition----------------------------- */
#define RC_SW_UP                ((uint16_t)1)
#define RC_SW_MID               ((uint16_t)3)
#define RC_SW_DOWN              ((uint16_t)2)
#define switch_is_down(s)       (s == RC_SW_DOWN)
#define switch_is_mid(s)        (s == RC_SW_MID)
#define switch_is_up(s)         (s == RC_SW_UP)
/* ----------------------- PC Key Definition-------------------------------- */
#define KEY_PRESSED_OFFSET_W            ((uint16_t)1 << 0)
#define KEY_PRESSED_OFFSET_S            ((uint16_t)1 << 1)
#define KEY_PRESSED_OFFSET_A            ((uint16_t)1 << 2)
#define KEY_PRESSED_OFFSET_D            ((uint16_t)1 << 3)
#define KEY_PRESSED_OFFSET_SHIFT        ((uint16_t)1 << 4)
#define KEY_PRESSED_OFFSET_CTRL         ((uint16_t)1 << 5)
#define KEY_PRESSED_OFFSET_Q            ((uint16_t)1 << 6)
#define KEY_PRESSED_OFFSET_E            ((uint16_t)1 << 7)
#define KEY_PRESSED_OFFSET_R            ((uint16_t)1 << 8)
#define KEY_PRESSED_OFFSET_F            ((uint16_t)1 << 9)
#define KEY_PRESSED_OFFSET_G            ((uint16_t)1 << 10)
#define KEY_PRESSED_OFFSET_Z            ((uint16_t)1 << 11)
#define KEY_PRESSED_OFFSET_X            ((uint16_t)1 << 12)
#define KEY_PRESSED_OFFSET_C            ((uint16_t)1 << 13)
#define KEY_PRESSED_OFFSET_V            ((uint16_t)1 << 14)
#define KEY_PRESSED_OFFSET_B            ((uint16_t)1 << 15)
/* ----------------------- Data Struct ------------------------------------- */
#pragma pack(push, 1)

typedef struct ManualInputRc
{
        int16_t ch[5];
        char s[2];
} ManualInputRc;

typedef struct ManualInputMouse
{
        int16_t x;
        int16_t y;
        int16_t z;
        uint8_t press_l;
        uint8_t press_r;
} ManualInputMouse;

typedef struct ManualInputKey
{
        uint16_t v;
} ManualInputKey;

typedef struct ManualInputState
{
        ManualInputRc rc;
        ManualInputMouse mouse;
        ManualInputKey key;
} ManualInputState;

#pragma pack(pop)

#define MANUAL_INPUT_RC_SIZE_BYTES      12u
#define MANUAL_INPUT_MOUSE_SIZE_BYTES   8u
#define MANUAL_INPUT_KEY_SIZE_BYTES     2u
#define MANUAL_INPUT_STATE_SIZE_BYTES   22u
#define MANUAL_INPUT_RC_OFFSET_BYTES    0u
#define MANUAL_INPUT_MOUSE_OFFSET_BYTES 12u
#define MANUAL_INPUT_KEY_OFFSET_BYTES   20u

typedef char ManualInputRcSizeCheck[(sizeof(ManualInputRc) == MANUAL_INPUT_RC_SIZE_BYTES) ? 1 : -1];
typedef char ManualInputMouseSizeCheck[(sizeof(ManualInputMouse) == MANUAL_INPUT_MOUSE_SIZE_BYTES) ? 1 : -1];
typedef char ManualInputKeySizeCheck[(sizeof(ManualInputKey) == MANUAL_INPUT_KEY_SIZE_BYTES) ? 1 : -1];
typedef char ManualInputStateSizeCheck[(sizeof(ManualInputState) == MANUAL_INPUT_STATE_SIZE_BYTES) ? 1 : -1];
typedef char ManualInputRcOffsetCheck[(offsetof(ManualInputState, rc) == MANUAL_INPUT_RC_OFFSET_BYTES) ? 1 : -1];
typedef char ManualInputMouseOffsetCheck[(offsetof(ManualInputState, mouse) == MANUAL_INPUT_MOUSE_OFFSET_BYTES) ? 1 : -1];
typedef char ManualInputKeyOffsetCheck[(offsetof(ManualInputState, key) == MANUAL_INPUT_KEY_OFFSET_BYTES) ? 1 : -1];

#define MANUAL_INPUT_SOURCE_FLAG_AUTO_AIM (1u << 0)
#define MANUAL_INPUT_SOURCE_FLAG_AUX_FIRE (1u << 1)
#define MANUAL_INPUT_SOURCE_RAW_PAUSE      (1u << 0)
#define MANUAL_INPUT_SOURCE_RAW_MOUSE_R    (1u << 1)
#define MANUAL_INPUT_SOURCE_RAW_BTN_L      (1u << 2)
#define MANUAL_INPUT_SOURCE_RAW_MOUSE_L    (1u << 3)
#define MANUAL_INPUT_SOURCE_RAW_BTN_R      (1u << 4)

/*
 * Manual input layers:
 * - `RcSbusTask.c`: only drains board-level SBUS/DBUS frames and forwards them here.
 * - `ManualInput.c`: arbitrates DBUS/ELRS/Image, unions only typed auxiliary flags, and publishes `ManualInputSnapshot`.
 * - `ControlInput.c`: uses the frozen input config to map the representative state into business axes/switches.
 *
 * If you want to:
 * - change SBUS/DBUS decode: edit `ManualInputDbus.h` and `ManualInputOnSbusFrame()` in `ManualInput.c`
 * - change source arbitration: edit the `ManualInput*` selection helpers
 * - change axis/switch mapping: edit `ControlInput.c` and `Robotconfig/<TARGET>/ConfigInput.inc`
 */

/*
 * 输入发布只允许通过 ManualInputSnapshotRead 读取；这里保留来源写入和诊断计数。
 * 下列 Init/Update/Invalidate/Refresh 接口都只允许任务上下文调用：它们会取 RTOS tick、
 * 更新检测状态并可能完成整帧发布与日志。中断只能把原始字节写入驱动缓冲后通知任务。
 */
extern void ManualInputInit(void);
extern void ManualInputOnSbusFrame(const uint8_t frame[RC_FRAME_LENGTH]);
/* 图传协议与来源固定绑定；语义开关在统一快照构建时按同代配置解释。 */
extern void ManualInputUpdateImageSource(const ManualInputState *rc,
                                         uint8_t protocol,
                                         uint8_t rawFlags,
                                         uint8_t rawSwitch1);
/* CRSF 原始 16 通道与解码值原子入库，配置刷新可用冻结映射重建旧帧。 */
extern void ManualInputUpdateElrsChannels(const ManualInputState *decoded,
                                          const uint16_t raw[16]);
/* CRC 正确但业务字段非法时使用；旧命令立即失效，等待该来源的新合法帧。 */
extern void ManualInputInvalidateSource(uint8_t source);
extern void ManualInputRefresh(void);
extern uint32_t ManualInputGetSbusFrameCount(void);
extern uint32_t ManualInputGetSbusRejectCount(void);
extern uint32_t ManualInputGetSetSourceCount(void);

extern void ManualInputLogRawSource(uint8_t source,
                                    uint8_t proto,
                                    uint8_t range_mode,
                                    uint8_t channel_count,
                                    const int16_t *ch_raw,
                                    const uint8_t sw_raw[2],
                                    const ManualInputState *decoded);
#endif
