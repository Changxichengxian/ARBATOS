/*
 * SPDX-FileCopyrightText: 2026 陈轩 <2811158416@qq.com>
 * SPDX-License-Identifier: Apache-2.0
 *
 * First published in this repository: 2026-04-06
 * Use of this file is governed by the LICENSE file in the repository root.
 */

/*
 * 阅读地图：
 * - 前段：校准表格式、插值/网格工具、蜂鸣器和运行日志。
 * - 中段：从 TF/SD 读取、保存 pitch 补偿表，并支持内置默认表回退。
 * - 后段：PitchCaliTickPre/control/post 组成状态机，驱动每个弹量/角度点采样。
 * - 对外入口：boot_load() 启动加载，tick_pre/control/tick_post 接入云台循环。
 */

#include "config.h"
#include "RobotTaskBuildConfig.h"

#if ROBOT_TASK_BUILD_ANY_GIMBAL

#include "PitchCali.h"

#include <math.h>
#include <stddef.h>
#include <string.h>

#include "BspTime.h"
#include "BspBuzzer.h"
#include "SdLog.h"
#include "user_lib.h"

#include "fatfs/ff.h"
#include "SdCard.h"

#include "Referee.h"

__weak ext_bullet_remaining_t bullet_remaining_t;

#define PITCH_CALI_MAGIC 0x4C414350u // 'PCAL'
#define PITCH_CALI_VERSION 2u

#define PITCH_CALI_MAX_ANGLE_POINTS 33u // HERO built-in table uses 33 points (32 segments)
#define PITCH_CALI_MAX_BULLET_POINTS 8u
#define PITCH_CALI_SEEK_SPEEDUP 1.5f
#define PITCH_CALI_BEEP_PERIOD_MS 2000u
#define PITCH_CALI_BEEP_ON_MS 50u
#define PITCH_CALI_LOG_PERIOD_MS 100u
#define PITCH_CALI_ENDPOINT_SETTLE_MS 1500u
#define PITCH_CALI_ENDPOINT_CMD_EPS 0.005f

typedef struct __attribute__((packed))
{
    uint32_t magic;
    uint16_t version;
    uint8_t angle_points;
    uint8_t bullet_points;
    uint32_t reserved;
} PitchCaliFileHeader;

typedef struct
{
    uint8_t valid;
    uint8_t angle_points;
    uint8_t bullet_points;
    uint16_t completed_cells;
    fp32 angle[PITCH_CALI_MAX_ANGLE_POINTS];
    uint16_t bullet[PITCH_CALI_MAX_BULLET_POINTS];

    int16_t hold_current[PITCH_CALI_MAX_BULLET_POINTS][PITCH_CALI_MAX_ANGLE_POINTS];
    uint16_t kick_up[PITCH_CALI_MAX_BULLET_POINTS][PITCH_CALI_MAX_ANGLE_POINTS];
    uint16_t kick_down[PITCH_CALI_MAX_BULLET_POINTS][PITCH_CALI_MAX_ANGLE_POINTS];
} PitchCaliTable;

typedef enum
{
    PITCH_CALI_STATE_IDLE = 0,
    PITCH_CALI_STATE_WAIT_BULLET,
    PITCH_CALI_STATE_MOVE_TO_ANGLE,
    PITCH_CALI_STATE_HOLD_AVG,
    PITCH_CALI_STATE_BREAKAWAY_UP,
    PITCH_CALI_STATE_RECOVER_UP,
    PITCH_CALI_STATE_BREAKAWAY_DOWN,
    PITCH_CALI_STATE_RECOVER_DOWN,
    PITCH_CALI_STATE_SAVE,
    PITCH_CALI_STATE_DONE,
    PITCH_CALI_STATE_ERROR,
} PitchCaliState;

typedef struct
{
    PitchCaliTable table;

    uint8_t last_active;
    GimbalBehaviour last_behaviour;

    PitchCaliState state;
    uint8_t running;

    uint8_t angle_idx;
    uint8_t bullet_idx;
    fp32 target_angle;
    fp32 cmd_angle;
    uint16_t target_bullet;

    uint32_t state_enter_ms;
    uint32_t stable_enter_ms;

    fp32 hold_sum;
    uint32_t hold_cnt;
    fp32 hold_avg;

    fp32 delta;
    uint32_t last_step_ms;
    fp32 breakaway_base_angle;
    uint32_t beep_toggle_ms;
    uint32_t last_log_ms;
    uint8_t beep_on;

    fp32 raw_current_cmd;
    int32_t last_error;
    sdlog_pitch_cali_t log;
} PitchCaliCtx;

static PitchCaliCtx s_ctx;
static PitchCaliTable s_boot_table_tmp;

__weak bool_t PitchCaliGetBuiltinDefault(PitchCaliBuiltinDesc *out)
{
    if (out != NULL)
    {
        memset(out, 0, sizeof(*out));
    }
    return 0;
}

static uint16_t PitchCaliBulletCountNow(void);
static bool_t PitchCaliBulletReady(uint16_t now, uint16_t target);
static bool_t PitchCaliIsStable(fp32 angle, fp32 gyro, fp32 target, const PitchCaliConfig *cfg);
static bool_t PitchCaliTrySnapEndpointTarget(fp32 angle, fp32 gyro, const PitchCaliConfig *cfg, uint32_t now);
static bool_t PitchCaliLoadBuiltinDefault(PitchCaliTable *out);

static uint16_t PitchCaliGridCountFromDims(uint8_t angle_points, uint8_t bullet_points)
{
    return (uint16_t)((uint16_t)angle_points * (uint16_t)bullet_points);
}

static bool_t PitchCaliLoadBuiltinDefault(PitchCaliTable *out)
{
    PitchCaliBuiltinDesc builtin = {0};
    uint16_t gridN = 0u;

    if (out == NULL)
    {
        return 0;
    }
    if (PitchCaliGetBuiltinDefault(&builtin) == 0)
    {
        return 0;
    }
    if (builtin.angle_points < 2u || builtin.angle_points > PITCH_CALI_MAX_ANGLE_POINTS ||
        builtin.bullet_points < 1u || builtin.bullet_points > PITCH_CALI_MAX_BULLET_POINTS ||
        builtin.angle == NULL || builtin.bullet == NULL ||
        builtin.hold_current == NULL || builtin.kick_up == NULL || builtin.kick_down == NULL)
    {
        return 0;
    }

    gridN = PitchCaliGridCountFromDims(builtin.angle_points, builtin.bullet_points);

    memset(out, 0, sizeof(*out));
    out->valid = 1u;
    out->angle_points = builtin.angle_points;
    out->bullet_points = builtin.bullet_points;
    out->completed_cells = gridN;

    memcpy(out->bullet, builtin.bullet, sizeof(uint16_t) * builtin.bullet_points);
    memcpy(out->angle, builtin.angle, sizeof(fp32) * builtin.angle_points);
    memcpy(out->hold_current, builtin.hold_current, sizeof(int16_t) * gridN);
    memcpy(out->kick_up, builtin.kick_up, sizeof(uint16_t) * gridN);
    memcpy(out->kick_down, builtin.kick_down, sizeof(uint16_t) * gridN);
    return 1;
}

static uint16_t PitchCaliGridCount(const PitchCaliTable *tab)
{
    if (tab == NULL)
    {
        return 0u;
    }
    return PitchCaliGridCountFromDims(tab->angle_points, tab->bullet_points);
}

static uint16_t PitchCaliGridIndex(const PitchCaliTable *tab, uint8_t bullet_idx, uint8_t angle_idx)
{
    if (tab == NULL || tab->angle_points == 0u)
    {
        return 0u;
    }
    return (uint16_t)(((uint16_t)bullet_idx * (uint16_t)tab->angle_points) + (uint16_t)angle_idx);
}

static void PitchCaliResumeIndicesFromCompleted(const PitchCaliTable *tab,
                                                     uint8_t *bullet_idx,
                                                     uint8_t *angle_idx)
{
    if (tab == NULL || bullet_idx == NULL || angle_idx == NULL || tab->angle_points == 0u)
    {
        return;
    }

    const uint16_t gridN = PitchCaliGridCount(tab);
    uint16_t completed = tab->completed_cells;
    if (completed > gridN)
    {
        completed = gridN;
    }

    *bullet_idx = (uint8_t)(completed / tab->angle_points);
    *angle_idx = (uint8_t)(completed % tab->angle_points);
}

static bool_t PitchCaliGridLayoutMatches(const PitchCaliTable *saved, const PitchCaliTable *fresh)
{
    if (saved == NULL || fresh == NULL)
    {
        return 0;
    }

    if (saved->angle_points != fresh->angle_points || saved->bullet_points != fresh->bullet_points)
    {
        return 0;
    }

    for (uint8_t i = 0u; i < fresh->angle_points; i++)
    {
        if (fabsf(saved->angle[i] - fresh->angle[i]) > 1e-6f)
        {
            return 0;
        }
    }

    for (uint8_t i = 0u; i < fresh->bullet_points; i++)
    {
        if (saved->bullet[i] != fresh->bullet[i])
        {
            return 0;
        }
    }

    return 1;
}

static bool_t PitchCaliBeepShouldRun(PitchCaliState state)
{
    return (state == PITCH_CALI_STATE_WAIT_BULLET ||
            state == PITCH_CALI_STATE_MOVE_TO_ANGLE ||
            state == PITCH_CALI_STATE_HOLD_AVG ||
            state == PITCH_CALI_STATE_RECOVER_UP ||
            state == PITCH_CALI_STATE_RECOVER_DOWN)
               ? 1
               : 0;
}

static void PitchCaliBuzzerUpdate(uint32_t now, bool_t active)
{
    if (!active)
    {
        if (s_ctx.beep_on != 0u)
        {
            BuzzerToneStop();
            s_ctx.beep_on = 0u;
        }
        s_ctx.beep_toggle_ms = now;
        return;
    }

    if (s_ctx.beep_toggle_ms == 0u)
    {
        s_ctx.beep_toggle_ms = now;
    }

    if (s_ctx.beep_on != 0u)
    {
        if ((uint32_t)(now - s_ctx.beep_toggle_ms) >= PITCH_CALI_BEEP_ON_MS)
        {
            BuzzerToneStop();
            s_ctx.beep_on = 0u;
            s_ctx.beep_toggle_ms = now;
        }
        return;
    }

    if ((uint32_t)(now - s_ctx.beep_toggle_ms) >= PITCH_CALI_BEEP_PERIOD_MS)
    {
        BuzzerToneStartLegacy(g_config.buzzer.soft_beep_psc, BuzzerLegacyPwmHalf());
        s_ctx.beep_on = 1u;
        s_ctx.beep_toggle_ms = now;
    }
}

static void PitchCaliLogRuntime(const GimbalControl *gimbal,
                                   const PitchCaliConfig *cfg,
                                   uint32_t now,
                                   fp32 angle,
                                   fp32 gyro,
                                   fp32 current)
{
    if (gimbal == NULL || cfg == NULL)
    {
        return;
    }
    if (SdLogIsActive() == 0)
    {
        return;
    }
    if (s_ctx.running == 0u)
    {
        return;
    }
    if (s_ctx.last_log_ms != 0u && (uint32_t)(now - s_ctx.last_log_ms) < PITCH_CALI_LOG_PERIOD_MS)
    {
        return;
    }

    const uint16_t bullet_now = PitchCaliBulletCountNow();
    memset(&s_ctx.log, 0, sizeof(s_ctx.log));
    s_ctx.log.state = (uint8_t)s_ctx.state;
    s_ctx.log.angle_idx = s_ctx.angle_idx;
    s_ctx.log.bullet_idx = s_ctx.bullet_idx;
    s_ctx.log.angle_points = s_ctx.table.angle_points;
    s_ctx.log.bullet_points = s_ctx.table.bullet_points;
    s_ctx.log.bullet_ready = PitchCaliBulletReady(bullet_now, s_ctx.target_bullet);
    s_ctx.log.is_stable = PitchCaliIsStable(angle, gyro, s_ctx.target_angle, cfg);
    s_ctx.log.motor_raw_mode = (gimbal->GimbalPitchMotor.mode == GIMBAL_MOTOR_RAW) ? 1u : 0u;
    s_ctx.log.target_bullet = s_ctx.target_bullet;
    s_ctx.log.bullet_now = bullet_now;
    s_ctx.log.completed_cells = s_ctx.table.completed_cells;
    s_ctx.log.grid_cells = PitchCaliGridCount(&s_ctx.table);
    s_ctx.log.state_elapsed_ms = (s_ctx.state_enter_ms == 0u) ? 0u : (uint32_t)(now - s_ctx.state_enter_ms);
    s_ctx.log.stable_elapsed_ms = (s_ctx.stable_enter_ms == 0u) ? 0u : (uint32_t)(now - s_ctx.stable_enter_ms);
    s_ctx.log.last_error = s_ctx.last_error;
    s_ctx.log.target_angle = s_ctx.target_angle;
    s_ctx.log.cmd_angle = s_ctx.cmd_angle;
    s_ctx.log.angle = angle;
    s_ctx.log.gyro = gyro;
    s_ctx.log.current = current;
    s_ctx.log.hold_avg = s_ctx.hold_avg;
    s_ctx.log.raw_current_cmd = s_ctx.raw_current_cmd;
    s_ctx.log.delta = s_ctx.delta;

    SdLogWrite(SDLOG_TAG_PITCH_CALI, &s_ctx.log, (uint16_t)sizeof(s_ctx.log));
    s_ctx.last_log_ms = now;
}

static uint32_t crc32_ieee_update(uint32_t crc, const uint8_t *data, uint32_t len)
{
    static const uint32_t t[16] = {
        0x00000000u, 0x1DB71064u, 0x3B6E20C8u, 0x26D930ACu,
        0x76DC4190u, 0x6B6B51F4u, 0x4DB26158u, 0x5005713Cu,
        0xEDB88320u, 0xF00F9344u, 0xD6D6A3E8u, 0xCB61B38Cu,
        0x9B64C2B0u, 0x86D3D2D4u, 0xA00AE278u, 0xBDBDF21Cu,
    };

    uint32_t c = crc;
    for (uint32_t i = 0; i < len; i++)
    {
        const uint8_t b = data[i];
        c ^= b;
        c = t[c & 0x0Fu] ^ (c >> 4);
        c = t[c & 0x0Fu] ^ (c >> 4);
    }
    return c;
}

static uint32_t crc32_ieee_finish(uint32_t crc)
{
    return crc ^ 0xFFFFFFFFu;
}

static fp32 PitchCaliLimitPitchCurrent(fp32 current)
{
    const fp32 limit = fabsf(g_config.gimbal.pitch_current_limit);
    if (limit <= 0.0f)
    {
        return current;
    }
    if (current > limit)
    {
        return limit;
    }
    if (current < -limit)
    {
        return -limit;
    }
    return current;
}

static uint16_t PitchCaliBulletCountNow(void)
{
    const PitchCaliConfig *cfg = &g_config.gimbal.pitch_cali;
    if (cfg->bullet_source == (uint8_t)PITCH_CALI_BULLET_SRC_REFEREE)
    {
        return bullet_remaining_t.projectile_allowance_17mm;
    }
    return cfg->bullet_manual;
}

uint16_t PitchCaliGetRuntimeBulletCount(void)
{
    return PitchCaliBulletCountNow();
}

bool_t PitchCaliIsRunning(void)
{
    return (s_ctx.running != 0u) ? 1u : 0u;
}

static fp32 PitchCaliLerp(fp32 a, fp32 b, fp32 t)
{
    return a + (b - a) * t;
}

static int32_t PitchCaliRoundI32(fp32 x)
{
    return (x >= 0.0f) ? (int32_t)(x + 0.5f) : (int32_t)(x - 0.5f);
}

static uint16_t PitchCaliRoundU16Sat(fp32 x)
{
    if (x <= 0.0f)
    {
        return 0u;
    }
    const fp32 v = x + 0.5f;
    if (v >= 65535.0f)
    {
        return 65535u;
    }
    return (uint16_t)v;
}

static fp32 PitchCaliInterp1AngleS16(const fp32 *xs, const int16_t *ys, uint8_t n, fp32 x)
{
    if (xs == NULL || ys == NULL || n == 0u)
    {
        return 0.0f;
    }
    if (n == 1u)
    {
        return (fp32)ys[0];
    }
    if (x <= xs[0])
    {
        return (fp32)ys[0];
    }
    if (x >= xs[n - 1u])
    {
        return (fp32)ys[n - 1u];
    }
    for (uint8_t i = 0u; i < (uint8_t)(n - 1u); i++)
    {
        const fp32 x0 = xs[i];
        const fp32 x1 = xs[i + 1u];
        if (x >= x0 && x <= x1)
        {
            const fp32 t = (x1 > x0) ? ((x - x0) / (x1 - x0)) : 0.0f;
            return PitchCaliLerp((fp32)ys[i], (fp32)ys[i + 1u], t);
        }
    }
    return (fp32)ys[n - 1u];
}

static fp32 PitchCaliInterp1AngleU16(const fp32 *xs, const uint16_t *ys, uint8_t n, fp32 x)
{
    if (xs == NULL || ys == NULL || n == 0u)
    {
        return 0.0f;
    }
    if (n == 1u)
    {
        return (fp32)ys[0];
    }
    if (x <= xs[0])
    {
        return (fp32)ys[0];
    }
    if (x >= xs[n - 1u])
    {
        return (fp32)ys[n - 1u];
    }
    for (uint8_t i = 0u; i < (uint8_t)(n - 1u); i++)
    {
        const fp32 x0 = xs[i];
        const fp32 x1 = xs[i + 1u];
        if (x >= x0 && x <= x1)
        {
            const fp32 t = (x1 > x0) ? ((x - x0) / (x1 - x0)) : 0.0f;
            return PitchCaliLerp((fp32)ys[i], (fp32)ys[i + 1u], t);
        }
    }
    return (fp32)ys[n - 1u];
}

static fp32 PitchCaliInterp1BulletF32(const uint16_t *xs, const fp32 *ys, uint8_t n, uint16_t x)
{
    if (xs == NULL || ys == NULL || n == 0u)
    {
        return 0.0f;
    }
    if (n == 1u)
    {
        return ys[0];
    }
    if (x <= xs[0])
    {
        return ys[0];
    }
    if (x >= xs[n - 1u])
    {
        return ys[n - 1u];
    }
    for (uint8_t i = 0u; i < (uint8_t)(n - 1u); i++)
    {
        const uint16_t x0 = xs[i];
        const uint16_t x1 = xs[i + 1u];
        if (x >= x0 && x <= x1)
        {
            const fp32 t = (x1 > x0) ? ((fp32)(x - x0) / (fp32)(x1 - x0)) : 0.0f;
            return PitchCaliLerp(ys[i], ys[i + 1u], t);
        }
    }
    return ys[n - 1u];
}

static void PitchCaliEvalComp(const PitchCaliTable *tab,
                                 fp32 angle,
                                 uint16_t bullet,
                                 fp32 *hold,
                                 fp32 *kick_up,
                                 fp32 *kick_down)
{
    fp32 hold_by_b[PITCH_CALI_MAX_BULLET_POINTS] = {0};
    fp32 up_by_b[PITCH_CALI_MAX_BULLET_POINTS] = {0};
    fp32 down_by_b[PITCH_CALI_MAX_BULLET_POINTS] = {0};

    const uint8_t aN = tab->angle_points;
    const uint8_t bN = tab->bullet_points;

    for (uint8_t b = 0u; b < bN; b++)
    {
        hold_by_b[b] = PitchCaliInterp1AngleS16(tab->angle, tab->hold_current[b], aN, angle);
        up_by_b[b] = PitchCaliInterp1AngleU16(tab->angle, tab->kick_up[b], aN, angle);
        down_by_b[b] = PitchCaliInterp1AngleU16(tab->angle, tab->kick_down[b], aN, angle);
    }

    if (hold != NULL)
    {
        *hold = PitchCaliInterp1BulletF32(tab->bullet, hold_by_b, bN, bullet);
    }
    if (kick_up != NULL)
    {
        *kick_up = PitchCaliInterp1BulletF32(tab->bullet, up_by_b, bN, bullet);
    }
    if (kick_down != NULL)
    {
        *kick_down = PitchCaliInterp1BulletF32(tab->bullet, down_by_b, bN, bullet);
    }
}

bool_t PitchCaliGetComp(fp32 pitch_angle,
                           fp32 *hold_current,
                           fp32 *kick_up_current,
                           fp32 *kick_down_current)
{
    if (hold_current)
    {
        *hold_current = 0.0f;
    }
    if (kick_up_current)
    {
        *kick_up_current = 0.0f;
    }
    if (kick_down_current)
    {
        *kick_down_current = 0.0f;
    }

    if (PitchCaliIsRunning())
    {
        return 0;
    }

    if (g_config.gimbal.pitch_cali.enable == 0u)
    {
        return 0;
    }
    if (s_ctx.table.valid == 0u)
    {
        return 0;
    }

    const uint16_t bullet = PitchCaliBulletCountNow();
    PitchCaliEvalComp(&s_ctx.table, pitch_angle, bullet, hold_current, kick_up_current, kick_down_current);
    return 1;
}

static int PitchCaliLoadFromSd(PitchCaliTable *out)
{
    if (out == NULL)
    {
        return -1;
    }

    // 函数地图：读文件头校验版本/尺寸，再读表体；校验失败时返回错误码给启动加载逻辑。
    FIL fp;
    FRESULT fr = f_open(&fp, PITCH_CALI_FILE_PATH, FA_READ);
    if (fr != FR_OK)
    {
        return (int)fr;
    }

    PitchCaliFileHeader hdr;
    UINT br = 0u;
    fr = f_read(&fp, &hdr, (UINT)sizeof(hdr), &br);
    if (fr != FR_OK || br != (UINT)sizeof(hdr))
    {
        (void)f_close(&fp);
        return -2;
    }

    if (hdr.magic != PITCH_CALI_MAGIC)
    {
        (void)f_close(&fp);
        return -3;
    }

    if (hdr.version != 1u && hdr.version != (uint16_t)PITCH_CALI_VERSION)
    {
        (void)f_close(&fp);
        return -3;
    }

    if (hdr.angle_points < 2u || hdr.angle_points > PITCH_CALI_MAX_ANGLE_POINTS ||
        hdr.bullet_points < 1u || hdr.bullet_points > PITCH_CALI_MAX_BULLET_POINTS)
    {
        (void)f_close(&fp);
        return -4;
    }

    memset(out, 0, sizeof(*out));
    out->angle_points = hdr.angle_points;
    out->bullet_points = hdr.bullet_points;

    uint32_t crc = 0xFFFFFFFFu;
    crc = crc32_ieee_update(crc, (const uint8_t *)&hdr, (uint32_t)sizeof(hdr));

    fr = f_read(&fp, out->bullet, (UINT)(sizeof(uint16_t) * out->bullet_points), &br);
    if (fr != FR_OK || br != (UINT)(sizeof(uint16_t) * out->bullet_points))
    {
        (void)f_close(&fp);
        return -5;
    }
    crc = crc32_ieee_update(crc, (const uint8_t *)out->bullet, (uint32_t)(sizeof(uint16_t) * out->bullet_points));

    fr = f_read(&fp, out->angle, (UINT)(sizeof(fp32) * out->angle_points), &br);
    if (fr != FR_OK || br != (UINT)(sizeof(fp32) * out->angle_points))
    {
        (void)f_close(&fp);
        return -6;
    }
    crc = crc32_ieee_update(crc, (const uint8_t *)out->angle, (uint32_t)(sizeof(fp32) * out->angle_points));

    const UINT gridN = (UINT)out->bullet_points * (UINT)out->angle_points;
    const uint16_t completed_cells = (hdr.version == 1u) ? (uint16_t)gridN : (uint16_t)hdr.reserved;
    if (completed_cells > (uint16_t)gridN)
    {
        (void)f_close(&fp);
        return -12;
    }

    fr = f_read(&fp, out->hold_current, (UINT)(sizeof(int16_t) * gridN), &br);
    if (fr != FR_OK || br != (UINT)(sizeof(int16_t) * gridN))
    {
        (void)f_close(&fp);
        return -7;
    }
    crc = crc32_ieee_update(crc, (const uint8_t *)out->hold_current, (uint32_t)(sizeof(int16_t) * gridN));

    fr = f_read(&fp, out->kick_up, (UINT)(sizeof(uint16_t) * gridN), &br);
    if (fr != FR_OK || br != (UINT)(sizeof(uint16_t) * gridN))
    {
        (void)f_close(&fp);
        return -8;
    }
    crc = crc32_ieee_update(crc, (const uint8_t *)out->kick_up, (uint32_t)(sizeof(uint16_t) * gridN));

    fr = f_read(&fp, out->kick_down, (UINT)(sizeof(uint16_t) * gridN), &br);
    if (fr != FR_OK || br != (UINT)(sizeof(uint16_t) * gridN))
    {
        (void)f_close(&fp);
        return -9;
    }
    crc = crc32_ieee_update(crc, (const uint8_t *)out->kick_down, (uint32_t)(sizeof(uint16_t) * gridN));

    uint32_t crc_file = 0u;
    fr = f_read(&fp, &crc_file, (UINT)sizeof(crc_file), &br);
    (void)f_close(&fp);
    if (fr != FR_OK || br != (UINT)sizeof(crc_file))
    {
        return -10;
    }

    crc = crc32_ieee_finish(crc);
    if (crc != crc_file)
    {
        return -11;
    }

    out->completed_cells = completed_cells;
    out->valid = (completed_cells >= (uint16_t)gridN) ? 1u : 0u;
    return 0;
}

static int PitchCaliSaveToSd(const PitchCaliTable *tab, uint16_t completed_cells)
{
    if (tab == NULL)
    {
        return -1;
    }

    const uint16_t gridN = PitchCaliGridCount(tab);
    if (gridN == 0u || completed_cells > gridN)
    {
        return -1;
    }

    FIL fp;
    FRESULT fr = f_open(&fp, PITCH_CALI_FILE_PATH, FA_CREATE_ALWAYS | FA_WRITE);
    if (fr != FR_OK)
    {
        return (int)fr;
    }

    PitchCaliFileHeader hdr = {0};
    hdr.magic = PITCH_CALI_MAGIC;
    hdr.version = (uint16_t)PITCH_CALI_VERSION;
    hdr.angle_points = tab->angle_points;
    hdr.bullet_points = tab->bullet_points;
    hdr.reserved = (uint32_t)completed_cells;

    uint32_t crc = 0xFFFFFFFFu;

    UINT bw = 0u;
    fr = f_write(&fp, &hdr, (UINT)sizeof(hdr), &bw);
    if (fr != FR_OK || bw != (UINT)sizeof(hdr))
    {
        (void)f_close(&fp);
        return -2;
    }
    crc = crc32_ieee_update(crc, (const uint8_t *)&hdr, (uint32_t)sizeof(hdr));

    const UINT bullets_bytes = (UINT)(sizeof(uint16_t) * tab->bullet_points);
    fr = f_write(&fp, tab->bullet, bullets_bytes, &bw);
    if (fr != FR_OK || bw != bullets_bytes)
    {
        (void)f_close(&fp);
        return -3;
    }
    crc = crc32_ieee_update(crc, (const uint8_t *)tab->bullet, (uint32_t)bullets_bytes);

    const UINT angles_bytes = (UINT)(sizeof(fp32) * tab->angle_points);
    fr = f_write(&fp, tab->angle, angles_bytes, &bw);
    if (fr != FR_OK || bw != angles_bytes)
    {
        (void)f_close(&fp);
        return -4;
    }
    crc = crc32_ieee_update(crc, (const uint8_t *)tab->angle, (uint32_t)angles_bytes);

    const UINT gridN_u = (UINT)gridN;

    const UINT hold_bytes = (UINT)(sizeof(int16_t) * gridN_u);
    fr = f_write(&fp, tab->hold_current, hold_bytes, &bw);
    if (fr != FR_OK || bw != hold_bytes)
    {
        (void)f_close(&fp);
        return -5;
    }
    crc = crc32_ieee_update(crc, (const uint8_t *)tab->hold_current, (uint32_t)hold_bytes);

    const UINT up_bytes = (UINT)(sizeof(uint16_t) * gridN_u);
    fr = f_write(&fp, tab->kick_up, up_bytes, &bw);
    if (fr != FR_OK || bw != up_bytes)
    {
        (void)f_close(&fp);
        return -6;
    }
    crc = crc32_ieee_update(crc, (const uint8_t *)tab->kick_up, (uint32_t)up_bytes);

    const UINT down_bytes = (UINT)(sizeof(uint16_t) * gridN_u);
    fr = f_write(&fp, tab->kick_down, down_bytes, &bw);
    if (fr != FR_OK || bw != down_bytes)
    {
        (void)f_close(&fp);
        return -7;
    }
    crc = crc32_ieee_update(crc, (const uint8_t *)tab->kick_down, (uint32_t)down_bytes);

    const uint32_t crc_file = crc32_ieee_finish(crc);
    fr = f_write(&fp, &crc_file, (UINT)sizeof(crc_file), &bw);
    if (fr != FR_OK || bw != (UINT)sizeof(crc_file))
    {
        (void)f_close(&fp);
        return -8;
    }

    (void)f_sync(&fp);
    (void)f_close(&fp);
    return 0;
}

static void PitchCaliStateReset(void)
{
    // Keep the (possibly already loaded) calibration table, only reset runtime state.
    const size_t off = offsetof(PitchCaliCtx, last_active);
    memset((uint8_t *)&s_ctx + off, 0, sizeof(s_ctx) - off);
    s_ctx.last_active = 0u;
    s_ctx.last_behaviour = GIMBAL_ZERO_FORCE;
    s_ctx.state = PITCH_CALI_STATE_IDLE;
    s_ctx.beep_on = 0u;
    s_ctx.beep_toggle_ms = 0u;
    BuzzerToneStop();
}

static void PitchCaliPrepareGrid(PitchCaliTable *tab)
{
    const PitchCaliConfig *cfg = &g_config.gimbal.pitch_cali;
    const uint8_t aN_cfg = cfg->angle_points;
    const uint8_t bN_cfg = cfg->bullet_points;

    const uint8_t aN =
        (aN_cfg < 2u) ? 2u : ((aN_cfg > PITCH_CALI_MAX_ANGLE_POINTS) ? PITCH_CALI_MAX_ANGLE_POINTS : aN_cfg);
    const uint8_t bN =
        (bN_cfg < 1u) ? 1u : ((bN_cfg > PITCH_CALI_MAX_BULLET_POINTS) ? PITCH_CALI_MAX_BULLET_POINTS : bN_cfg);

    memset(tab, 0, sizeof(*tab));
    tab->valid = 0u;
    tab->angle_points = aN;
    tab->bullet_points = bN;
    tab->completed_cells = 0u;

    uint16_t bmin = cfg->bullet_min;
    uint16_t bmax = cfg->bullet_max;
    if (bmax < bmin)
    {
        const uint16_t t = bmax;
        bmax = bmin;
        bmin = t;
    }
    if (bN == 1u)
    {
        tab->bullet[0] = PitchCaliBulletCountNow();
    }
    else
    {
        for (uint8_t i = 0u; i < bN; i++)
        {
            const fp32 t = (bN > 1u) ? ((fp32)i / (fp32)(bN - 1u)) : 0.0f;
            const fp32 v = (fp32)bmin + ((fp32)(bmax - bmin) * t);
            uint16_t u = (uint16_t)((v < 0.0f) ? 0.0f : (v + 0.5f));
            if (u < bmin)
            {
                u = bmin;
            }
            if (u > bmax)
            {
                u = bmax;
            }
            tab->bullet[i] = u;
        }
    }

    fp32 amin = g_config.gimbal.pitch_soft_limit_down;
    fp32 amax = g_config.gimbal.pitch_soft_limit_up;
    if (amax < amin)
    {
        const fp32 t = amax;
        amax = amin;
        amin = t;
    }
    const fp32 margin = fabsf(cfg->angle_margin);
    amin += margin;
    amax -= margin;
    if (amax < amin)
    {
        const fp32 mid = (amax + amin) * 0.5f;
        amin = mid;
        amax = mid;
    }

    if (aN == 1u)
    {
        tab->angle[0] = (amin + amax) * 0.5f;
    }
    else
    {
        for (uint8_t i = 0u; i < aN; i++)
        {
            const fp32 t = (aN > 1u) ? ((fp32)i / (fp32)(aN - 1u)) : 0.0f;
            tab->angle[i] = PitchCaliLerp(amin, amax, t);
        }
    }
}

void PitchCaliBootLoad(void)
{
    PitchCaliStateReset();
    if (g_config.gimbal.pitch_cali.enable == 0u)
    {
        return;
    }

    (void)PitchCaliLoadBuiltinDefault(&s_ctx.table);

    if (SdcardIsMounted() == 0)
    {
        (void)SdcardMount();
    }

    if (PitchCaliLoadFromSd(&s_boot_table_tmp) == 0)
    {
        memcpy(&s_ctx.table, &s_boot_table_tmp, sizeof(s_ctx.table));
    }
}

static bool_t PitchCaliBulletReady(uint16_t now, uint16_t target)
{
    const uint16_t diff = (now > target) ? (uint16_t)(now - target) : (uint16_t)(target - now);
    return (diff <= 1u) ? 1u : 0u;
}

static bool_t PitchCaliIsStable(fp32 angle, fp32 gyro, fp32 target, const PitchCaliConfig *cfg)
{
    return ((fabsf(angle - target) <= fabsf(cfg->stable_angle_err)) && (fabsf(gyro) <= fabsf(cfg->stable_gyro_err))) ? 1u : 0u;
}

static void PitchCaliSeekCmdAngle(fp32 angle, const PitchCaliConfig *cfg)
{
    if (cfg == NULL)
    {
        return;
    }

    const fp32 ki = cfg->seek_k;
    if (ki == 0.0f)
    {
        return;
    }

    fp32 amin = g_config.gimbal.pitch_soft_limit_down;
    fp32 amax = g_config.gimbal.pitch_soft_limit_up;
    if (amax < amin)
    {
        const fp32 t = amax;
        amax = amin;
        amin = t;
    }

    const fp32 dt_scale = (fp32)((g_config.gimbal.control_period_ms == 0u) ? 1u : g_config.gimbal.control_period_ms);
    const fp32 err = s_ctx.target_angle - angle;
    s_ctx.cmd_angle += (err * ki * dt_scale * PITCH_CALI_SEEK_SPEEDUP);
    s_ctx.cmd_angle = fp32_constrain(s_ctx.cmd_angle, amin, amax);
}

static void PitchCaliStateEnter(PitchCaliState st, uint32_t now)
{
    s_ctx.state = st;
    s_ctx.state_enter_ms = now;
    s_ctx.stable_enter_ms = 0u;
    s_ctx.last_log_ms = 0u;
}

static bool_t PitchCaliTrySnapEndpointTarget(fp32 angle, fp32 gyro, const PitchCaliConfig *cfg, uint32_t now)
{
    if (cfg == NULL || s_ctx.table.angle_points < 2u)
    {
        return 0;
    }

    const bool_t is_first = (s_ctx.angle_idx == 0u) ? 1u : 0u;
    const bool_t is_last = ((uint8_t)(s_ctx.angle_idx + 1u) >= s_ctx.table.angle_points) ? 1u : 0u;
    if (!is_first && !is_last)
    {
        return 0;
    }

    if ((uint32_t)(now - s_ctx.state_enter_ms) < PITCH_CALI_ENDPOINT_SETTLE_MS)
    {
        return 0;
    }

    fp32 amin = g_config.gimbal.pitch_soft_limit_down;
    fp32 amax = g_config.gimbal.pitch_soft_limit_up;
    if (amax < amin)
    {
        const fp32 t = amax;
        amax = amin;
        amin = t;
    }

    const bool_t cmd_saturated =
        (is_first && fabsf(s_ctx.cmd_angle - amin) <= PITCH_CALI_ENDPOINT_CMD_EPS) ||
        (is_last && fabsf(s_ctx.cmd_angle - amax) <= PITCH_CALI_ENDPOINT_CMD_EPS);
    if (!cmd_saturated)
    {
        return 0;
    }

    if (fabsf(gyro) > (fabsf(cfg->stable_gyro_err) * 1.5f))
    {
        return 0;
    }

    const fp32 err = s_ctx.target_angle - angle;
    const fp32 err_abs = fabsf(err);
    if (err_abs <= fabsf(cfg->stable_angle_err))
    {
        return 0;
    }

    if ((is_first && err >= 0.0f) || (is_last && err <= 0.0f))
    {
        return 0;
    }

    s_ctx.target_angle = angle;
    s_ctx.cmd_angle = angle;
    s_ctx.table.angle[s_ctx.angle_idx] = angle;
    s_ctx.stable_enter_ms = 0u;
    return 1;
}

void PitchCaliTickPre(GimbalControl *gimbal, GimbalBehaviour behaviour, uint8_t PitchCaliMode)
{
    (void)gimbal;

    // 函数地图：先确认校准入口和 SD 卡，再准备/恢复网格，最后决定本周期 pitch 电机控制方式。
    if (PitchCaliMode == 0u)
    {
        PitchCaliBuzzerUpdate(BspTimeGetTickMs(), 0);
        s_ctx.running = 0u;
        s_ctx.last_active = 0u;
        s_ctx.last_behaviour = behaviour;
        if (s_ctx.state != PITCH_CALI_STATE_IDLE)
        {
            PitchCaliStateEnter(PITCH_CALI_STATE_IDLE, BspTimeGetTickMs());
        }
        return;
    }

    if (SdcardIsMounted() == 0)
    {
        const int m = SdcardMount();
        if (m != 0)
        {
            PitchCaliBuzzerUpdate(BspTimeGetTickMs(), 0);
            s_ctx.running = 0u;
            s_ctx.last_error = m;
            PitchCaliStateEnter(PITCH_CALI_STATE_ERROR, BspTimeGetTickMs());
            s_ctx.last_active = PitchCaliMode;
            s_ctx.last_behaviour = behaviour;
            return;
        }
    }

    const bool_t can_run = (behaviour == GIMBAL_PITCH_CALI) ? 1u : 0u;
    if (!can_run)
    {
        PitchCaliBuzzerUpdate(BspTimeGetTickMs(), 0);
        s_ctx.running = 0u;
        s_ctx.last_active = PitchCaliMode;
        s_ctx.last_behaviour = behaviour;
        return;
    }

    if (gimbal == NULL)
    {
        return;
    }

    const uint32_t now = BspTimeGetTickMs();

    if (s_ctx.state == PITCH_CALI_STATE_IDLE)
    {
        PitchCaliTable fresh = {0};
        PitchCaliPrepareGrid(&fresh);

        const uint16_t fresh_gridN = PitchCaliGridCount(&fresh);
        const bool_t can_resume = (s_ctx.table.completed_cells > 0u) &&
                                  (s_ctx.table.completed_cells < fresh_gridN) &&
                                  PitchCaliGridLayoutMatches(&s_ctx.table, &fresh);

        if (!can_resume)
        {
            memcpy(&s_ctx.table, &fresh, sizeof(s_ctx.table));
        }

        if (s_ctx.table.completed_cells >= PitchCaliGridCount(&s_ctx.table))
        {
            memcpy(&s_ctx.table, &fresh, sizeof(s_ctx.table));
        }

        PitchCaliResumeIndicesFromCompleted(&s_ctx.table, &s_ctx.bullet_idx, &s_ctx.angle_idx);
        s_ctx.target_bullet = s_ctx.table.bullet[s_ctx.bullet_idx];
        s_ctx.target_angle = s_ctx.table.angle[s_ctx.angle_idx];
        s_ctx.cmd_angle = s_ctx.target_angle;
        s_ctx.raw_current_cmd = 0.0f;
        s_ctx.last_error = 0;
        PitchCaliStateEnter(PITCH_CALI_STATE_WAIT_BULLET, now);
    }

    if (s_ctx.last_behaviour != GIMBAL_PITCH_CALI)
    {
        if (s_ctx.state != PITCH_CALI_STATE_IDLE && s_ctx.state != PITCH_CALI_STATE_DONE && s_ctx.state != PITCH_CALI_STATE_ERROR)
        {
            s_ctx.cmd_angle = s_ctx.target_angle;
            PitchCaliStateEnter(PITCH_CALI_STATE_MOVE_TO_ANGLE, now);
        }
    }

    s_ctx.running = 1u;
    PitchCaliBuzzerUpdate(now, PitchCaliBeepShouldRun(s_ctx.state));

    if (s_ctx.state == PITCH_CALI_STATE_BREAKAWAY_UP || s_ctx.state == PITCH_CALI_STATE_BREAKAWAY_DOWN)
    {
        gimbal->GimbalPitchMotor.mode = GIMBAL_MOTOR_RAW;
    }
    else
    {
        gimbal->GimbalPitchMotor.mode = GIMBAL_MOTOR_ENCODER;
    }

    s_ctx.last_active = PitchCaliMode;
    s_ctx.last_behaviour = behaviour;
}

void PitchCaliControl(fp32 *yaw_cmd, fp32 *pitch_cmd, GimbalControl *gimbal)
{
    if (yaw_cmd == NULL || pitch_cmd == NULL || gimbal == NULL)
    {
        return;
    }

    *yaw_cmd = 0.0f;

    if (gimbal->GimbalPitchMotor.mode == GIMBAL_MOTOR_RAW)
    {
        *pitch_cmd = s_ctx.raw_current_cmd;
        return;
    }

    fp32 cmd = gimbal->GimbalPitchMotor.angle;
    if (s_ctx.running &&
        s_ctx.state != PITCH_CALI_STATE_IDLE &&
        s_ctx.state != PITCH_CALI_STATE_DONE &&
        s_ctx.state != PITCH_CALI_STATE_ERROR)
    {
        cmd = s_ctx.cmd_angle;
    }
    *pitch_cmd = cmd - gimbal->GimbalPitchMotor.angle_set;
}

void PitchCaliTickPost(const GimbalControl *gimbal, GimbalBehaviour behaviour, uint8_t PitchCaliMode)
{
    if (PitchCaliMode == 0u || gimbal == NULL)
    {
        return;
    }
    if (behaviour != GIMBAL_PITCH_CALI)
    {
        return;
    }
    if (s_ctx.running == 0u)
    {
        return;
    }

    const PitchCaliConfig *cfg = &g_config.gimbal.pitch_cali;
    const uint32_t now = BspTimeGetTickMs();

    // 函数地图：记录当前状态；按状态机等待弹量、移动到角度、测上下静摩擦点、推进网格或保存。
    const fp32 angle = gimbal->GimbalPitchMotor.angle;
    const fp32 gyro = gimbal->GimbalPitchMotor.motor_gyro;
    const fp32 current = gimbal->GimbalPitchMotor.current_set;

    PitchCaliLogRuntime(gimbal, cfg, now, angle, gyro, current);

    if (s_ctx.state == PITCH_CALI_STATE_ERROR || s_ctx.state == PITCH_CALI_STATE_DONE)
    {
        return;
    }

    switch (s_ctx.state)
    {
    case PITCH_CALI_STATE_WAIT_BULLET:
    {
        s_ctx.target_bullet = s_ctx.table.bullet[s_ctx.bullet_idx];
        const uint16_t bullet_now = PitchCaliBulletCountNow();
        if (!PitchCaliBulletReady(bullet_now, s_ctx.target_bullet))
        {
            const fp32 hold = (g_config.gimbal.init_pitch_set);
            fp32 amin = g_config.gimbal.pitch_soft_limit_down;
            fp32 amax = g_config.gimbal.pitch_soft_limit_up;
            if (amax < amin)
            {
                const fp32 t = amax;
                amax = amin;
                amin = t;
            }
            s_ctx.target_angle = fp32_constrain(hold, amin, amax);
            s_ctx.cmd_angle = s_ctx.target_angle;
            break;
        }

        s_ctx.target_angle = s_ctx.table.angle[s_ctx.angle_idx];
        s_ctx.cmd_angle = s_ctx.target_angle;
        PitchCaliStateEnter(PITCH_CALI_STATE_MOVE_TO_ANGLE, now);
        break;
    }
    case PITCH_CALI_STATE_MOVE_TO_ANGLE:
    {
        PitchCaliSeekCmdAngle(angle, cfg);
        (void)PitchCaliTrySnapEndpointTarget(angle, gyro, cfg, now);
        if (!PitchCaliIsStable(angle, gyro, s_ctx.target_angle, cfg))
        {
            s_ctx.stable_enter_ms = 0u;
            break;
        }

        if (s_ctx.stable_enter_ms == 0u)
        {
            s_ctx.stable_enter_ms = now;
            break;
        }
        if ((now - s_ctx.stable_enter_ms) < cfg->stable_time_ms)
        {
            break;
        }

        s_ctx.hold_sum = 0.0f;
        s_ctx.hold_cnt = 0u;
        PitchCaliStateEnter(PITCH_CALI_STATE_HOLD_AVG, now);
        break;
    }
    case PITCH_CALI_STATE_HOLD_AVG:
    {
        s_ctx.hold_sum += current;
        s_ctx.hold_cnt++;

        if ((now - s_ctx.state_enter_ms) < cfg->hold_avg_time_ms)
        {
            break;
        }

        if (s_ctx.hold_cnt == 0u)
        {
            PitchCaliStateEnter(PITCH_CALI_STATE_ERROR, now);
            s_ctx.last_error = -20;
            break;
        }

        s_ctx.hold_avg = s_ctx.hold_sum / (fp32)s_ctx.hold_cnt;

        const int32_t v = PitchCaliRoundI32(s_ctx.hold_avg);
        int16_t v16 = 0;
        if (v > 32767)
        {
            v16 = 32767;
        }
        else if (v < -32768)
        {
            v16 = -32768;
        }
        else
        {
            v16 = (int16_t)v;
        }

        s_ctx.table.hold_current[s_ctx.bullet_idx][s_ctx.angle_idx] = v16;

        s_ctx.delta = 0.0f;
        s_ctx.last_step_ms = now;
        s_ctx.breakaway_base_angle = angle;
        s_ctx.raw_current_cmd = s_ctx.hold_avg;
        PitchCaliStateEnter(PITCH_CALI_STATE_BREAKAWAY_UP, now);
        break;
    }
    case PITCH_CALI_STATE_BREAKAWAY_UP:
    {
        if ((now - s_ctx.last_step_ms) >= cfg->breakaway_step_period_ms)
        {
            s_ctx.last_step_ms = now;
            s_ctx.delta += (fp32)cfg->breakaway_step_current;
            if (s_ctx.delta > (fp32)cfg->breakaway_max_extra_current)
            {
                s_ctx.delta = (fp32)cfg->breakaway_max_extra_current;
            }
        }

        s_ctx.raw_current_cmd = PitchCaliLimitPitchCurrent(s_ctx.hold_avg + s_ctx.delta);

        const fp32 gyro_thr = fabsf(cfg->breakaway_gyro_threshold);
        const fp32 ang_thr = fabsf(cfg->breakaway_angle_threshold);
        const bool_t moved = ((gyro >= gyro_thr) ||
                              ((angle - s_ctx.breakaway_base_angle) >= ang_thr))
                                 ? 1u
                                 : 0u;
        const bool_t timeout = (s_ctx.delta >= (fp32)cfg->breakaway_max_extra_current) ? 1u : 0u;

        if (moved || timeout)
        {
            s_ctx.table.kick_up[s_ctx.bullet_idx][s_ctx.angle_idx] = PitchCaliRoundU16Sat(fabsf(s_ctx.delta));
            s_ctx.cmd_angle = s_ctx.target_angle;
            PitchCaliStateEnter(PITCH_CALI_STATE_RECOVER_UP, now);
        }
        break;
    }
    case PITCH_CALI_STATE_RECOVER_UP:
    {
        PitchCaliSeekCmdAngle(angle, cfg);
        (void)PitchCaliTrySnapEndpointTarget(angle, gyro, cfg, now);
        if ((now - s_ctx.state_enter_ms) < cfg->recover_time_ms)
        {
            break;
        }
        if (!PitchCaliIsStable(angle, gyro, s_ctx.target_angle, cfg))
        {
            s_ctx.stable_enter_ms = 0u;
            break;
        }
        if (s_ctx.stable_enter_ms == 0u)
        {
            s_ctx.stable_enter_ms = now;
            break;
        }
        if ((now - s_ctx.stable_enter_ms) < cfg->stable_time_ms)
        {
            break;
        }

        s_ctx.delta = 0.0f;
        s_ctx.last_step_ms = now;
        s_ctx.breakaway_base_angle = angle;
        s_ctx.raw_current_cmd = s_ctx.hold_avg;
        PitchCaliStateEnter(PITCH_CALI_STATE_BREAKAWAY_DOWN, now);
        break;
    }
    case PITCH_CALI_STATE_BREAKAWAY_DOWN:
    {
        if ((now - s_ctx.last_step_ms) >= cfg->breakaway_step_period_ms)
        {
            s_ctx.last_step_ms = now;
            s_ctx.delta += (fp32)cfg->breakaway_step_current;
            if (s_ctx.delta > (fp32)cfg->breakaway_max_extra_current)
            {
                s_ctx.delta = (fp32)cfg->breakaway_max_extra_current;
            }
        }

        s_ctx.raw_current_cmd = PitchCaliLimitPitchCurrent(s_ctx.hold_avg - s_ctx.delta);

        const fp32 gyro_thr = fabsf(cfg->breakaway_gyro_threshold);
        const fp32 ang_thr = fabsf(cfg->breakaway_angle_threshold);
        const bool_t moved = ((gyro <= -gyro_thr) ||
                              ((s_ctx.breakaway_base_angle - angle) >= ang_thr))
                                 ? 1u
                                 : 0u;
        const bool_t timeout = (s_ctx.delta >= (fp32)cfg->breakaway_max_extra_current) ? 1u : 0u;

        if (moved || timeout)
        {
            s_ctx.table.kick_down[s_ctx.bullet_idx][s_ctx.angle_idx] = PitchCaliRoundU16Sat(fabsf(s_ctx.delta));
            s_ctx.cmd_angle = s_ctx.target_angle;
            PitchCaliStateEnter(PITCH_CALI_STATE_RECOVER_DOWN, now);
        }
        break;
    }
    case PITCH_CALI_STATE_RECOVER_DOWN:
    {
        PitchCaliSeekCmdAngle(angle, cfg);
        (void)PitchCaliTrySnapEndpointTarget(angle, gyro, cfg, now);
        if ((now - s_ctx.state_enter_ms) < cfg->recover_time_ms)
        {
            break;
        }
        if (!PitchCaliIsStable(angle, gyro, s_ctx.target_angle, cfg))
        {
            s_ctx.stable_enter_ms = 0u;
            break;
        }
        if (s_ctx.stable_enter_ms == 0u)
        {
            s_ctx.stable_enter_ms = now;
            break;
        }
        if ((now - s_ctx.stable_enter_ms) < cfg->stable_time_ms)
        {
            break;
        }

        const uint16_t completed_cells = (uint16_t)(PitchCaliGridIndex(&s_ctx.table, s_ctx.bullet_idx, s_ctx.angle_idx) + 1u);
        const uint16_t gridN = PitchCaliGridCount(&s_ctx.table);

        s_ctx.table.completed_cells = completed_cells;
        s_ctx.table.valid = (completed_cells >= gridN) ? 1u : 0u;

        const int save_r = PitchCaliSaveToSd(&s_ctx.table, completed_cells);
        if (save_r != 0)
        {
            s_ctx.last_error = save_r;
            PitchCaliStateEnter(PITCH_CALI_STATE_ERROR, now);
            break;
        }

        if (completed_cells >= gridN)
        {
            PitchCaliStateEnter(PITCH_CALI_STATE_DONE, now);
            break;
        }

        PitchCaliResumeIndicesFromCompleted(&s_ctx.table, &s_ctx.bullet_idx, &s_ctx.angle_idx);
        s_ctx.target_angle = s_ctx.table.angle[s_ctx.angle_idx];
        s_ctx.cmd_angle = s_ctx.target_angle;
        PitchCaliStateEnter(PITCH_CALI_STATE_WAIT_BULLET, now);
        break;
    }
    case PITCH_CALI_STATE_SAVE:
    {
        s_ctx.table.valid = 1u;
        s_ctx.table.completed_cells = PitchCaliGridCount(&s_ctx.table);
        const int r = PitchCaliSaveToSd(&s_ctx.table, s_ctx.table.completed_cells);
        if (r == 0)
        {
            PitchCaliStateEnter(PITCH_CALI_STATE_DONE, now);
        }
        else
        {
            s_ctx.last_error = r;
            PitchCaliStateEnter(PITCH_CALI_STATE_ERROR, now);
        }
        break;
    }
    default:
        break;
    }
}

#endif
