/*
 * SPDX-FileCopyrightText: 2026 闄堣僵 <2811158416@qq.com>
 * SPDX-License-Identifier: PolyForm-Noncommercial-1.0.0
 * Required Notice: Copyright 2026 闄堣僵 <2811158416@qq.com>
 *
 * First published in this repository: 2026-04-06
 * Use of this file is governed by the LICENSE file in the repository root.
 */

#include "pitch_cali.h"

#include <string.h>

// F407 built-in default table:
// offline-expanded from 10 measured points using a shape-preserving cubic Hermite model.
static const uint16_t k_pitch_cali_builtin_bullet[1] = {0u};
static const fp32 k_pitch_cali_builtin_angle[33] = {
    -0.400000000f,
    -0.369062000f,
    -0.338125000f,
    -0.307187000f,
    -0.276250000f,
    -0.245312000f,
    -0.214375000f,
    -0.183438000f,
    -0.152500000f,
    -0.121563000f,
    -0.090625000f,
    -0.059688000f,
    -0.028750000f,
    0.002187000f,
    0.033125000f,
    0.064062000f,
    0.095000000f,
    0.125937000f,
    0.156875000f,
    0.187812000f,
    0.218750000f,
    0.249687000f,
    0.280625000f,
    0.311562000f,
    0.342500000f,
    0.373437000f,
    0.404375000f,
    0.435312000f,
    0.466250000f,
    0.497187000f,
    0.528125000f,
    0.559062000f,
    0.590000000f
};
static const int16_t k_pitch_cali_builtin_hold[33] = {
    -234,
    -89,
    131,
    405,
    750,
    1344,
    1953,
    2271,
    1902,
    968,
    106,
    -115,
    -113,
    -110,
    -104,
    37,
    458,
    877,
    1012,
    992,
    949,
    888,
    743,
    390,
    32,
    -105,
    156,
    620,
    982,
    1001,
    801,
    414,
    -157
};
static const uint16_t k_pitch_cali_builtin_kick_up[33] = {
    2800u,
    2787u,
    2750u,
    2691u,
    2542u,
    1924u,
    1172u,
    755u,
    1180u,
    2254u,
    3246u,
    3498u,
    3469u,
    3409u,
    3322u,
    2969u,
    2104u,
    1268u,
    1009u,
    1222u,
    1603u,
    1994u,
    2314u,
    2665u,
    2949u,
    3046u,
    2748u,
    2219u,
    1806u,
    1803u,
    2146u,
    2775u,
    3650u
};
static const uint16_t k_pitch_cali_builtin_kick_down[33] = {
    700u,
    1015u,
    1204u,
    1289u,
    1259u,
    957u,
    570u,
    353u,
    362u,
    396u,
    433u,
    456u,
    467u,
    477u,
    494u,
    647u,
    1064u,
    1488u,
    1661u,
    1705u,
    1735u,
    1749u,
    1639u,
    1206u,
    738u,
    554u,
    863u,
    1413u,
    1842u,
    1871u,
    1666u,
    1262u,
    650u
};

bool_t pitch_cali_get_builtin_default(pitch_cali_builtin_desc_t *out)
{
    if (out == NULL)
    {
        return 0;
    }

    memset(out, 0, sizeof(*out));
    out->angle_points = (uint8_t)(sizeof(k_pitch_cali_builtin_angle) / sizeof(k_pitch_cali_builtin_angle[0]));
    out->bullet_points = (uint8_t)(sizeof(k_pitch_cali_builtin_bullet) / sizeof(k_pitch_cali_builtin_bullet[0]));
    out->angle = k_pitch_cali_builtin_angle;
    out->bullet = k_pitch_cali_builtin_bullet;
    out->hold_current = k_pitch_cali_builtin_hold;
    out->kick_up = k_pitch_cali_builtin_kick_up;
    out->kick_down = k_pitch_cali_builtin_kick_down;
    return 1;
}
