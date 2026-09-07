/*
 * SPDX-FileCopyrightText: 2026 陈轩 <2811158416@qq.com>
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ARBATOS_ZEPHYR_DT_H
#define ARBATOS_ZEPHYR_DT_H

#include <zephyr/device.h>
#include <zephyr/devicetree.h>

/*
 * Zephyr 自带的 PWM_DT_SPEC_GET_BY_IDX() 固定读取名为 pwms 的属性。
 * ARBATOS 聚合节点需要区分蜂鸣器、IMU 加热等用途，因此提供等价的具名属性版本。
 */
#define ARBATOS_PWM_DT_SPEC_GET_BY_IDX(node_id, prop, idx)                     \
    {                                                                          \
        .dev = DEVICE_DT_GET(DT_PHANDLE_BY_IDX(node_id, prop, idx)),           \
        .channel = DT_PHA_BY_IDX(node_id, prop, idx, channel),                 \
        .period = DT_PHA_BY_IDX(node_id, prop, idx, period),                   \
        .flags = DT_PHA_BY_IDX_OR(node_id, prop, idx, flags, 0),               \
    }

#endif
