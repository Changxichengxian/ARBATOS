/*
 * SPDX-FileCopyrightText: 2026 陈轩 <2811158416@qq.com>
 * SPDX-License-Identifier: Apache-2.0
 *
 * First published in this repository: 2026-07-10
 * Use of this file is governed by the LICENSE file in the repository root.
 */

#ifndef MANUAL_INPUT_SNAPSHOT_H
#define MANUAL_INPUT_SNAPSHOT_H

#include <stdint.h>

#include "ManualInput.h"
#include "ControlInput.h"

#define MANUAL_INPUT_DEFAULT_TIMEOUT_MS 100u

/*
 * 一次发布同时固定原始输入、业务映射、来源、协议业务标志和在线状态。
 * activeMask 的 bit0..bit3 依次表示 DBUS、ELRS、Image、USB。
 * sourceSeq 在无来源时为 0，switchSeq 在尚未发生真实切源时为 0；
 * semanticsSeq 在来源选择、协议映射、轴/开关映射或业务语义变化时递增；
 * 所有已启用序号回绕时都跳过 0。
 */
typedef struct ManualInputSnapshot
{
    ManualInputState manual;
    ControlInputState control;
    /* 与输入值同代冻结，消费者不得用实时配置重新解释本帧开关。 */
    ManualInputSemanticsConfig semantics;
    uint32_t activeMask;
    uint32_t sourceTickMs;
    uint32_t publishTickMs;
    uint32_t readTickMs;
    uint32_t sourceAgeMs;
    uint32_t sourceTimeoutMs;
    uint32_t publishSeq;
    uint32_t sourceSeq;
    uint32_t switchSeq;
    uint32_t semanticsSeq;
    uint8_t activeSource;
    uint8_t online;
    uint8_t mixMode;
    uint8_t sourceProtocol;
    uint8_t sourceFlags;
    uint8_t reserved[3];
} ManualInputSnapshot;

/* 仅供任务上下文读取；不会触发来源合并、控制映射、日志或监控更新。 */
uint8_t ManualInputSnapshotRead(ManualInputSnapshot *out);

#endif
