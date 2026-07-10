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
#define MANUAL_INPUT_SNAPSHOT_STACK_BUDGET_BYTES 128u

/*
 * 一次发布同时固定原始输入、业务映射、来源、协议业务标志和在线状态。
 * activeMask 的 bit0..bit3 依次表示 DBUS、ELRS、Image、USB。
 * sourceSeq 在无来源时为 0；MERGE 中 activeSource/sourceProtocol/sourceSeq/时间
 * 描述拥有完整操纵帧的稳定健康代表来源；activeMask 是全部健康候选来源。
 * MERGE 也不会拼接连续轴、通用键鼠和拨杆，只会把类型明确的 sourceFlags 做并集。
 * switchSeq 只随正式 bank 发布递增；读端在来源失效边界基于同代证据回退时保持原值，
 * 随后的正式发布会记录该次切源。switchSeq 在尚未发生正式切源时为 0；
 * semanticsSeq 在协议映射参数、轴/开关映射或业务语义配置变化时递增；
 * actionSeq 在实际动作来源集合、会话或解释变化时递增；备用候选抖动不会推进，
 * 离散动作消费者据此要求按键释放。
 * authoritySeq 只在代表控制来源、协议或会话变化时递增，生命周期据此重新看安全档；
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
    uint32_t actionSeq;
    uint32_t authoritySeq;
    uint8_t activeSource;
    uint8_t online;
    /* 发布时已按统一输入规则校验；online!=0 必然蕴含 dataValid!=0。 */
    uint8_t dataValid;
    uint8_t mixMode;
    uint8_t sourceProtocol;
    uint8_t sourceFlags;
    uint8_t reserved[2];
} ManualInputSnapshot;

/* 当前为 112 字节；多个高频任务按值持有，增长必须显式重审任务栈。 */
typedef char ManualInputSnapshotStackBudgetCheck[
    (sizeof(ManualInputSnapshot) <= MANUAL_INPUT_SNAPSHOT_STACK_BUDGET_BYTES) ? 1 : -1];

/* 仅供任务上下文读取；正常路径固定复制，失效边界会用同代证据纯计算回退。 */
uint8_t ManualInputSnapshotRead(ManualInputSnapshot *out);

#endif
