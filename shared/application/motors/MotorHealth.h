/*
 * SPDX-FileCopyrightText: 2026 Xie Yuhan <2811158416@qq.com>
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef MOTOR_HEALTH_H
#define MOTOR_HEALTH_H

#include <stdint.h>

#include "LowCmd.h"

#ifdef __cplusplus
extern "C" {
#endif

#define MOTOR_HEALTH_MAX_MOTORS 18u
#define MOTOR_HEALTH_AGE_UNKNOWN 0xFFFFFFFFu

typedef enum
{
    MOTOR_HEALTH_REASON_NONE = 0u,
    MOTOR_HEALTH_REASON_NO_FEEDBACK = (1u << 0),
    MOTOR_HEALTH_REASON_TIMEOUT = (1u << 1),
    MOTOR_HEALTH_REASON_DRIVE_OFFLINE = (1u << 2),
    MOTOR_HEALTH_REASON_DRIVE_FAULT = (1u << 3),
    MOTOR_HEALTH_REASON_READ_FAILED = (1u << 4),
    MOTOR_HEALTH_REASON_INVALID_ID = (1u << 5),
} MotorHealthReason;

typedef struct
{
    MotorId motorId;
    MotorState feedback;
    uint32_t ageMs;
    uint16_t reasonMask;
    uint8_t feedbackValid;
    uint8_t fresh;
    uint8_t healthy;
    uint8_t reserved0;
} MotorHealthResult;

/*
 * 固定 18 轴的结果约占 1 KiB，只供低频故障汇总、诊断和测试使用。
 * 1~3 ms 高频任务应逐轴使用 MotorHealthResult；确需批量时由模块持有静态实例，
 * 不要把本结构放进高频任务栈，也不要在高频循环反复清整块内存。
 */
typedef struct
{
    /* bit i 对应调用方传入的 ids[i]，不是 MotorId 的数值。 */
    uint32_t faultMask;
    uint8_t count;
    uint8_t reserved0[3];
    MotorHealthResult item[MOTOR_HEALTH_MAX_MOTORS];
} MotorHealthBatch;

/*
 * 纯判定入口，不读取系统状态，主机测试和仿真可直接注入反馈。
 * online 字段只随快照输出，不参与健康判定；有效反馈、时间和驱动状态才是依据。
 */
uint8_t MotorHealthEval(MotorId id,
                        const MotorState *feedback,
                        uint32_t nowMs,
                        uint32_t timeoutMs,
                        MotorHealthResult *out);

/* 低频批量入口；实时控制循环的使用约束见 MotorHealthBatch。 */
uint8_t MotorHealthEvalMany(const MotorId *ids,
                            const MotorState *feedback,
                            uint8_t count,
                            uint32_t nowMs,
                            uint32_t timeoutMs,
                            MotorHealthBatch *out);

/*
 * 读取入口通过 LowStateGetMotor() 获取完整反馈副本。
 * 返回值表示快照读取是否成功；设备能否使用应读取 out->healthy。
 */
uint8_t MotorHealthRead(MotorId id,
                        uint32_t nowMs,
                        uint32_t timeoutMs,
                        MotorHealthResult *out);

/* 低频批量入口；实时控制循环的使用约束见 MotorHealthBatch。 */
uint8_t MotorHealthReadMany(const MotorId *ids,
                            uint8_t count,
                            uint32_t nowMs,
                            uint32_t timeoutMs,
                            MotorHealthBatch *out);

#ifdef __cplusplus
}
#endif

#endif
