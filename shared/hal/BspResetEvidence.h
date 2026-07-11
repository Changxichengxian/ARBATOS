/*
 * SPDX-FileCopyrightText: 2026 Xie Yuhan <2811158416@qq.com>
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef BSP_RESET_EVIDENCE_H
#define BSP_RESET_EVIDENCE_H

#include <stdint.h>

#define BSP_RESET_EVIDENCE_FORMAT_VERSION 1u

/*
 * 这份摘要只由系统致命入口写入。字段全部固定为 32 位，便于异常上下文直接写，
 * 也让 F4、H7、SD 日志和主机解析工具共用同一布局。
 */
typedef struct
{
    uint32_t formatVersion;
    uint32_t recordSize;
    uint32_t sequence;
    uint32_t reason;
    uint32_t arg0;
    uint32_t arg1;
    uint32_t ipsr;
    uint32_t excReturn;
    uint32_t msp;
    uint32_t psp;
    uint32_t cfsr;
    uint32_t hfsr;
    uint32_t dfsr;
    uint32_t afsr;
    uint32_t mmfar;
    uint32_t bfar;
    uint32_t icsr;
    uint32_t shcsr;
    uint32_t control;
    uint32_t stackPtr;
    uint32_t r0;
    uint32_t r1;
    uint32_t r2;
    uint32_t r3;
    uint32_t r12;
    uint32_t lr;
    uint32_t pc;
    uint32_t xpsr;
    uint32_t tickMs;
    uint32_t bootStage;
    uint32_t taskHandle;
} BspResetEvidenceRecord;

typedef struct
{
    uint32_t resetFlags;
    uint32_t evidenceValid;
    BspResetEvidenceRecord evidence;
} BspResetEvidenceBoot;

/* 必须在 HAL_Init() 前调用；接口幂等，WatchInit 也会做一次兜底。 */
void BspResetEvidenceCaptureBoot(void);
uint8_t BspResetEvidenceGetBoot(BspResetEvidenceBoot *out);

/* 不依赖 HAL、FreeRTOS、动态内存、Flash 或 SD，可从异常上下文调用。 */
void BspResetEvidenceWriteFatal(const BspResetEvidenceRecord *record);

/* 只有证据已经可靠写入外部存储后才确认，序号不匹配时不会清除新记录。 */
void BspResetEvidenceAcknowledge(uint32_t sequence);

#endif
