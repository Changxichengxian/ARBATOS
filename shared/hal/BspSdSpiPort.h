/*
 * SPDX-FileCopyrightText: 2026 陈轩 <2811158416@qq.com>
 * SPDX-License-Identifier: Apache-2.0
 *
 * First published in this repository: 2026-04-06
 * Use of this file is governed by the LICENSE file in the repository root.
 */

#ifndef BSP_SD_SPI_PORT_H
#define BSP_SD_SPI_PORT_H

#include <stdint.h>

typedef enum
{
    SD_SPI_PORT_SPEED_INIT = 0,
    SD_SPI_PORT_SPEED_FAST,
} SdSpiPortSpeed;

void SdSpiPortCsHigh(void);
void SdSpiPortCsLow(void);
uint8_t SdSpiPortTxrx(uint8_t data);
uint32_t SdSpiPortTickMs(void);

int SdSpiPortTxrxDma(const uint8_t *tx, uint8_t *rx, uint16_t len, uint32_t timeout_ms);
int SdSpiPortReceive(uint8_t *buf, uint16_t len, uint32_t timeout_ms);
int SdSpiPortTransmit(const uint8_t *buf, uint16_t len, uint32_t timeout_ms);

void SdSpiPortSetSpeed(SdSpiPortSpeed speed);

#if defined(__ZEPHYR__)
typedef struct
{
    uint32_t currentHz;
    uint32_t transferCalls;
    uint32_t transferErrors;
    int32_t lastError;
} SdSpiPortDiagState;

extern volatile SdSpiPortDiagState SdSpiPortDiag;

/* Zephyr SPI 卡由端口按板级能力决定是否启用这些正式优化。 */
uint8_t SdSpiPortUseMultiRead(void);
uint8_t SdSpiPortVerifyReadCrc(void);
uint8_t SdSpiPortHasHighSpeed(void);
int SdSpiPortEnableHighSpeed(void);
#endif

#if defined(SD_BENCH_TEST)
/* SD_BENCH_TEST 专用：三个模式分别用于复现旧路径、整扇区 HAL 和 FIFO 打包写入。 */
#define SD_SPI_PORT_BENCH_MODE_OLD_64B       0u
#define SD_SPI_PORT_BENCH_MODE_HAL_512B      1u
#define SD_SPI_PORT_BENCH_MODE_FIFO_512B     2u

typedef struct
{
    uint32_t transferCalls;
    uint32_t transferErrors;
    uint32_t transferTimeouts;
    uint32_t lastHalStatus;
    uint32_t lastHalError;
} SdSpiPortBenchStats;

/* 高频仅在卡已由上层用 CMD6 切到 High Speed 后申请。 */
int SdSpiPortBenchConfigure(uint32_t targetHz, uint32_t mode, uint32_t *actualHz);
void SdSpiPortBenchGetStats(SdSpiPortBenchStats *stats);
#endif

#if defined(SUB_BOARD_FACTORY_TEST) || defined(SD_BENCH_TEST)
/* 仅工厂测试使用：按实际 SPI3 时钟选择最接近目标的分频，并返回实际频率。 */
int SdSpiPortFactorySetHz(uint32_t target_hz, uint32_t *actual_hz);
#endif

#endif
