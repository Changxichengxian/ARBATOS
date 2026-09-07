#ifndef SD_SPI_H
#define SD_SPI_H

#include <stdint.h>

typedef enum
{
    SD_SPI_CARD_NONE = 0,
    SD_SPI_CARD_MMC,
    SD_SPI_CARD_SDSC,
    SD_SPI_CARD_SDHC,
} SdSpiCardType;

#if defined(SD_BENCH_TEST) || defined(__ZEPHYR__)
typedef enum
{
    SD_SPI_BENCH_HS_OK = 0,
    SD_SPI_BENCH_HS_NOT_READY = -1,
    SD_SPI_BENCH_HS_NOT_SD = -2,
    SD_SPI_BENCH_HS_CHECK_CMD = -3,
    SD_SPI_BENCH_HS_CHECK_DATA = -4,
    SD_SPI_BENCH_HS_UNSUPPORTED = -5,
    SD_SPI_BENCH_HS_CHECK_SELECTION = -6,
    SD_SPI_BENCH_HS_CHECK_BUSY = -7,
    SD_SPI_BENCH_HS_STATUS_VERSION = -8,
    SD_SPI_BENCH_HS_SWITCH_CMD = -9,
    SD_SPI_BENCH_HS_SWITCH_DATA = -10,
    SD_SPI_BENCH_HS_SWITCH_SELECTION = -11,
    SD_SPI_BENCH_HS_SWITCH_BUSY = -12,
    SD_SPI_BENCH_HS_PORT_SPEED = -13,
} SdSpiBenchHighSpeedResult;

#if defined(__ZEPHYR__)
/* 调试器可读：本次初始化的 CMD6 协商结果。 */
extern volatile int SdSpiHighSpeedResult;
#endif
#endif

/**
  * @brief  Initialize SD card on the board SPI SD port.
  * @retval 0 on success, non-zero on failure.
  */
int SdSpiInit(void);

/**
  * @brief  Check if SD card is initialized and ready.
  * @retval 1 ready, 0 not ready.
  */
int SdSpiIsReady(void);

/**
  * @brief  Get detected SD card type.
  */
SdSpiCardType SdSpiGetCardType(void);

/**
  * @brief  Read 512-byte sectors.
  * @retval 0 on success, non-zero on failure.
  */
int SdSpiRead(uint8_t *buf, uint32_t sector, uint32_t count);

/**
  * @brief  Write 512-byte sectors.
  * @retval 0 on success, non-zero on failure.
  */
int SdSpiWrite(const uint8_t *buf, uint32_t sector, uint32_t count);

/**
  * @brief  Get sector count (capacity / 512).
  * @retval 0 on success, non-zero on failure.
  */
int SdSpiGetSectorCount(uint32_t *out_sectors);

/**
  * @brief  Ensure the card is not busy (write finished).
  * @retval 0 on success, non-zero on failure.
  */
int SdSpiSync(void);

#if defined(SD_BENCH_TEST)
/* 仅测试固件：count 大于 1 时选择 CMD17 或 CMD18。 */
void SdSpiBenchSetMultiRead(uint8_t enabled);

/* 仅测试固件：开关本机对接收数据块 CRC16 的校验，不改变卡端 CRC 设置。 */
int SdSpiBenchSetCrc16(uint8_t enabled);

/*
 * 仅测试固件：在当前时钟不高于 25 MHz 时查询、切换并确认卡端 High Speed。
 * 成功后仍需由 BSP 设置主机 SPI 时钟。
 * @retval SdSpiBenchHighSpeedResult。
 */
int SdSpiBenchHighSpeed(void);
#endif

#endif
