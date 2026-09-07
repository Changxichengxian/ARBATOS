
#include "BspSdSpiPort.h"

#include <string.h>

#include "main.h"

#define SD_SPI_PORT_HANDLE               SPI3
#define SD_SPI_PORT_GPIO                 GPIOC
#define SD_SPI_PORT_GPIO_CLK_ENABLE()    __HAL_RCC_GPIOC_CLK_ENABLE()
#define SD_SPI_PORT_CS_GPIO              GPIOE
#define SD_SPI_PORT_CS_GPIO_CLK_ENABLE() __HAL_RCC_GPIOE_CLK_ENABLE()
#define SD_SPI_PORT_SCK_PIN              GPIO_PIN_10
#define SD_SPI_PORT_MISO_PIN             GPIO_PIN_11
#define SD_SPI_PORT_MOSI_PIN             GPIO_PIN_12
#define SD_SPI_PORT_CS_PIN               GPIO_PIN_14
#define SD_SPI_PORT_AF                   GPIO_AF6_SPI3
#define SD_SPI_PORT_TIMEOUT_MS           1000u
#define SD_SPI_PORT_BYTE_TIMEOUT_MS      10u
#define SD_SPI_PORT_INIT_MAX_HZ          400000u
#define SD_SPI_PORT_FAST_MAX_HZ          8000000u
#define SD_SPI_PORT_CHUNK_SIZE           64u
#define SD_SPI_PORT_SECTOR_SIZE           512u

static SPI_HandleTypeDef hspi3_sd;
static uint8_t SdSpiPortInited = 0u;
static uint32_t SdSpiPortPrescaler = 0u;
static uint8_t SdSpiPortDummyTx[SD_SPI_PORT_CHUNK_SIZE];

#if defined(SD_BENCH_TEST)
static uint32_t SdSpiPortBenchMode = SD_SPI_PORT_BENCH_MODE_OLD_64B;
static SdSpiPortBenchStats SdSpiPortBenchTransferStats;
static uint8_t SdSpiPortBenchDummyTx[SD_SPI_PORT_SECTOR_SIZE];
#endif

static uint32_t SdSpiPortFifoThreshold(void)
{
#if defined(SD_BENCH_TEST)
    if (SdSpiPortBenchMode == SD_SPI_PORT_BENCH_MODE_FIFO_512B)
    {
        /* H7 HAL 的阻塞发送会在阈值大于 3 字节时按 32 位写 TXDR。 */
        return SPI_FIFO_THRESHOLD_04DATA;
    }
#endif
    return SPI_FIFO_THRESHOLD_01DATA;
}

#if defined(SD_BENCH_TEST)
static void SdSpiPortRecordTransfer(HAL_StatusTypeDef status)
{
    SdSpiPortBenchTransferStats.transferCalls++;
    SdSpiPortBenchTransferStats.lastHalStatus = (uint32_t)status;
    if (status != HAL_OK)
    {
        SdSpiPortBenchTransferStats.transferErrors++;
        if (status == HAL_TIMEOUT)
        {
            SdSpiPortBenchTransferStats.transferTimeouts++;
        }
        SdSpiPortBenchTransferStats.lastHalError = HAL_SPI_GetError(&hspi3_sd);
    }
}
#else
#define SdSpiPortRecordTransfer(status) ((void)(status))
#endif

static uint16_t SdSpiPortReceiveChunkSize(uint16_t remaining)
{
    uint16_t chunk_size = SD_SPI_PORT_CHUNK_SIZE;

#if defined(SD_BENCH_TEST)
    if (SdSpiPortBenchMode != SD_SPI_PORT_BENCH_MODE_OLD_64B)
    {
        chunk_size = SD_SPI_PORT_SECTOR_SIZE;
    }
#endif

    return (remaining > chunk_size) ? chunk_size : remaining;
}

static uint16_t SdSpiPortTransmitChunkSize(uint16_t remaining)
{
    /* 原路径写入本就是整块 HAL，基准测试不能人为拆成 64 字节。 */
    return remaining;
}

static uint8_t *SdSpiPortDummyTxForChunk(uint16_t chunk)
{
#if defined(SD_BENCH_TEST)
    if (chunk > SD_SPI_PORT_CHUNK_SIZE)
    {
        return SdSpiPortBenchDummyTx;
    }
#endif

    return SdSpiPortDummyTx;
}

static uint32_t SdSpiPortPickPrescaler(uint32_t target_hz)
{
    static const struct
    {
        uint16_t div;
        uint32_t reg;
    } prescaler_table[] = {
        {2u, SPI_BAUDRATEPRESCALER_2},
        {4u, SPI_BAUDRATEPRESCALER_4},
        {8u, SPI_BAUDRATEPRESCALER_8},
        {16u, SPI_BAUDRATEPRESCALER_16},
        {32u, SPI_BAUDRATEPRESCALER_32},
        {64u, SPI_BAUDRATEPRESCALER_64},
        {128u, SPI_BAUDRATEPRESCALER_128},
        {256u, SPI_BAUDRATEPRESCALER_256},
    };

    const uint32_t spi_clk_hz = HAL_RCCEx_GetPeriphCLKFreq(RCC_PERIPHCLK_SPI123);

    if (spi_clk_hz == 0u || target_hz == 0u)
    {
        return SPI_BAUDRATEPRESCALER_256;
    }

    for (uint32_t i = 0u; i < (uint32_t)(sizeof(prescaler_table) / sizeof(prescaler_table[0])); i++)
    {
        if (spi_clk_hz <= (uint32_t)prescaler_table[i].div * target_hz)
        {
            return prescaler_table[i].reg;
        }
    }

    return SPI_BAUDRATEPRESCALER_256;
}

static void SdSpiPortGpioInit(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    SD_SPI_PORT_GPIO_CLK_ENABLE();
    SD_SPI_PORT_CS_GPIO_CLK_ENABLE();

    GPIO_InitStruct.Pin = SD_SPI_PORT_SCK_PIN | SD_SPI_PORT_MOSI_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStruct.Alternate = SD_SPI_PORT_AF;
    HAL_GPIO_Init(SD_SPI_PORT_GPIO, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = SD_SPI_PORT_MISO_PIN;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(SD_SPI_PORT_GPIO, &GPIO_InitStruct);

    HAL_GPIO_WritePin(SD_SPI_PORT_CS_GPIO, SD_SPI_PORT_CS_PIN, GPIO_PIN_SET);

    GPIO_InitStruct.Pin = SD_SPI_PORT_CS_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStruct.Alternate = 0u;
    HAL_GPIO_Init(SD_SPI_PORT_CS_GPIO, &GPIO_InitStruct);
}

static void SdSpiPortClockInit(void)
{
    RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

    PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_SPI3;
    PeriphClkInitStruct.Spi123ClockSelection = RCC_SPI123CLKSOURCE_PLL;
    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
    {
        Error_Handler();
    }

    __HAL_RCC_SPI3_CLK_ENABLE();
}

static void SdSpiPortApplyPrescaler(uint32_t prescaler)
{
    const uint32_t fifo_threshold = SdSpiPortFifoThreshold();

    if (SdSpiPortInited != 0u && SdSpiPortPrescaler == prescaler &&
        hspi3_sd.Init.FifoThreshold == fifo_threshold)
    {
        SdSpiPortClockInit();
        SdSpiPortGpioInit();
        HAL_GPIO_WritePin(SD_SPI_PORT_CS_GPIO, SD_SPI_PORT_CS_PIN, GPIO_PIN_SET);
        return;
    }

    SdSpiPortClockInit();
    SdSpiPortGpioInit();

    if (SdSpiPortInited == 0u)
    {
        memset(SdSpiPortDummyTx, 0xFF, sizeof(SdSpiPortDummyTx));
#if defined(SD_BENCH_TEST)
        memset(SdSpiPortBenchDummyTx, 0xFF, sizeof(SdSpiPortBenchDummyTx));
#endif
        memset(&hspi3_sd, 0, sizeof(hspi3_sd));
    }
    else
    {
        (void)HAL_SPI_DeInit(&hspi3_sd);
    }

    hspi3_sd.Instance = SD_SPI_PORT_HANDLE;
    hspi3_sd.Init.Mode = SPI_MODE_MASTER;
    hspi3_sd.Init.Direction = SPI_DIRECTION_2LINES;
    hspi3_sd.Init.DataSize = SPI_DATASIZE_8BIT;
    hspi3_sd.Init.CLKPolarity = SPI_POLARITY_LOW;
    hspi3_sd.Init.CLKPhase = SPI_PHASE_1EDGE;
    hspi3_sd.Init.NSS = SPI_NSS_SOFT;
    hspi3_sd.Init.BaudRatePrescaler = prescaler;
    hspi3_sd.Init.FirstBit = SPI_FIRSTBIT_MSB;
    hspi3_sd.Init.TIMode = SPI_TIMODE_DISABLE;
    hspi3_sd.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
    hspi3_sd.Init.CRCPolynomial = 0x0;
    hspi3_sd.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;
    hspi3_sd.Init.NSSPolarity = SPI_NSS_POLARITY_LOW;
    hspi3_sd.Init.FifoThreshold = fifo_threshold;
    hspi3_sd.Init.TxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
    hspi3_sd.Init.RxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
    hspi3_sd.Init.MasterSSIdleness = SPI_MASTER_SS_IDLENESS_00CYCLE;
    hspi3_sd.Init.MasterInterDataIdleness = SPI_MASTER_INTERDATA_IDLENESS_00CYCLE;
    hspi3_sd.Init.MasterReceiverAutoSusp = SPI_MASTER_RX_AUTOSUSP_DISABLE;
    hspi3_sd.Init.MasterKeepIOState = SPI_MASTER_KEEP_IO_STATE_DISABLE;
    hspi3_sd.Init.IOSwap = SPI_IO_SWAP_DISABLE;

    if (HAL_SPI_Init(&hspi3_sd) != HAL_OK)
    {
        Error_Handler();
    }

    SdSpiPortPrescaler = prescaler;
    SdSpiPortInited = 1u;
    HAL_GPIO_WritePin(SD_SPI_PORT_CS_GPIO, SD_SPI_PORT_CS_PIN, GPIO_PIN_SET);
}

static void SdSpiPortEnsureReady(void)
{
    if (SdSpiPortInited == 0u)
    {
        SdSpiPortApplyPrescaler(SdSpiPortPickPrescaler(SD_SPI_PORT_INIT_MAX_HZ));
    }
}

void SdSpiPortCsHigh(void)
{
    SdSpiPortEnsureReady();
    HAL_GPIO_WritePin(SD_SPI_PORT_CS_GPIO, SD_SPI_PORT_CS_PIN, GPIO_PIN_SET);
}

void SdSpiPortCsLow(void)
{
    SdSpiPortEnsureReady();
    HAL_GPIO_WritePin(SD_SPI_PORT_CS_GPIO, SD_SPI_PORT_CS_PIN, GPIO_PIN_RESET);
}

uint8_t SdSpiPortTxrx(uint8_t data)
{
    uint8_t rx = 0xFFu;
    HAL_StatusTypeDef status;

    SdSpiPortEnsureReady();
    status = HAL_SPI_TransmitReceive(&hspi3_sd, &data, &rx, 1u, SD_SPI_PORT_BYTE_TIMEOUT_MS);
    SdSpiPortRecordTransfer(status);
    return rx;
}

uint32_t SdSpiPortTickMs(void)
{
    return HAL_GetTick();
}

int SdSpiPortTxrxDma(const uint8_t *tx, uint8_t *rx, uint16_t len, uint32_t timeout_ms)
{
    (void)tx;
    (void)rx;
    (void)len;
    (void)timeout_ms;
    return -2;
}

int SdSpiPortReceive(uint8_t *buf, uint16_t len, uint32_t timeout_ms)
{
    uint16_t offset = 0u;

    if (buf == NULL || len == 0u)
    {
        return -1;
    }

    SdSpiPortEnsureReady();

    while (offset < len)
    {
        const uint16_t remaining = (uint16_t)(len - offset);
        const uint16_t chunk = SdSpiPortReceiveChunkSize(remaining);
        const HAL_StatusTypeDef status = HAL_SPI_TransmitReceive(&hspi3_sd, SdSpiPortDummyTxForChunk(chunk),
                                                                  &buf[offset], chunk, timeout_ms);
        SdSpiPortRecordTransfer(status);
        if (status != HAL_OK)
        {
            return -2;
        }
        offset = (uint16_t)(offset + chunk);
    }

    return 0;
}

int SdSpiPortTransmit(const uint8_t *buf, uint16_t len, uint32_t timeout_ms)
{
    uint16_t offset = 0u;

    if (buf == NULL || len == 0u)
    {
        return -1;
    }

    SdSpiPortEnsureReady();
    while (offset < len)
    {
        const uint16_t remaining = (uint16_t)(len - offset);
        const uint16_t chunk = SdSpiPortTransmitChunkSize(remaining);
        const HAL_StatusTypeDef status = HAL_SPI_Transmit(&hspi3_sd, &buf[offset], chunk, timeout_ms);
        SdSpiPortRecordTransfer(status);
        if (status != HAL_OK)
        {
            return -2;
        }
        offset = (uint16_t)(offset + chunk);
    }

    return 0;
}

void SdSpiPortSetSpeed(SdSpiPortSpeed speed)
{
    const uint32_t target_hz = (speed == SD_SPI_PORT_SPEED_FAST) ? SD_SPI_PORT_FAST_MAX_HZ : SD_SPI_PORT_INIT_MAX_HZ;
    SdSpiPortApplyPrescaler(SdSpiPortPickPrescaler(target_hz));
}

#if defined(SD_BENCH_TEST)
void SdSpiPortBenchGetStats(SdSpiPortBenchStats *stats)
{
    if (stats != NULL)
    {
        *stats = SdSpiPortBenchTransferStats;
    }
}

int SdSpiPortBenchConfigure(uint32_t targetHz, uint32_t mode, uint32_t *actualHz)
{
    if (targetHz == 0u || actualHz == NULL || mode > SD_SPI_PORT_BENCH_MODE_FIFO_512B)
    {
        return -1;
    }

    SdSpiPortBenchMode = mode;
    memset(&SdSpiPortBenchTransferStats, 0, sizeof(SdSpiPortBenchTransferStats));

    return SdSpiPortFactorySetHz(targetHz, actualHz);
}
#endif

#if defined(SUB_BOARD_FACTORY_TEST) || defined(SD_BENCH_TEST)
static uint32_t SdSpiPortPrescalerDivider(uint32_t prescaler)
{
    switch (prescaler)
    {
    case SPI_BAUDRATEPRESCALER_2: return 2u;
    case SPI_BAUDRATEPRESCALER_4: return 4u;
    case SPI_BAUDRATEPRESCALER_8: return 8u;
    case SPI_BAUDRATEPRESCALER_16: return 16u;
    case SPI_BAUDRATEPRESCALER_32: return 32u;
    case SPI_BAUDRATEPRESCALER_64: return 64u;
    case SPI_BAUDRATEPRESCALER_128: return 128u;
    default: return 256u;
    }
}

int SdSpiPortFactorySetHz(uint32_t target_hz, uint32_t *actual_hz)
{
    static const uint32_t candidates[] = {
        SPI_BAUDRATEPRESCALER_2, SPI_BAUDRATEPRESCALER_4,
        SPI_BAUDRATEPRESCALER_8, SPI_BAUDRATEPRESCALER_16,
        SPI_BAUDRATEPRESCALER_32, SPI_BAUDRATEPRESCALER_64,
        SPI_BAUDRATEPRESCALER_128, SPI_BAUDRATEPRESCALER_256,
    };
    uint32_t spi_hz;
    uint32_t prescaler = SPI_BAUDRATEPRESCALER_256;
    uint32_t best_hz = 0u;

    if (target_hz == 0u || actual_hz == NULL)
    {
        return -1;
    }

    SdSpiPortClockInit();
    spi_hz = HAL_RCCEx_GetPeriphCLKFreq(RCC_PERIPHCLK_SPI123);
    if (spi_hz == 0u)
    {
        return -2;
    }

    for (uint32_t i = 0u; i < (uint32_t)(sizeof(candidates) / sizeof(candidates[0])); i++)
    {
        const uint32_t candidate_hz = spi_hz / SdSpiPortPrescalerDivider(candidates[i]);
        if (candidate_hz <= target_hz && candidate_hz > best_hz)
        {
            best_hz = candidate_hz;
            prescaler = candidates[i];
        }
    }
    if (best_hz == 0u)
    {
        best_hz = spi_hz / 256u;
        prescaler = SPI_BAUDRATEPRESCALER_256;
    }
    SdSpiPortApplyPrescaler(prescaler);
    *actual_hz = best_hz;
    return 0;
}
#endif
