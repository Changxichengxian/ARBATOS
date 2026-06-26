
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
#define SD_SPI_PORT_INIT_MAX_HZ          400000u
#define SD_SPI_PORT_FAST_MAX_HZ          12000000u
#define SD_SPI_PORT_CHUNK_SIZE           64u

static SPI_HandleTypeDef hspi3_sd;
static uint8_t SdSpiPortInited = 0u;
static uint32_t SdSpiPortPrescaler = 0u;
static uint8_t SdSpiPortDummyTx[SD_SPI_PORT_CHUNK_SIZE];

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
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = SD_SPI_PORT_AF;
    HAL_GPIO_Init(SD_SPI_PORT_GPIO, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = SD_SPI_PORT_MISO_PIN;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(SD_SPI_PORT_GPIO, &GPIO_InitStruct);

    HAL_GPIO_WritePin(SD_SPI_PORT_CS_GPIO, SD_SPI_PORT_CS_PIN, GPIO_PIN_SET);

    GPIO_InitStruct.Pin = SD_SPI_PORT_CS_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
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
    if (SdSpiPortInited != 0u && SdSpiPortPrescaler == prescaler)
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
    hspi3_sd.Init.FifoThreshold = SPI_FIFO_THRESHOLD_01DATA;
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

    SdSpiPortEnsureReady();
    (void)HAL_SPI_TransmitReceive(&hspi3_sd, &data, &rx, 1u, SD_SPI_PORT_TIMEOUT_MS);
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
        const uint16_t max_chunk = (uint16_t)SD_SPI_PORT_CHUNK_SIZE;
        const uint16_t chunk = (remaining > max_chunk) ? max_chunk : remaining;
        if (HAL_SPI_TransmitReceive(&hspi3_sd, SdSpiPortDummyTx, &buf[offset], chunk, timeout_ms) != HAL_OK)
        {
            return -2;
        }
        offset = (uint16_t)(offset + chunk);
    }

    return 0;
}

int SdSpiPortTransmit(const uint8_t *buf, uint16_t len, uint32_t timeout_ms)
{
    if (buf == NULL || len == 0u)
    {
        return -1;
    }

    SdSpiPortEnsureReady();
    return (HAL_SPI_Transmit(&hspi3_sd, (uint8_t *)buf, len, timeout_ms) == HAL_OK) ? 0 : -2;
}

void SdSpiPortSetSpeed(SdSpiPortSpeed speed)
{
    const uint32_t target_hz = (speed == SD_SPI_PORT_SPEED_FAST) ? SD_SPI_PORT_FAST_MAX_HZ : SD_SPI_PORT_INIT_MAX_HZ;
    SdSpiPortApplyPrescaler(SdSpiPortPickPrescaler(target_hz));
}
