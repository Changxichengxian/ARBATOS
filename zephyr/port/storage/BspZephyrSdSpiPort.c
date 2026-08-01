/* Zephyr SPI/GPIO implementation for the repository SdSpi protocol layer. */
#include "BspZephyrStorageConfig.h"
#include "BspSdSpiPort.h"

#include <errno.h>
#include <string.h>

#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/util.h>

#define ARB_SD_SPI_CHUNK_SIZE 64u

#ifdef ARB_STORAGE_SPI_NODE
static const struct spi_dt_spec ArbSdSpiSpec =
    SPI_DT_SPEC_GET(ARB_STORAGE_SPI_NODE, SPI_WORD_SET(8) | SPI_TRANSFER_MSB);
static const struct gpio_dt_spec ArbSdCs = SPI_CS_GPIOS_DT_SPEC_GET(ARB_STORAGE_SPI_NODE);
static struct spi_config ArbSdSpiConfig;
static uint8_t ArbSdSpiReady;
static uint8_t ArbSdDummyTx[ARB_SD_SPI_CHUNK_SIZE];

static int ArbSdSpiEnsure(void)
{
    if (ArbSdSpiReady != 0u)
    {
        return 0;
    }
    if (!spi_is_ready_dt(&ArbSdSpiSpec) || ArbSdCs.port == NULL || !device_is_ready(ArbSdCs.port))
    {
        return -ENODEV;
    }
    ArbSdSpiConfig = ArbSdSpiSpec.config;
    /* SdSpi owns CS across command, response and data phases. */
    ArbSdSpiConfig.cs = (struct spi_cs_control){0};
    if (gpio_pin_configure_dt(&ArbSdCs, GPIO_OUTPUT_INACTIVE) != 0)
    {
        return -EIO;
    }
    memset(ArbSdDummyTx, 0xFF, sizeof(ArbSdDummyTx));
    ArbSdSpiReady = 1u;
    return 0;
}

static int ArbSdSpiTransceive(const uint8_t *tx, uint8_t *rx, uint16_t len)
{
    struct spi_buf tx_buf = {.buf = (void *)tx, .len = len};
    struct spi_buf rx_buf = {.buf = rx, .len = len};
    struct spi_buf_set tx_set = {.buffers = &tx_buf, .count = 1u};
    struct spi_buf_set rx_set = {.buffers = &rx_buf, .count = 1u};

    if (tx == NULL || rx == NULL || len == 0u || ArbSdSpiEnsure() != 0)
    {
        return -EINVAL;
    }
    return spi_transceive(ArbSdSpiSpec.bus, &ArbSdSpiConfig, &tx_set, &rx_set);
}
#endif

void SdSpiPortCsHigh(void)
{
#ifdef ARB_STORAGE_SPI_NODE
    if (ArbSdSpiEnsure() == 0)
    {
        (void)gpio_pin_set_dt(&ArbSdCs, 0);
    }
#endif
}

void SdSpiPortCsLow(void)
{
#ifdef ARB_STORAGE_SPI_NODE
    if (ArbSdSpiEnsure() == 0)
    {
        (void)gpio_pin_set_dt(&ArbSdCs, 1);
    }
#endif
}

uint8_t SdSpiPortTxrx(uint8_t data)
{
#ifdef ARB_STORAGE_SPI_NODE
    uint8_t rx = 0xFFu;
    return (ArbSdSpiTransceive(&data, &rx, 1u) == 0) ? rx : 0xFFu;
#else
    ARG_UNUSED(data);
    return 0xFFu;
#endif
}

uint32_t SdSpiPortTickMs(void)
{
    return k_uptime_get_32();
}

int SdSpiPortTxrxDma(const uint8_t *tx, uint8_t *rx, uint16_t len, uint32_t timeout_ms)
{
    ARG_UNUSED(timeout_ms);
#ifdef ARB_STORAGE_SPI_NODE
    if (tx == NULL || rx == NULL || len == 0u)
    {
        return -1;
    }
    return (ArbSdSpiTransceive(tx, rx, len) == 0) ? 0 : -2;
#else
    ARG_UNUSED(tx); ARG_UNUSED(rx); ARG_UNUSED(len);
    return -3;
#endif
}

int SdSpiPortReceive(uint8_t *buf, uint16_t len, uint32_t timeout_ms)
{
    ARG_UNUSED(timeout_ms);
    if (buf == NULL || len == 0u)
    {
        return -1;
    }
#ifdef ARB_STORAGE_SPI_NODE
    uint16_t offset = 0u;

    while (offset < len)
    {
        const uint16_t remain = (uint16_t)(len - offset);
        const uint16_t chunk = (remain > ARB_SD_SPI_CHUNK_SIZE) ? ARB_SD_SPI_CHUNK_SIZE : remain;
        if (ArbSdSpiTransceive(ArbSdDummyTx, &buf[offset], chunk) != 0)
        {
            return -2;
        }
        offset = (uint16_t)(offset + chunk);
    }
    return 0;
#else
    return -3;
#endif
}

int SdSpiPortTransmit(const uint8_t *buf, uint16_t len, uint32_t timeout_ms)
{
    ARG_UNUSED(timeout_ms);
    if (buf == NULL || len == 0u)
    {
        return -1;
    }
#ifdef ARB_STORAGE_SPI_NODE
    uint8_t discard[ARB_SD_SPI_CHUNK_SIZE];
    uint16_t offset = 0u;

    while (offset < len)
    {
        const uint16_t remain = (uint16_t)(len - offset);
        const uint16_t chunk = (remain > ARB_SD_SPI_CHUNK_SIZE) ? ARB_SD_SPI_CHUNK_SIZE : remain;
        if (ArbSdSpiTransceive(&buf[offset], discard, chunk) != 0)
        {
            return -2;
        }
        offset = (uint16_t)(offset + chunk);
    }
    return 0;
#else
    return -3;
#endif
}

void SdSpiPortSetSpeed(SdSpiPortSpeed speed)
{
#ifdef ARB_STORAGE_SPI_NODE
    if (ArbSdSpiEnsure() == 0)
    {
        ArbSdSpiConfig.frequency = (speed == SD_SPI_PORT_SPEED_FAST) ?
            ARB_STORAGE_SPI_FAST_HZ : ARB_STORAGE_SPI_INIT_HZ;
    }
#else
    ARG_UNUSED(speed);
#endif
}
