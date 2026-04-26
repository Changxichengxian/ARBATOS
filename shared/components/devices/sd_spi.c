#include "sd_spi.h"
#include "bsp_sd_spi_port.h"

#include <string.h>

#include "cmsis_os2.h"

#define SD_SPI_SECTOR_SIZE 512u

// Start slow for init, then speed up. Actual prescaler values are owned by the BSP port.
#define SD_SPI_SPEED_INIT SD_SPI_PORT_SPEED_INIT
#define SD_SPI_SPEED_FAST SD_SPI_PORT_SPEED_FAST

#define SD_SPI_CMD0  (0u)   // GO_IDLE_STATE
#define SD_SPI_CMD1  (1u)   // SEND_OP_COND (MMC)
#define SD_SPI_CMD8  (8u)   // SEND_IF_COND
#define SD_SPI_CMD9  (9u)   // SEND_CSD
#define SD_SPI_CMD10 (10u)  // SEND_CID
#define SD_SPI_CMD12 (12u)  // STOP_TRANSMISSION
#define SD_SPI_CMD16 (16u)  // SET_BLOCKLEN
#define SD_SPI_CMD17 (17u)  // READ_SINGLE_BLOCK
#define SD_SPI_CMD24 (24u)  // WRITE_BLOCK
#define SD_SPI_CMD55 (55u)  // APP_CMD
#define SD_SPI_CMD58 (58u)  // READ_OCR

#define SD_SPI_ACMD41 (41u) // SD_SEND_OP_COND

// Data tokens
#define SD_SPI_TOKEN_START_BLOCK 0xFEu

// Card type flags
#define SD_SPI_TYPE_MMC  0x01u
#define SD_SPI_TYPE_SDSC 0x02u
#define SD_SPI_TYPE_SDHC 0x04u

static osMutexId_t sd_spi_mutex = NULL;
static const osMutexAttr_t sd_spi_mutex_attr = {
    .name = "sdSpiMutex",
};

static uint8_t sd_spi_dummy_tx[SD_SPI_SECTOR_SIZE];
static uint8_t sd_spi_dummy_rx[SD_SPI_SECTOR_SIZE];

static uint8_t sd_spi_inited = 0u;
static uint8_t sd_spi_type = 0u;

static void sd_spi_lock(void)
{
    if (sd_spi_mutex == NULL)
    {
        sd_spi_mutex = osMutexNew(&sd_spi_mutex_attr);
    }
    if (sd_spi_mutex != NULL)
    {
        (void)osMutexAcquire(sd_spi_mutex, osWaitForever);
    }
}

static void sd_spi_unlock(void)
{
    if (sd_spi_mutex != NULL)
    {
        (void)osMutexRelease(sd_spi_mutex);
    }
}

static void sd_spi_cs_high(void)
{
    sd_spi_port_cs_high();
}

static void sd_spi_cs_low(void)
{
    sd_spi_port_cs_low();
}

static uint8_t sd_spi_txrx(uint8_t data)
{
    return sd_spi_port_txrx(data);
}

static void sd_spi_tx_dummy(uint32_t count)
{
    while (count--)
    {
        (void)sd_spi_txrx(0xFFu);
    }
}

static uint8_t sd_spi_wait_ready(uint32_t timeout_ms)
{
    const uint32_t start = sd_spi_port_tick_ms();
    do
    {
        if (sd_spi_txrx(0xFFu) == 0xFFu)
        {
            return 1u;
        }
    } while ((uint32_t)(sd_spi_port_tick_ms() - start) < timeout_ms);
    return 0u;
}

static int sd_spi_txrx_dma(const uint8_t *tx, uint8_t *rx, uint16_t len, uint32_t timeout_ms)
{
    return sd_spi_port_txrx_dma(tx, rx, len, timeout_ms);
}

static void sd_spi_set_speed(sd_spi_port_speed_e speed)
{
    sd_spi_port_set_speed(speed);
}

static void sd_spi_deselect(void)
{
    sd_spi_cs_high();
    (void)sd_spi_txrx(0xFFu);
}

static uint8_t sd_spi_select(void)
{
    sd_spi_cs_low();
    (void)sd_spi_txrx(0xFFu);
    if (!sd_spi_wait_ready(200u))
    {
        sd_spi_deselect();
        return 0u;
    }
    return 1u;
}

static uint8_t sd_spi_send_cmd(uint8_t cmd, uint32_t arg)
{
    uint8_t buf[6];
    uint8_t crc = 0x01u;

    if (cmd == SD_SPI_CMD0)
    {
        crc = 0x95u;
    }
    else if (cmd == SD_SPI_CMD8)
    {
        crc = 0x87u;
    }

    buf[0] = (uint8_t)(0x40u | cmd);
    buf[1] = (uint8_t)(arg >> 24u);
    buf[2] = (uint8_t)(arg >> 16u);
    buf[3] = (uint8_t)(arg >> 8u);
    buf[4] = (uint8_t)(arg);
    buf[5] = crc;

    if (cmd == SD_SPI_CMD12)
    {
        // Stop transmission: one extra dummy byte before response.
        (void)sd_spi_txrx(0xFFu);
    }

    for (uint8_t i = 0u; i < sizeof(buf); i++)
    {
        (void)sd_spi_txrx(buf[i]);
    }

    // Wait for response (max 10 bytes)
    for (uint8_t i = 0u; i < 10u; i++)
    {
        const uint8_t r = sd_spi_txrx(0xFFu);
        if ((r & 0x80u) == 0u)
        {
            return r;
        }
    }
    return 0xFFu;
}

static uint8_t sd_spi_send_acmd(uint8_t acmd, uint32_t arg)
{
    uint8_t r = sd_spi_send_cmd(SD_SPI_CMD55, 0u);
    if (r > 1u)
    {
        return r;
    }
    return sd_spi_send_cmd(acmd, arg);
}

static int sd_spi_recv_data(uint8_t *buf, uint32_t len, uint32_t timeout_ms)
{
    const uint32_t start = sd_spi_port_tick_ms();
    uint8_t token;
    do
    {
        token = sd_spi_txrx(0xFFu);
        if (token == SD_SPI_TOKEN_START_BLOCK)
        {
            break;
        }
    } while ((uint32_t)(sd_spi_port_tick_ms() - start) < timeout_ms);

    if (token != SD_SPI_TOKEN_START_BLOCK)
    {
        return -1;
    }

    // Receive data and discard CRC
    if (len <= SD_SPI_SECTOR_SIZE && sd_spi_txrx_dma(sd_spi_dummy_tx, buf, (uint16_t)len, timeout_ms) == 0)
    {
        // ok
    }
    else if (sd_spi_port_receive(buf, (uint16_t)len, 1000u) != 0)
    {
        return -2;
    }
    (void)sd_spi_txrx(0xFFu);
    (void)sd_spi_txrx(0xFFu);
    return 0;
}

static int sd_spi_xmit_data(const uint8_t *buf, uint32_t len)
{
    if (!sd_spi_wait_ready(500u))
    {
        return -1;
    }

    (void)sd_spi_txrx(SD_SPI_TOKEN_START_BLOCK);
    if (len <= SD_SPI_SECTOR_SIZE && sd_spi_txrx_dma(buf, sd_spi_dummy_rx, (uint16_t)len, 1000u) == 0)
    {
        // ok
    }
    else if (sd_spi_port_transmit(buf, (uint16_t)len, 1000u) != 0)
    {
        return -2;
    }

    // Dummy CRC
    (void)sd_spi_txrx(0xFFu);
    (void)sd_spi_txrx(0xFFu);

    const uint8_t resp = sd_spi_txrx(0xFFu);
    if ((resp & 0x1Fu) != 0x05u)
    {
        return -3;
    }

    if (!sd_spi_wait_ready(500u))
    {
        return -4;
    }
    return 0;
}

int sd_spi_init(void)
{
    sd_spi_lock();

    memset(sd_spi_dummy_tx, 0xFF, sizeof(sd_spi_dummy_tx));

    sd_spi_inited = 0u;
    sd_spi_type = 0u;

    sd_spi_set_speed(SD_SPI_SPEED_INIT);
    sd_spi_cs_high();

    // Provide >=74 clocks with CS high.
    sd_spi_tx_dummy(10u);

    // Enter idle
    uint8_t r = 0xFFu;
    for (uint8_t n = 0u; n < 10u; n++)
    {
        if (sd_spi_select())
        {
            r = sd_spi_send_cmd(SD_SPI_CMD0, 0u);
            sd_spi_deselect();
            if (r == 0x01u)
            {
                break;
            }
        }
        osDelay(2);
    }
    if (r != 0x01u)
    {
        sd_spi_unlock();
        return -1;
    }

    // Check SD version via CMD8
    uint8_t ocr[4] = {0};
    if (!sd_spi_select())
    {
        sd_spi_unlock();
        return -2;
    }
    r = sd_spi_send_cmd(SD_SPI_CMD8, 0x000001AAu);
    if (r == 0x01u)
    {
        // R7: 4 bytes
        for (uint8_t i = 0u; i < 4u; i++)
        {
            ocr[i] = sd_spi_txrx(0xFFu);
        }
        sd_spi_deselect();

        if (ocr[2] != 0x01u || ocr[3] != 0xAAu)
        {
            sd_spi_unlock();
            return -3;
        }

        // SDv2: ACMD41 with HCS
        uint32_t start = sd_spi_port_tick_ms();
        do
        {
            if (!sd_spi_select())
            {
                sd_spi_unlock();
                return -4;
            }
            r = sd_spi_send_acmd(SD_SPI_ACMD41, 0x40000000u);
            sd_spi_deselect();
            if (r == 0u)
            {
                break;
            }
            osDelay(2);
        } while ((uint32_t)(sd_spi_port_tick_ms() - start) < 2000u);

        if (r != 0u)
        {
            sd_spi_unlock();
            return -5;
        }

        // Read OCR to detect SDHC
        if (!sd_spi_select())
        {
            sd_spi_unlock();
            return -6;
        }
        r = sd_spi_send_cmd(SD_SPI_CMD58, 0u);
        if (r != 0u)
        {
            sd_spi_deselect();
            sd_spi_unlock();
            return -7;
        }
        for (uint8_t i = 0u; i < 4u; i++)
        {
            ocr[i] = sd_spi_txrx(0xFFu);
        }
        sd_spi_deselect();

        sd_spi_type = (ocr[0] & 0x40u) ? (SD_SPI_TYPE_SDHC) : (SD_SPI_TYPE_SDSC);
    }
    else
    {
        // SDv1 or MMC
        sd_spi_deselect();

        uint8_t cmd = SD_SPI_ACMD41;
        uint8_t type = SD_SPI_TYPE_SDSC;

        uint32_t start = sd_spi_port_tick_ms();
        do
        {
            if (!sd_spi_select())
            {
                sd_spi_unlock();
                return -8;
            }
            r = sd_spi_send_acmd(cmd, 0u);
            sd_spi_deselect();
            if (r <= 1u)
            {
                break;
            }
            osDelay(2);
        } while ((uint32_t)(sd_spi_port_tick_ms() - start) < 2000u);

        if (r > 1u)
        {
            // Try MMC init with CMD1
            cmd = SD_SPI_CMD1;
            type = SD_SPI_TYPE_MMC;
            start = sd_spi_port_tick_ms();
            do
            {
                if (!sd_spi_select())
                {
                    sd_spi_unlock();
                    return -9;
                }
                r = sd_spi_send_cmd(cmd, 0u);
                sd_spi_deselect();
                if (r == 0u)
                {
                    break;
                }
                osDelay(2);
            } while ((uint32_t)(sd_spi_port_tick_ms() - start) < 2000u);
        }

        if (r != 0u)
        {
            sd_spi_unlock();
            return -10;
        }

        // Set block length to 512 for SDSC/MMC
        if (!sd_spi_select())
        {
            sd_spi_unlock();
            return -11;
        }
        r = sd_spi_send_cmd(SD_SPI_CMD16, SD_SPI_SECTOR_SIZE);
        sd_spi_deselect();
        if (r != 0u)
        {
            sd_spi_unlock();
            return -12;
        }

        sd_spi_type = type;
    }

    sd_spi_set_speed(SD_SPI_SPEED_FAST);
    sd_spi_inited = 1u;

    sd_spi_unlock();
    return 0;
}

int sd_spi_is_ready(void)
{
    return (sd_spi_inited != 0u) ? 1 : 0;
}

sd_spi_card_type_e sd_spi_get_card_type(void)
{
    if (!sd_spi_inited)
    {
        return SD_SPI_CARD_NONE;
    }
    if (sd_spi_type & SD_SPI_TYPE_SDHC)
    {
        return SD_SPI_CARD_SDHC;
    }
    if (sd_spi_type & SD_SPI_TYPE_SDSC)
    {
        return SD_SPI_CARD_SDSC;
    }
    if (sd_spi_type & SD_SPI_TYPE_MMC)
    {
        return SD_SPI_CARD_MMC;
    }
    return SD_SPI_CARD_NONE;
}

int sd_spi_sync(void)
{
    if (!sd_spi_inited)
    {
        return -1;
    }

    sd_spi_lock();
    if (!sd_spi_select())
    {
        sd_spi_unlock();
        return -2;
    }
    const uint8_t ready = sd_spi_wait_ready(500u);
    sd_spi_deselect();
    sd_spi_unlock();
    return ready ? 0 : -3;
}

int sd_spi_read(uint8_t *buf, uint32_t sector, uint32_t count)
{
    if (!sd_spi_inited || buf == NULL || count == 0u)
    {
        return -1;
    }

    sd_spi_lock();

    uint32_t addr = sector;
    if ((sd_spi_type & SD_SPI_TYPE_SDHC) == 0u)
    {
        addr *= SD_SPI_SECTOR_SIZE;
    }

    for (uint32_t i = 0u; i < count; i++)
    {
        if (!sd_spi_select())
        {
            sd_spi_unlock();
            return -2;
        }

        const uint8_t r = sd_spi_send_cmd(SD_SPI_CMD17, addr);
        if (r != 0u)
        {
            sd_spi_deselect();
            sd_spi_unlock();
            return -3;
        }

        if (sd_spi_recv_data(buf, SD_SPI_SECTOR_SIZE, 200u) != 0)
        {
            sd_spi_deselect();
            sd_spi_unlock();
            return -4;
        }

        sd_spi_deselect();
        buf += SD_SPI_SECTOR_SIZE;
        addr += ((sd_spi_type & SD_SPI_TYPE_SDHC) != 0u) ? 1u : SD_SPI_SECTOR_SIZE;
    }

    sd_spi_unlock();
    return 0;
}

int sd_spi_write(const uint8_t *buf, uint32_t sector, uint32_t count)
{
    if (!sd_spi_inited || buf == NULL || count == 0u)
    {
        return -1;
    }

    sd_spi_lock();

    uint32_t addr = sector;
    if ((sd_spi_type & SD_SPI_TYPE_SDHC) == 0u)
    {
        addr *= SD_SPI_SECTOR_SIZE;
    }

    for (uint32_t i = 0u; i < count; i++)
    {
        if (!sd_spi_select())
        {
            sd_spi_unlock();
            return -2;
        }

        const uint8_t r = sd_spi_send_cmd(SD_SPI_CMD24, addr);
        if (r != 0u)
        {
            sd_spi_deselect();
            sd_spi_unlock();
            return -3;
        }

        if (sd_spi_xmit_data(buf, SD_SPI_SECTOR_SIZE) != 0)
        {
            sd_spi_deselect();
            sd_spi_unlock();
            return -4;
        }

        sd_spi_deselect();
        buf += SD_SPI_SECTOR_SIZE;
        addr += ((sd_spi_type & SD_SPI_TYPE_SDHC) != 0u) ? 1u : SD_SPI_SECTOR_SIZE;
    }

    sd_spi_unlock();
    return 0;
}

static int sd_spi_read_csd(uint8_t *csd16)
{
    if (!sd_spi_select())
    {
        return -1;
    }

    const uint8_t r = sd_spi_send_cmd(SD_SPI_CMD9, 0u);
    if (r != 0u)
    {
        sd_spi_deselect();
        return -2;
    }

    const int ret = sd_spi_recv_data(csd16, 16u, 200u);
    sd_spi_deselect();
    return ret;
}

int sd_spi_get_sector_count(uint32_t *out_sectors)
{
    if (!sd_spi_inited || out_sectors == NULL)
    {
        return -1;
    }

    sd_spi_lock();

    uint8_t csd[16];
    if (sd_spi_read_csd(csd) != 0)
    {
        sd_spi_unlock();
        return -2;
    }

    uint32_t sectors = 0u;

    if ((csd[0] & 0xC0u) == 0x40u)
    {
        // CSD v2.0 (SDHC/SDXC)
        const uint32_t c_size = ((uint32_t)(csd[7] & 0x3Fu) << 16u) | ((uint32_t)csd[8] << 8u) | (uint32_t)csd[9];
        sectors = (c_size + 1u) * 1024u;
    }
    else
    {
        // CSD v1.0 (SDSC/MMC)
        const uint32_t read_bl_len = (uint32_t)(csd[5] & 0x0Fu);
        const uint32_t c_size = ((uint32_t)(csd[6] & 0x03u) << 10u) | ((uint32_t)csd[7] << 2u) | ((uint32_t)(csd[8] & 0xC0u) >> 6u);
        const uint32_t c_size_mult = ((uint32_t)(csd[9] & 0x03u) << 1u) | ((uint32_t)(csd[10] & 0x80u) >> 7u);

        const uint32_t block_len = 1u << read_bl_len;
        const uint32_t mult = 1u << (c_size_mult + 2u);
        const uint32_t blocknr = (c_size + 1u) * mult;
        const uint32_t capacity = blocknr * block_len;
        sectors = capacity / SD_SPI_SECTOR_SIZE;
    }

    *out_sectors = sectors;

    sd_spi_unlock();
    return (sectors != 0u) ? 0 : -3;
}
