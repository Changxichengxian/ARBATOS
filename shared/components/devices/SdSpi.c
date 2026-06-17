#include "SdSpi.h"
#include "BspSdSpiPort.h"

#include <string.h>

#include "cmsis_os.h"

#define SD_SPI_SECTOR_SIZE 512u

#ifndef SD_SPI_SELECT_READY_TIMEOUT_MS
#define SD_SPI_SELECT_READY_TIMEOUT_MS 2000u
#endif

#ifndef SD_SPI_WRITE_READY_TIMEOUT_MS
#if defined(STM32F407xx)
#define SD_SPI_WRITE_READY_TIMEOUT_MS 10000u
#else
#define SD_SPI_WRITE_READY_TIMEOUT_MS 5000u
#endif
#endif

#ifndef SD_SPI_TRANSFER_TIMEOUT_MS
#define SD_SPI_TRANSFER_TIMEOUT_MS 1000u
#endif

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

static osMutexId_t SdSpiMutex = NULL;
static const osMutexAttr_t SdSpiMutexAttr = {
    .name = "sdSpiMutex",
};

static uint8_t SdSpiDummyTx[SD_SPI_SECTOR_SIZE];
static uint8_t SdSpiDummyRx[SD_SPI_SECTOR_SIZE];

static uint8_t SdSpiInited = 0u;
static uint8_t SdSpiType = 0u;

static void SdSpiLock(void)
{
    if (SdSpiMutex == NULL)
    {
        SdSpiMutex = osMutexNew(&SdSpiMutexAttr);
    }
    if (SdSpiMutex != NULL)
    {
        (void)osMutexAcquire(SdSpiMutex, osWaitForever);
    }
}

static void SdSpiUnlock(void)
{
    if (SdSpiMutex != NULL)
    {
        (void)osMutexRelease(SdSpiMutex);
    }
}

static void SdSpiCsHigh(void)
{
    SdSpiPortCsHigh();
}

static void SdSpiCsLow(void)
{
    SdSpiPortCsLow();
}

static uint8_t SdSpiTxrx(uint8_t data)
{
    return SdSpiPortTxrx(data);
}

static void SdSpiTxDummy(uint32_t count)
{
    while (count--)
    {
        (void)SdSpiTxrx(0xFFu);
    }
}

static uint8_t SdSpiWaitReady(uint32_t timeout_ms)
{
    const uint32_t start = SdSpiPortTickMs();
    do
    {
        if (SdSpiTxrx(0xFFu) == 0xFFu)
        {
            return 1u;
        }
    } while ((uint32_t)(SdSpiPortTickMs() - start) < timeout_ms);
    return 0u;
}

static int SdSpiTxrxDma(const uint8_t *tx, uint8_t *rx, uint16_t len, uint32_t timeout_ms)
{
    return SdSpiPortTxrxDma(tx, rx, len, timeout_ms);
}

static void SdSpiSetSpeed(SdSpiPortSpeed speed)
{
    SdSpiPortSetSpeed(speed);
}

static void SdSpiDeselect(void)
{
    SdSpiCsHigh();
    (void)SdSpiTxrx(0xFFu);
}

static uint8_t SdSpiSelect(void)
{
    SdSpiCsLow();
    (void)SdSpiTxrx(0xFFu);
    if (!SdSpiWaitReady(SD_SPI_SELECT_READY_TIMEOUT_MS))
    {
        SdSpiDeselect();
        return 0u;
    }
    return 1u;
}

static uint8_t SdSpiSendCmd(uint8_t cmd, uint32_t arg)
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
        (void)SdSpiTxrx(0xFFu);
    }

    for (uint8_t i = 0u; i < sizeof(buf); i++)
    {
        (void)SdSpiTxrx(buf[i]);
    }

    // Wait for response (max 10 bytes)
    for (uint8_t i = 0u; i < 10u; i++)
    {
        const uint8_t r = SdSpiTxrx(0xFFu);
        if ((r & 0x80u) == 0u)
        {
            return r;
        }
    }
    return 0xFFu;
}

static uint8_t SdSpiSendAcmd(uint8_t acmd, uint32_t arg)
{
    uint8_t r = SdSpiSendCmd(SD_SPI_CMD55, 0u);
    if (r > 1u)
    {
        return r;
    }
    return SdSpiSendCmd(acmd, arg);
}

static int SdSpiRecvData(uint8_t *buf, uint32_t len, uint32_t timeout_ms)
{
    const uint32_t start = SdSpiPortTickMs();
    uint8_t token;
    do
    {
        token = SdSpiTxrx(0xFFu);
        if (token == SD_SPI_TOKEN_START_BLOCK)
        {
            break;
        }
    } while ((uint32_t)(SdSpiPortTickMs() - start) < timeout_ms);

    if (token != SD_SPI_TOKEN_START_BLOCK)
    {
        return -1;
    }

    // Receive data and discard CRC
    if (len <= SD_SPI_SECTOR_SIZE && SdSpiTxrxDma(SdSpiDummyTx, buf, (uint16_t)len, timeout_ms) == 0)
    {
        // ok
    }
    else if (SdSpiPortReceive(buf, (uint16_t)len, 1000u) != 0)
    {
        return -2;
    }
    (void)SdSpiTxrx(0xFFu);
    (void)SdSpiTxrx(0xFFu);
    return 0;
}

static int SdSpiXmitData(const uint8_t *buf, uint32_t len)
{
    if (!SdSpiWaitReady(SD_SPI_WRITE_READY_TIMEOUT_MS))
    {
        return -1;
    }

    (void)SdSpiTxrx(SD_SPI_TOKEN_START_BLOCK);
    if (len <= SD_SPI_SECTOR_SIZE &&
        SdSpiTxrxDma(buf, SdSpiDummyRx, (uint16_t)len, SD_SPI_TRANSFER_TIMEOUT_MS) == 0)
    {
        // ok
    }
    else if (SdSpiPortTransmit(buf, (uint16_t)len, SD_SPI_TRANSFER_TIMEOUT_MS) != 0)
    {
        return -2;
    }

    // Dummy CRC
    (void)SdSpiTxrx(0xFFu);
    (void)SdSpiTxrx(0xFFu);

    const uint8_t resp = SdSpiTxrx(0xFFu);
    if ((resp & 0x1Fu) != 0x05u)
    {
        return -3;
    }

    if (!SdSpiWaitReady(SD_SPI_WRITE_READY_TIMEOUT_MS))
    {
        return -4;
    }
    return 0;
}

int SdSpiInit(void)
{
    SdSpiLock();

    memset(SdSpiDummyTx, 0xFF, sizeof(SdSpiDummyTx));

    SdSpiInited = 0u;
    SdSpiType = 0u;

    SdSpiSetSpeed(SD_SPI_SPEED_INIT);
    SdSpiCsHigh();

    // Provide >=74 clocks with CS high.
    SdSpiTxDummy(10u);

    // Enter idle
    uint8_t r = 0xFFu;
    for (uint8_t n = 0u; n < 10u; n++)
    {
        if (SdSpiSelect())
        {
            r = SdSpiSendCmd(SD_SPI_CMD0, 0u);
            SdSpiDeselect();
            if (r == 0x01u)
            {
                break;
            }
        }
        osDelay(2);
    }
    if (r != 0x01u)
    {
        SdSpiUnlock();
        return -1;
    }

    // Check SD version via CMD8
    uint8_t ocr[4] = {0};
    if (!SdSpiSelect())
    {
        SdSpiUnlock();
        return -2;
    }
    r = SdSpiSendCmd(SD_SPI_CMD8, 0x000001AAu);
    if (r == 0x01u)
    {
        // R7: 4 bytes
        for (uint8_t i = 0u; i < 4u; i++)
        {
            ocr[i] = SdSpiTxrx(0xFFu);
        }
        SdSpiDeselect();

        if (ocr[2] != 0x01u || ocr[3] != 0xAAu)
        {
            SdSpiUnlock();
            return -3;
        }

        // SDv2: ACMD41 with HCS
        uint32_t start = SdSpiPortTickMs();
        do
        {
            if (!SdSpiSelect())
            {
                SdSpiUnlock();
                return -4;
            }
            r = SdSpiSendAcmd(SD_SPI_ACMD41, 0x40000000u);
            SdSpiDeselect();
            if (r == 0u)
            {
                break;
            }
            osDelay(2);
        } while ((uint32_t)(SdSpiPortTickMs() - start) < 2000u);

        if (r != 0u)
        {
            SdSpiUnlock();
            return -5;
        }

        // Read OCR to detect SDHC
        if (!SdSpiSelect())
        {
            SdSpiUnlock();
            return -6;
        }
        r = SdSpiSendCmd(SD_SPI_CMD58, 0u);
        if (r != 0u)
        {
            SdSpiDeselect();
            SdSpiUnlock();
            return -7;
        }
        for (uint8_t i = 0u; i < 4u; i++)
        {
            ocr[i] = SdSpiTxrx(0xFFu);
        }
        SdSpiDeselect();

        SdSpiType = (ocr[0] & 0x40u) ? (SD_SPI_TYPE_SDHC) : (SD_SPI_TYPE_SDSC);
    }
    else
    {
        // SDv1 or MMC
        SdSpiDeselect();

        uint8_t cmd = SD_SPI_ACMD41;
        uint8_t type = SD_SPI_TYPE_SDSC;

        uint32_t start = SdSpiPortTickMs();
        do
        {
            if (!SdSpiSelect())
            {
                SdSpiUnlock();
                return -8;
            }
            r = SdSpiSendAcmd(cmd, 0u);
            SdSpiDeselect();
            if (r <= 1u)
            {
                break;
            }
            osDelay(2);
        } while ((uint32_t)(SdSpiPortTickMs() - start) < 2000u);

        if (r > 1u)
        {
            // Try MMC init with CMD1
            cmd = SD_SPI_CMD1;
            type = SD_SPI_TYPE_MMC;
            start = SdSpiPortTickMs();
            do
            {
                if (!SdSpiSelect())
                {
                    SdSpiUnlock();
                    return -9;
                }
                r = SdSpiSendCmd(cmd, 0u);
                SdSpiDeselect();
                if (r == 0u)
                {
                    break;
                }
                osDelay(2);
            } while ((uint32_t)(SdSpiPortTickMs() - start) < 2000u);
        }

        if (r != 0u)
        {
            SdSpiUnlock();
            return -10;
        }

        // Set block length to 512 for SDSC/MMC
        if (!SdSpiSelect())
        {
            SdSpiUnlock();
            return -11;
        }
        r = SdSpiSendCmd(SD_SPI_CMD16, SD_SPI_SECTOR_SIZE);
        SdSpiDeselect();
        if (r != 0u)
        {
            SdSpiUnlock();
            return -12;
        }

        SdSpiType = type;
    }

    SdSpiSetSpeed(SD_SPI_SPEED_FAST);
    SdSpiInited = 1u;

    SdSpiUnlock();
    return 0;
}

int SdSpiIsReady(void)
{
    return (SdSpiInited != 0u) ? 1 : 0;
}

SdSpiCardType SdSpiGetCardType(void)
{
    if (!SdSpiInited)
    {
        return SD_SPI_CARD_NONE;
    }
    if (SdSpiType & SD_SPI_TYPE_SDHC)
    {
        return SD_SPI_CARD_SDHC;
    }
    if (SdSpiType & SD_SPI_TYPE_SDSC)
    {
        return SD_SPI_CARD_SDSC;
    }
    if (SdSpiType & SD_SPI_TYPE_MMC)
    {
        return SD_SPI_CARD_MMC;
    }
    return SD_SPI_CARD_NONE;
}

int SdSpiSync(void)
{
    if (!SdSpiInited)
    {
        return -1;
    }

    SdSpiLock();
    if (!SdSpiSelect())
    {
        SdSpiUnlock();
        return -2;
    }
    const uint8_t ready = SdSpiWaitReady(SD_SPI_WRITE_READY_TIMEOUT_MS);
    SdSpiDeselect();
    SdSpiUnlock();
    return ready ? 0 : -3;
}

int SdSpiRead(uint8_t *buf, uint32_t sector, uint32_t count)
{
    if (!SdSpiInited || buf == NULL || count == 0u)
    {
        return -1;
    }

    SdSpiLock();

    uint32_t addr = sector;
    if ((SdSpiType & SD_SPI_TYPE_SDHC) == 0u)
    {
        addr *= SD_SPI_SECTOR_SIZE;
    }

    for (uint32_t i = 0u; i < count; i++)
    {
        if (!SdSpiSelect())
        {
            SdSpiUnlock();
            return -2;
        }

        const uint8_t r = SdSpiSendCmd(SD_SPI_CMD17, addr);
        if (r != 0u)
        {
            SdSpiDeselect();
            SdSpiUnlock();
            return -3;
        }

        if (SdSpiRecvData(buf, SD_SPI_SECTOR_SIZE, 200u) != 0)
        {
            SdSpiDeselect();
            SdSpiUnlock();
            return -4;
        }

        SdSpiDeselect();
        buf += SD_SPI_SECTOR_SIZE;
        addr += ((SdSpiType & SD_SPI_TYPE_SDHC) != 0u) ? 1u : SD_SPI_SECTOR_SIZE;
    }

    SdSpiUnlock();
    return 0;
}

int SdSpiWrite(const uint8_t *buf, uint32_t sector, uint32_t count)
{
    if (!SdSpiInited || buf == NULL || count == 0u)
    {
        return -1;
    }

    SdSpiLock();

    uint32_t addr = sector;
    if ((SdSpiType & SD_SPI_TYPE_SDHC) == 0u)
    {
        addr *= SD_SPI_SECTOR_SIZE;
    }

    for (uint32_t i = 0u; i < count; i++)
    {
        if (!SdSpiSelect())
        {
            SdSpiUnlock();
            return -2;
        }

        const uint8_t r = SdSpiSendCmd(SD_SPI_CMD24, addr);
        if (r != 0u)
        {
            SdSpiDeselect();
            SdSpiUnlock();
            return -3;
        }

        if (SdSpiXmitData(buf, SD_SPI_SECTOR_SIZE) != 0)
        {
            SdSpiDeselect();
            SdSpiUnlock();
            return -4;
        }

        SdSpiDeselect();
        buf += SD_SPI_SECTOR_SIZE;
        addr += ((SdSpiType & SD_SPI_TYPE_SDHC) != 0u) ? 1u : SD_SPI_SECTOR_SIZE;
    }

    SdSpiUnlock();
    return 0;
}

static int SdSpiReadCsd(uint8_t *csd16)
{
    if (!SdSpiSelect())
    {
        return -1;
    }

    const uint8_t r = SdSpiSendCmd(SD_SPI_CMD9, 0u);
    if (r != 0u)
    {
        SdSpiDeselect();
        return -2;
    }

    const int ret = SdSpiRecvData(csd16, 16u, 200u);
    SdSpiDeselect();
    return ret;
}

int SdSpiGetSectorCount(uint32_t *out_sectors)
{
    if (!SdSpiInited || out_sectors == NULL)
    {
        return -1;
    }

    SdSpiLock();

    uint8_t csd[16];
    if (SdSpiReadCsd(csd) != 0)
    {
        SdSpiUnlock();
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

    SdSpiUnlock();
    return (sectors != 0u) ? 0 : -3;
}
