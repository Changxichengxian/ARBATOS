/*-----------------------------------------------------------------------*/
/* Low level disk I/O module for FatFs (SDIO/SD)                          */
/*-----------------------------------------------------------------------*/

#include "fatfs/ff.h"
#include "fatfs/diskio.h"

#include "sdio.h"

#define DEV_SD 0

static volatile DSTATUS sd_stat = STA_NOINIT;

static DSTATUS sd_try_ready(void)
{
    const uint32_t ready_deadline_ms = 100u;
    const uint32_t start_ms = HAL_GetTick();

    if (hsd.Instance == NULL)
    {
        sd_stat = STA_NOINIT;
        return sd_stat;
    }

    while ((uint32_t)(HAL_GetTick() - start_ms) < ready_deadline_ms)
    {
        if (HAL_SD_GetCardState(&hsd) == HAL_SD_CARD_TRANSFER)
        {
            sd_stat &= (BYTE)~STA_NOINIT;
            return sd_stat;
        }
    }

    // Only check whether the existing SD handle becomes ready.
    // Reinitializing here can destabilize some cards after board startup.
    sd_stat = STA_NOINIT;
    return sd_stat;
}

DSTATUS disk_status(BYTE pdrv)
{
    if (pdrv != DEV_SD)
    {
        return STA_NOINIT;
    }
    return sd_stat;
}

DSTATUS disk_initialize(BYTE pdrv)
{
    if (pdrv != DEV_SD)
    {
        return STA_NOINIT;
    }

    return sd_try_ready();
}

DRESULT disk_read(BYTE pdrv, BYTE *buff, DWORD sector, UINT count)
{
    if (pdrv != DEV_SD || buff == NULL || count == 0u)
    {
        return RES_PARERR;
    }
    if (sd_try_ready() & STA_NOINIT)
    {
        return RES_NOTRDY;
    }

    if (HAL_SD_ReadBlocks(&hsd, (uint8_t *)buff, (uint32_t)sector, (uint32_t)count, 5000u) != HAL_OK)
    {
        sd_stat = STA_NOINIT;
        return RES_ERROR;
    }
    if (HAL_SD_GetCardState(&hsd) != HAL_SD_CARD_TRANSFER)
    {
        sd_stat = STA_NOINIT;
        return RES_ERROR;
    }

    return RES_OK;
}

#if FF_FS_READONLY == 0
DRESULT disk_write(BYTE pdrv, const BYTE *buff, DWORD sector, UINT count)
{
    if (pdrv != DEV_SD || buff == NULL || count == 0u)
    {
        return RES_PARERR;
    }
    if (sd_try_ready() & STA_NOINIT)
    {
        return RES_NOTRDY;
    }

    if (HAL_SD_WriteBlocks(&hsd, (uint8_t *)buff, (uint32_t)sector, (uint32_t)count, 5000u) != HAL_OK)
    {
        sd_stat = STA_NOINIT;
        return RES_ERROR;
    }
    if (HAL_SD_GetCardState(&hsd) != HAL_SD_CARD_TRANSFER)
    {
        sd_stat = STA_NOINIT;
        return RES_ERROR;
    }

    return RES_OK;
}
#endif

DRESULT disk_ioctl(BYTE pdrv, BYTE cmd, void *buff)
{
    if (pdrv != DEV_SD)
    {
        return RES_PARERR;
    }
    if (sd_try_ready() & STA_NOINIT)
    {
        return RES_NOTRDY;
    }

    switch (cmd)
    {
    case CTRL_SYNC:
        if (HAL_SD_GetCardState(&hsd) == HAL_SD_CARD_TRANSFER)
        {
            return RES_OK;
        }
        sd_stat = STA_NOINIT;
        return RES_ERROR;

    case GET_SECTOR_COUNT:
    {
        if (buff == NULL)
        {
            return RES_PARERR;
        }

        HAL_SD_CardInfoTypeDef info;
        if (HAL_SD_GetCardInfo(&hsd, &info) != HAL_OK)
        {
            sd_stat = STA_NOINIT;
            return RES_ERROR;
        }
        *(DWORD *)buff = (DWORD)info.LogBlockNbr;
        return RES_OK;
    }

    case GET_SECTOR_SIZE:
        if (buff == NULL)
        {
            return RES_PARERR;
        }
        *(WORD *)buff = 512u;
        return RES_OK;

    case GET_BLOCK_SIZE:
        if (buff == NULL)
        {
            return RES_PARERR;
        }
        *(DWORD *)buff = 1u;
        return RES_OK;

    default:
        return RES_PARERR;
    }
}
