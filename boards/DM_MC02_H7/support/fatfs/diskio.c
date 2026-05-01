
#include "fatfs/ff.h"
#include "fatfs/diskio.h"

#include "sd_spi.h"

#define DEV_SD 0

static volatile DSTATUS sd_stat = STA_NOINIT;

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

    if (sd_spi_init() == 0)
    {
        sd_stat &= (BYTE)~STA_NOINIT;
    }
    else
    {
        sd_stat = STA_NOINIT;
    }
    return sd_stat;
}

DRESULT disk_read(BYTE pdrv, BYTE *buff, DWORD sector, UINT count)
{
    if (pdrv != DEV_SD || buff == NULL || count == 0u)
    {
        return RES_PARERR;
    }
    if (sd_stat & STA_NOINIT)
    {
        return RES_NOTRDY;
    }

    return (sd_spi_read((uint8_t *)buff, (uint32_t)sector, (uint32_t)count) == 0) ? RES_OK : RES_ERROR;
}

#if FF_FS_READONLY == 0
DRESULT disk_write(BYTE pdrv, const BYTE *buff, DWORD sector, UINT count)
{
    if (pdrv != DEV_SD || buff == NULL || count == 0u)
    {
        return RES_PARERR;
    }
    if (sd_stat & STA_NOINIT)
    {
        return RES_NOTRDY;
    }

    return (sd_spi_write((const uint8_t *)buff, (uint32_t)sector, (uint32_t)count) == 0) ? RES_OK : RES_ERROR;
}
#endif

DRESULT disk_ioctl(BYTE pdrv, BYTE cmd, void *buff)
{
    if (pdrv != DEV_SD)
    {
        return RES_PARERR;
    }
    if (sd_stat & STA_NOINIT)
    {
        return RES_NOTRDY;
    }

    switch (cmd)
    {
    case CTRL_SYNC:
        return (sd_spi_sync() == 0) ? RES_OK : RES_ERROR;

    case GET_SECTOR_COUNT:
    {
        if (buff == NULL)
        {
            return RES_PARERR;
        }
        uint32_t sectors = 0u;
        if (sd_spi_get_sector_count(&sectors) != 0)
        {
            return RES_ERROR;
        }
        *(DWORD *)buff = (DWORD)sectors;
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
