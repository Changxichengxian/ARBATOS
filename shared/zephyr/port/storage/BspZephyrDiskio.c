/* FatFs diskio bridge backed by Zephyr disk_access or the legacy SdSpi core. */
#include "BspZephyrStorageConfig.h"

#include "fatfs/ff.h"
#include "fatfs/diskio.h"

#if ARB_STORAGE_USE_DISK_ACCESS
#include <zephyr/drivers/disk.h>
#include <zephyr/storage/disk_access.h>
#else
#include "SdSpi.h"
#endif

#define ARB_STORAGE_PDRV_SD 0u

static volatile DSTATUS ArbStorageStatus = STA_NOINIT;

static DSTATUS ArbStorageUpdateStatus(void)
{
#if ARB_STORAGE_USE_DISK_ACCESS
    const int status = disk_access_status(ARB_STORAGE_DISK_NAME);
    if (status == DISK_STATUS_OK)
    {
        ArbStorageStatus = 0u;
    }
    else
    {
        ArbStorageStatus = STA_NOINIT;
        if ((status & DISK_STATUS_NOMEDIA) != 0)
        {
            ArbStorageStatus |= STA_NODISK;
        }
        if ((status & DISK_STATUS_WR_PROTECT) != 0)
        {
            ArbStorageStatus |= STA_PROTECT;
        }
    }
#else
    if (SdSpiIsReady() != 0)
    {
        ArbStorageStatus &= (BYTE)~STA_NOINIT;
    }
    else
    {
        ArbStorageStatus = STA_NOINIT;
    }
#endif
    return ArbStorageStatus;
}

DSTATUS disk_status(BYTE pdrv)
{
    return (pdrv == ARB_STORAGE_PDRV_SD) ? ArbStorageStatus : STA_NOINIT;
}

DSTATUS disk_initialize(BYTE pdrv)
{
    int ret;

    if (pdrv != ARB_STORAGE_PDRV_SD)
    {
        return STA_NOINIT;
    }
#if ARB_STORAGE_USE_DISK_ACCESS
    ret = disk_access_init(ARB_STORAGE_DISK_NAME);
#else
    ret = SdSpiInit();
#endif
    if (ret != 0)
    {
        ArbStorageStatus = STA_NOINIT;
        return ArbStorageStatus;
    }
    return ArbStorageUpdateStatus();
}

DRESULT disk_read(BYTE pdrv, BYTE *buff, DWORD sector, UINT count)
{
    int ret;

    if (pdrv != ARB_STORAGE_PDRV_SD || buff == NULL || count == 0u)
    {
        return RES_PARERR;
    }
    if ((ArbStorageUpdateStatus() & STA_NOINIT) != 0u)
    {
        return RES_NOTRDY;
    }
#if ARB_STORAGE_USE_DISK_ACCESS
    ret = disk_access_read(ARB_STORAGE_DISK_NAME, (uint8_t *)buff, (uint32_t)sector, (uint32_t)count);
#else
    ret = SdSpiRead((uint8_t *)buff, (uint32_t)sector, (uint32_t)count);
#endif
    if (ret != 0)
    {
        ArbStorageStatus = STA_NOINIT;
        return RES_ERROR;
    }
    return RES_OK;
}

#if FF_FS_READONLY == 0
DRESULT disk_write(BYTE pdrv, const BYTE *buff, DWORD sector, UINT count)
{
    int ret;

    if (pdrv != ARB_STORAGE_PDRV_SD || buff == NULL || count == 0u)
    {
        return RES_PARERR;
    }
    if ((ArbStorageUpdateStatus() & STA_NOINIT) != 0u)
    {
        return RES_NOTRDY;
    }
    if ((ArbStorageStatus & STA_PROTECT) != 0u)
    {
        return RES_WRPRT;
    }
#if ARB_STORAGE_USE_DISK_ACCESS
    ret = disk_access_write(ARB_STORAGE_DISK_NAME, (const uint8_t *)buff, (uint32_t)sector, (uint32_t)count);
#else
    ret = SdSpiWrite((const uint8_t *)buff, (uint32_t)sector, (uint32_t)count);
#endif
    if (ret != 0)
    {
        ArbStorageStatus = STA_NOINIT;
        return RES_ERROR;
    }
    return RES_OK;
}
#endif

DRESULT disk_ioctl(BYTE pdrv, BYTE cmd, void *buff)
{
    int ret;

    if (pdrv != ARB_STORAGE_PDRV_SD)
    {
        return RES_PARERR;
    }
    if ((ArbStorageUpdateStatus() & STA_NOINIT) != 0u)
    {
        return RES_NOTRDY;
    }
    switch (cmd)
    {
    case CTRL_SYNC:
#if ARB_STORAGE_USE_DISK_ACCESS
        ret = disk_access_ioctl(ARB_STORAGE_DISK_NAME, DISK_IOCTL_CTRL_SYNC, NULL);
#else
        ret = SdSpiSync();
#endif
        return (ret == 0) ? RES_OK : RES_ERROR;
    case GET_SECTOR_COUNT:
        if (buff == NULL) { return RES_PARERR; }
#if ARB_STORAGE_USE_DISK_ACCESS
        ret = disk_access_ioctl(ARB_STORAGE_DISK_NAME, DISK_IOCTL_GET_SECTOR_COUNT, buff);
        return (ret == 0) ? RES_OK : RES_ERROR;
#else
        return (SdSpiGetSectorCount((uint32_t *)buff) == 0) ? RES_OK : RES_ERROR;
#endif
    case GET_SECTOR_SIZE:
        if (buff == NULL) { return RES_PARERR; }
#if ARB_STORAGE_USE_DISK_ACCESS
        ret = disk_access_ioctl(ARB_STORAGE_DISK_NAME, DISK_IOCTL_GET_SECTOR_SIZE, buff);
        return (ret == 0) ? RES_OK : RES_ERROR;
#else
        *(WORD *)buff = 512u; return RES_OK;
#endif
    case GET_BLOCK_SIZE:
        if (buff == NULL) { return RES_PARERR; }
#if ARB_STORAGE_USE_DISK_ACCESS
        ret = disk_access_ioctl(ARB_STORAGE_DISK_NAME, DISK_IOCTL_GET_ERASE_BLOCK_SZ, buff);
        return (ret == 0) ? RES_OK : RES_ERROR;
#else
        *(DWORD *)buff = 1u; return RES_OK;
#endif
    default:
        return RES_PARERR;
    }
}
