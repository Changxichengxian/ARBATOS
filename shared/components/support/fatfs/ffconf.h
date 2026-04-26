/*---------------------------------------------------------------------------/
/  FatFs Functional Configurations (project-local)
/---------------------------------------------------------------------------*/

#ifndef _FFCONF_DEFINED
#define _FFCONF_DEFINED

#define FFCONF_DEF 86604 /* Matches ff.c/ff.h revision */

/*---------------------------------------------------------------------------/
/ Function Configurations
/---------------------------------------------------------------------------*/

#define FF_FS_READONLY 0
#define FF_FS_MINIMIZE 0
#define FF_USE_STRFUNC 0
#define FF_USE_FIND 0
#define FF_USE_MKFS 0
#define FF_USE_FASTSEEK 0
#define FF_USE_EXPAND 0
#define FF_USE_CHMOD 0
#define FF_USE_LABEL 0
#define FF_USE_FORWARD 0

/*---------------------------------------------------------------------------/
/ Locale and Namespace Configurations
/---------------------------------------------------------------------------*/

#define FF_CODE_PAGE 437

// Enable long file names (LFN). Requires ffunicode.c.
#define FF_USE_LFN 3
#define FF_MAX_LFN 64
#define FF_LFN_UNICODE 0
#define FF_LFN_BUF 64
#define FF_SFN_BUF 12
#define FF_STRF_ENCODE 0

#define FF_FS_RPATH 0

/*---------------------------------------------------------------------------/
/ Drive/Volume Configurations
/---------------------------------------------------------------------------*/

#define FF_VOLUMES 1
#define FF_STR_VOLUME_ID 0
#define FF_VOLUME_STRS "SD"
#define FF_MULTI_PARTITION 0

#define FF_MIN_SS 512
#define FF_MAX_SS 512

#define FF_USE_TRIM 0
#define FF_FS_NOFSINFO 0

/*---------------------------------------------------------------------------/
/ System Configurations
/---------------------------------------------------------------------------*/

#define FF_FS_TINY 1
#define FF_FS_EXFAT 1

// No RTC: use a fixed timestamp.
#define FF_FS_NORTC 1
#define FF_NORTC_MON 1
#define FF_NORTC_MDAY 1
#define FF_NORTC_YEAR 2025

#define FF_FS_LOCK 8

/*---------------------------------------------------------------------------/
/ OS/Thread Safety
/---------------------------------------------------------------------------*/

#include "FreeRTOS.h"
#include "semphr.h"

#define FF_FS_REENTRANT 1
#define FF_FS_TIMEOUT 1000 /* ms */
#define FF_SYNC_t SemaphoreHandle_t

#endif /* _FFCONF_DEFINED */
