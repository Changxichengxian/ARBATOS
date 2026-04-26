/*-----------------------------------------------------------------------*/
/* FatFs system interface (heap + re-entrancy)                            */
/*-----------------------------------------------------------------------*/

#include "ff.h"

#include "mem_mang.h"

#include "FreeRTOS.h"
#include "semphr.h"

#if (FF_USE_LFN == 3)
void *ff_memalloc(UINT msize)
{
    return heap_malloc((uint32_t)msize);
}

void ff_memfree(void *mblock)
{
    heap_free(mblock);
}
#endif

#if FF_FS_REENTRANT
static StaticSemaphore_t s_ff_sync_buf[FF_VOLUMES];
static SemaphoreHandle_t s_ff_sync_handle[FF_VOLUMES];

int ff_cre_syncobj(BYTE vol, FF_SYNC_t *sobj)
{
    if (sobj == NULL)
    {
        return 0;
    }

    if (vol >= FF_VOLUMES)
    {
        *sobj = NULL;
        return 0;
    }

    if (s_ff_sync_handle[vol] == NULL)
    {
        s_ff_sync_handle[vol] = xSemaphoreCreateMutexStatic(&s_ff_sync_buf[vol]);
    }

    *sobj = s_ff_sync_handle[vol];
    return (*sobj != NULL) ? 1 : 0;
}

int ff_del_syncobj(FF_SYNC_t sobj)
{
    if (sobj == NULL)
    {
        return 1;
    }

    for (UINT i = 0u; i < (UINT)FF_VOLUMES; i++)
    {
        if (s_ff_sync_handle[i] == sobj)
        {
            vSemaphoreDelete(sobj);
            s_ff_sync_handle[i] = NULL;
            break;
        }
    }
    return 1;
}

int ff_req_grant(FF_SYNC_t sobj)
{
    if (sobj == NULL)
    {
        return 0;
    }
    return (xSemaphoreTake(sobj, pdMS_TO_TICKS(FF_FS_TIMEOUT)) == pdTRUE) ? 1 : 0;
}

void ff_rel_grant(FF_SYNC_t sobj)
{
    if (sobj != NULL)
    {
        (void)xSemaphoreGive(sobj);
    }
}
#endif
