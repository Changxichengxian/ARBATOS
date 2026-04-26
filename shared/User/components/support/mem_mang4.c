
#include "mem_mang.h"

#include <stddef.h>

#include "FreeRTOS.h"

// Use FreeRTOS heap_4 (pvPortMalloc/vPortFree) instead of a dedicated static
// heap buffer to reduce global RAM usage.

void *heap_malloc(uint32_t wanted_size)
{
    if (wanted_size == 0u)
    {
        return NULL;
    }
    return pvPortMalloc((size_t)wanted_size);
}

void heap_free(void *pv)
{
    if (pv == NULL)
    {
        return;
    }
    vPortFree(pv);
}

uint32_t heap_get_free(void)
{
    return (uint32_t)xPortGetFreeHeapSize();
}

uint32_t heap_get_ever_free(void)
{
    return (uint32_t)xPortGetMinimumEverFreeHeapSize();
}

void heap_print_block(void)
{
    // Not supported for FreeRTOS heap backend.
}

