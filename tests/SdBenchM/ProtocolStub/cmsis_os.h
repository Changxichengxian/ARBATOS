#ifndef CMSIS_OS_H
#define CMSIS_OS_H

#include <stdint.h>

typedef void *osMutexId_t;

typedef struct
{
    const char *name;
} osMutexAttr_t;

#define osWaitForever 0xFFFFFFFFu

osMutexId_t osMutexNew(const osMutexAttr_t *attr);
int32_t osMutexAcquire(osMutexId_t mutex_id, uint32_t timeout);
int32_t osMutexRelease(osMutexId_t mutex_id);
void osDelay(uint32_t ticks);

#endif
