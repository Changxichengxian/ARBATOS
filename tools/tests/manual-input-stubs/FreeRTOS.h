#ifndef MANUAL_INPUT_TEST_FREERTOS_H
#define MANUAL_INPUT_TEST_FREERTOS_H

#include <stdint.h>

typedef uint32_t TickType_t;
typedef uint32_t UBaseType_t;
typedef void *TimerHandle_t;
typedef struct
{
    uint32_t opaque;
} StaticTimer_t;

#define pdTRUE                 1u
#define portTICK_PERIOD_MS     1u
#define pdMS_TO_TICKS(ms)      ((TickType_t)(ms))

#endif
