#ifndef TEST_IMU_SERVICE_ATOMIC_H
#define TEST_IMU_SERVICE_ATOMIC_H

#include <stdint.h>

typedef int32_t atomic_t;
typedef int32_t atomic_val_t;

static inline atomic_val_t atomic_get(const atomic_t *target)
{
    return *target;
}

static inline void atomic_set(atomic_t *target, atomic_val_t value)
{
    *target = value;
}

static inline void atomic_clear(atomic_t *target)
{
    *target = 0;
}

#endif
