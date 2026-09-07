#ifndef TEST_SOC_H
#define TEST_SOC_H

#include <stddef.h>
#include <stdint.h>

typedef struct
{
    uint32_t APB1HENR;
} TestRcc;

extern TestRcc TestRccInstance;
extern uint32_t TestPrimask;
extern uint32_t TestDsbCount;

#define RCC (&TestRccInstance)
#define RCC_APB1HENR_FDCANEN (1u << 8)
#define ARRAY_SIZE(array) (sizeof(array) / sizeof((array)[0]))
#define BIT(index) (1u << (index))
#define FIELD_GET(mask, value) (((value) & (mask)) >> TestFieldShift(mask))

static inline uint32_t TestFieldShift(uint32_t mask)
{
    uint32_t shift = 0u;
    while ((mask & 1u) == 0u)
    {
        mask >>= 1u;
        shift++;
    }
    return shift;
}

static inline uint32_t __get_PRIMASK(void)
{
    return TestPrimask;
}

static inline void __DSB(void)
{
    TestDsbCount++;
}

#endif
