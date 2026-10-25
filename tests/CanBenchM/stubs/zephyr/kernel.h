#ifndef CAN_BENCH_TEST_KERNEL_H
#define CAN_BENCH_TEST_KERNEL_H
#include <stdbool.h>
#include <stdint.h>
#define ARG_UNUSED(x) ((void)(x))
#define ARRAY_SIZE(x) (sizeof(x) / sizeof((x)[0]))
#define BUILD_ASSERT(x) _Static_assert(x, "layout")
#define K_NO_WAIT 0
extern uint32_t TestNow;
void TestAdvance(uint32_t us);
static inline uint32_t k_cycle_get_32(void) { return TestNow; }
static inline uint64_t k_cyc_to_us_floor64(uint64_t value) { return value; }
static inline uint32_t k_uptime_get_32(void) { return TestNow / 1000u; }
static inline void k_usleep(uint32_t us) { TestAdvance(us); }
static inline void k_msleep(uint32_t ms) { TestAdvance(ms * 1000u); }
static inline unsigned irq_lock(void) { return 0u; }
static inline void irq_unlock(unsigned key) { (void)key; }
#endif
