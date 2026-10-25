#ifndef TEST_ZEPHYR_KERNEL_H
#define TEST_ZEPHYR_KERNEL_H

#include <stdint.h>
typedef int atomic_t;
struct k_mutex { int unused; };
struct k_msgq { int unused; };
#define K_FOREVER (-1)
#define K_MUTEX_DEFINE(name) struct k_mutex name
#define K_MSGQ_DEFINE(name, size, count, align) struct k_msgq name
#define K_THREAD_DEFINE(name, stack, entry, a, b, c, priority, options, delay) \
    static void (*const name)(void *, void *, void *) __attribute__((used)) = (entry)
#define K_NO_WAIT 0
#define ARG_UNUSED(value) ((void)(value))
#define BUILD_ASSERT(condition) _Static_assert((condition), "build assertion")
#define IS_ENABLED(value) (value)
#define __weak __attribute__((weak))
#define __aligned(value) __attribute__((aligned(value)))
static inline int k_mutex_lock(struct k_mutex *mutex, int timeout)
{ (void)mutex; (void)timeout; return 0; }
static inline int k_mutex_unlock(struct k_mutex *mutex)
{ (void)mutex; return 0; }
static inline int atomic_get(const atomic_t *value) { return *value; }
static inline void atomic_set(atomic_t *value, int set) { *value = set; }
static inline void atomic_clear(atomic_t *value) { *value = 0; }
static inline int k_msgq_get(struct k_msgq *queue, void *data, int timeout)
{ (void)queue; (void)data; (void)timeout; return -1; }
static inline int k_msgq_put(struct k_msgq *queue, const void *data, int timeout)
{ (void)queue; (void)data; (void)timeout; return 0; }

#endif
