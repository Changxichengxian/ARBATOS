#include <stdint.h>
typedef struct { uint64_t execution_cycles, total_cycles; } k_thread_runtime_stats_t;
int k_thread_runtime_stats_cpu_get(int cpu, k_thread_runtime_stats_t *stats);
