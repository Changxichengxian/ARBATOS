#include <assert.h>
#include <zephyr/kernel.h>
#include "CpuUsage.h"

static k_thread_runtime_stats_t Sample;
static int Failure;
int k_thread_runtime_stats_cpu_get(int cpu, k_thread_runtime_stats_t *stats)
{
    assert(cpu == 0);
    if (Failure) return -1;
    *stats = Sample;
    return 0;
}

int main(void)
{
    Sample.execution_cycles = 100;
    Sample.total_cycles = 40;
    CpuUsageInit();
    Sample.execution_cycles += 1000;
    Sample.total_cycles += 250;
    assert(CpuUsageGetPermille() == 250);
    assert(CpuUsageGetPermille() == 250);
    Failure = 1;
    assert(CpuUsageGetPermille() == 250);
    Failure = 0;
    Sample.execution_cycles += 100;
    Sample.total_cycles += 200;
    assert(CpuUsageGetPermille() == 1000);
    return 0;
}
