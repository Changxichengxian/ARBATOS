/* 桌面观测独立低优先级运行；ADC 等待不能拖慢 IMU 采样。 */
#include "BspAdc.h"
#include "ElrsTask.h"
#include "ManualInputSnapshot.h"
#include "RobotTargetConfig.h"
#include "RobotTaskBuildConfig.h"
#include "SdLog.h"
#include <math.h>
#include <string.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(board_bench, LOG_LEVEL_INF);

static void MBenchStack(const struct k_thread *thread, void *context)
{
    ARG_UNUSED(context);
    const char *name = k_thread_name_get((k_tid_t)thread);
    if (name != NULL && strcmp(name, "robotTask3") == 0) {
        size_t unused = 0u;
        if (k_thread_stack_space_get(thread, &unused) == 0) {
            LOG_INF("@STACK,%u,%u,%u", k_uptime_get_32(),
                    (unsigned)unused, (unsigned)thread->stack_info.size);
        }
    }
}

static void MBenchRun(void *a, void *b, void *c)
{
    uint8_t slow_divider = 0u;

    ARG_UNUSED(a);
    ARG_UNUSED(b);
    ARG_UNUSED(c);
    for (;;) {
        if (slow_divider == 0u) {
            const float temperature = get_temprate();
            const float battery = get_battery_voltage();
            LOG_INF("@MCU,%u,%d,%d", k_uptime_get_32(),
                    isfinite(temperature) ? (int)(temperature * 1000.0f) : (int)INT32_MIN,
                    isfinite(battery) ? (int)(battery * 1000.0f) : (int)INT32_MIN);
            SdLogStats storage;
            SdLogGetStats(&storage);
            LOG_INF("@SD,%u,%u,%u,%u,%d", k_uptime_get_32(), storage.active,
                    storage.bytes_flushed, storage.dropped, storage.last_error);
            k_thread_foreach(MBenchStack, NULL);
#if ROBOT_TASK_BUILD_ELRS_LINK
            ElrsLinkStats stats;
            ElrsLinkGetStats(&stats);
            LOG_INF("@ELRS,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u",
                    k_uptime_get_32(), stats.port_active, stats.state, stats.uplink_lq,
                    stats.rc_frame_count, stats.link_stats_count, stats.rc_publish_count,
                    stats.crc_error_count, stats.length_error_count, stats.overflow_count,
                    stats.restart_count, stats.stats_timeout_count);
            BspElrsUartDiag uart;
            BspElrsLinkGetDiag(&uart);
            LOG_INF("@UART,%u,%u,%u,%u,%u,%u,%u,%u", k_uptime_get_32(),
                    uart.irqBytes, uart.errors, uart.overrun, uart.framing,
                    uart.parity, uart.noise, uart.lastError);
            ManualInputSnapshot input;
            if (ManualInputSnapshotRead(&input) != 0u) {
                LOG_INF("@INPUT,%u,%u,%u,%u,%d,%d,%d,%d,%d,%u,%u", k_uptime_get_32(),
                        input.online, input.activeSource, (unsigned)input.activeMask,
                        input.manual.rc.ch[0], input.manual.rc.ch[1], input.manual.rc.ch[2],
                        input.manual.rc.ch[3], input.manual.rc.ch[4],
                        (unsigned)input.manual.rc.s[0], (unsigned)input.manual.rc.s[1]);
            }
#endif
        }
#if ROBOT_TASK_BUILD_ELRS_LINK
        ElrsChannelSnapshot channel;
        ElrsChannelGetSnapshot(&channel);
        LOG_INF("@RC,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u",
                k_uptime_get_32(), channel.last_frame_ms, channel.valid, channel.sequence,
                channel.channel[0], channel.channel[1], channel.channel[2], channel.channel[3],
                channel.channel[4], channel.channel[5], channel.channel[6], channel.channel[7],
                channel.channel[8], channel.channel[9], channel.channel[10], channel.channel[11],
                channel.channel[12], channel.channel[13], channel.channel[14], channel.channel[15]);
#endif
        slow_divider = (uint8_t)((slow_divider + 1u) % 10u);
        k_msleep(100);
    }
}

K_THREAD_DEFINE(MBenchThread, 2048, MBenchRun, NULL, NULL, NULL, 14, 0, 2000);
