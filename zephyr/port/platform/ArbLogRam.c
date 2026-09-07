/* SPDX-License-Identifier: Apache-2.0 */
#include <zephyr/kernel.h>
#include <zephyr/logging/log_backend.h>
#include <zephyr/logging/log_backend_std.h>
#include <zephyr/logging/log_output.h>

#if defined(CONFIG_BOARD_DM_MC02_H7)
#define ARB_LOG_RAM_SIZE 2048u
#else
#define ARB_LOG_RAM_SIZE 512u
#endif

/* 没有串口日志出口时仍要注册后端，否则日志线程启动会触发断言。 */
volatile struct
{
    uint32_t magic;
    uint32_t size;
    uint32_t position;
    uint32_t total;
    uint32_t panic;
    uint8_t data[ARB_LOG_RAM_SIZE];
} ArbLogRam = {.magic = 0x414C4F47u, .size = ARB_LOG_RAM_SIZE};

static struct k_spinlock ArbLogLock;
static uint8_t ArbLogFormatBuffer[64];

static int ArbLogWrite(uint8_t *data, size_t length, void *context)
{
    ARG_UNUSED(context);
    k_spinlock_key_t key = k_spin_lock(&ArbLogLock);
    for (size_t i = 0u; i < length; i++) {
        ArbLogRam.data[ArbLogRam.position] = data[i];
        ArbLogRam.position = (ArbLogRam.position + 1u) % ARB_LOG_RAM_SIZE;
    }
    ArbLogRam.total += length;
    k_spin_unlock(&ArbLogLock, key);
    return (int)length;
}

LOG_OUTPUT_DEFINE(ArbLogOutput, ArbLogWrite, ArbLogFormatBuffer, sizeof(ArbLogFormatBuffer));

static void ArbLogProcess(const struct log_backend *const backend, union log_msg_generic *message)
{
    ARG_UNUSED(backend);
    log_output_msg_process(&ArbLogOutput, &message->log, log_backend_std_get_flags());
}

static void ArbLogPanic(const struct log_backend *const backend)
{
    ARG_UNUSED(backend);
    ArbLogRam.panic = 1u;
    log_output_flush(&ArbLogOutput);
}

static void ArbLogDropped(const struct log_backend *const backend, uint32_t count)
{
    ARG_UNUSED(backend);
    log_output_dropped_process(&ArbLogOutput, count);
}

static const struct log_backend_api ArbLogApi = {
    .process = ArbLogProcess,
    .panic = ArbLogPanic,
    .dropped = ArbLogDropped,
};

LOG_BACKEND_DEFINE(ArbLogBackend, ArbLogApi, true);
