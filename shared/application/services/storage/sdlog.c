/*
 * SPDX-FileCopyrightText: 2026 陈轩 <2811158416@qq.com>
 * SPDX-License-Identifier: Apache-2.0
 *
 * First published in this repository: 2026-04-06
 * Use of this file is governed by the LICENSE file in the repository root.
 */

/*
 * 阅读地图：
 * - 前段：RAM 环形缓存、文件序号索引、CRC 和变长整数工具。
 * - 中段：小型 LZ4 块压缩、数据块写入。
 * - 后段：打开下一个日志文件、start/stop/write/isr_write/poll 运行接口。
 * - 设计重点：写入接口不阻塞，sdlog_poll() 在低优先级任务里慢慢刷出。
 */

#include "sdlog.h"

#include <stdio.h>
#include <string.h>

#include "cmsis_os.h"
#include "bsp_time.h"
#include "sdcard.h"
#include "rt_profiler.h"
#include "config.h"
#include "control_manager.h"
#include "motor_instance.h"
#include "robot_device_config.h"
#include "robot_task_profile.h"

#if defined(__CC_ARM)
#include "../../../generated/build_info_autogen.h"
#elif defined(__has_include)
#if __has_include("../../../generated/build_info_autogen.h")
#include "../../../generated/build_info_autogen.h"
#endif
#endif

#include "fatfs/ff.h"

#include "FreeRTOS.h"
#include "task.h"

#ifndef SDLOG_BUF_SIZE
#if defined(STM32H723xx) || defined(STM32H7xx) || defined(STM32H7)
#define SDLOG_BUF_SIZE (64u * 1024u)
#elif defined(STM32F407xx)
#define SDLOG_BUF_SIZE (32u * 1024u)
#else
#define SDLOG_BUF_SIZE (16u * 1024u)
#endif
#endif

#ifndef SDLOG_FLUSH_CHUNK_MAX
#if defined(STM32F407xx)
#define SDLOG_FLUSH_CHUNK_MAX 512u
#else
#define SDLOG_FLUSH_CHUNK_MAX (2u * 1024u)
#endif
#endif

#ifndef SDLOG_SYNC_PERIOD_MS
#if defined(STM32H723xx) || defined(STM32H7xx) || defined(STM32H7) || defined(STM32F407xx)
#define SDLOG_SYNC_PERIOD_MS 5000u
#else
#define SDLOG_SYNC_PERIOD_MS 1000u
#endif
#endif

#ifndef SDLOG_FLUSH_BLOCKS_PER_POLL
#define SDLOG_FLUSH_BLOCKS_PER_POLL 4u
#endif

#ifndef SDLOG_ENABLE_COMPRESSION
#define SDLOG_ENABLE_COMPRESSION 0u
#endif

#ifndef ARBATOS_TARGET_NAME
#define ARBATOS_TARGET_NAME "unknown-target"
#endif

#ifndef ARBATOS_BOARD_NAME
#define ARBATOS_BOARD_NAME "unknown-board"
#endif

#ifndef ARBATOS_GIT_SHA
#define ARBATOS_GIT_SHA "unknown"
#endif

#ifndef ARBATOS_BUILD_DIRTY
#define ARBATOS_BUILD_DIRTY 0u
#endif

#ifndef ARBATOS_BUILD_DATE
#define ARBATOS_BUILD_DATE __DATE__
#endif

#ifndef ARBATOS_BUILD_TIME
#define ARBATOS_BUILD_TIME __TIME__
#endif

#define SDLOG_RESTART_REASON_RUNTIME_IO 1u
#define SDLOG_RESTART_REASON_OPEN_FAIL 2u
#define SDLOG_RESTART_REASON_STARTUP_HEADER 3u
#define SDLOG_RESTART_REASON_STARTUP_BLOCK 4u
#define SDLOG_RESTART_REASON_STARTUP_SYNC 5u
#define SDLOG_RESTART_REASON_STOP 6u

static FIL sdlog_fp;
static volatile uint8_t sdlog_active = 0u;
static volatile uint32_t sdlog_dropped = 0u;
static uint32_t sdlog_last_sync_ms = 0u;
static volatile uint32_t sdlog_bytes_flushed = 0u;
static volatile int32_t sdlog_last_error = 0;
static uint16_t sdlog_prev_error_reason = 0u;
static int32_t sdlog_prev_error_code = 0;
static uint32_t sdlog_prev_error_bytes = 0u;
static uint32_t sdlog_last_tick_ms = 0u;

__attribute__((section(".ccmram"))) static uint8_t sdlog_buf[SDLOG_BUF_SIZE];
static volatile uint32_t sdlog_head = 0u;
static volatile uint32_t sdlog_tail = 0u;

uint8_t sdlog_high_rate_divider(void)
{
    const uint8_t div = g_config.sdlog.high_rate_div;
    if (div >= 4u)
    {
        return 4u;
    }
    if (div == 3u)
    {
        return 4u;
    }
    if (div >= 2u)
    {
        return 2u;
    }
    return 1u;
}

static void sdlog_close_on_error(void);

// Persisted "next log index" to avoid scanning 0:/ on every boot.
#define SDLOG_INDEX_FILE_PATH "0:/sdlog_index.bin"
#define SDLOG_INDEX_MAGIC 0x58494453u /* 'SDIX' */

typedef struct __attribute__((packed))
{
    uint32_t magic;     // SDLOG_INDEX_MAGIC
    uint32_t next_idx;  // next sdlog_XXXX.bin index
    uint32_t crc32;     // CRC32(magic||next_idx) CRC-32/IEEE
    uint32_t reserved;  // reserved for future
} sdlog_index_file_t;

// Optional block compression (LZ4 block format, implemented locally for small blocks).
#if SDLOG_ENABLE_COMPRESSION
#define SDLOG_LZ4_HASH_BITS 11u
#define SDLOG_LZ4_HASH_SIZE (1u << SDLOG_LZ4_HASH_BITS)
#define SDLOG_LZ4_MAX_OUTPUT(n) ((n) + ((n) / 255u) + 16u)
#endif

static uint8_t sdlog_flush_in[SDLOG_FLUSH_CHUNK_MAX];
#if SDLOG_ENABLE_COMPRESSION
static uint8_t sdlog_flush_out[SDLOG_LZ4_MAX_OUTPUT(SDLOG_FLUSH_CHUNK_MAX)];
__attribute__((section(".ccmram"))) static uint16_t sdlog_lz4_hash[SDLOG_LZ4_HASH_SIZE];
#endif

static uint32_t sdlog_used_bytes(uint32_t head, uint32_t tail)
{
    if (head >= tail)
    {
        return head - tail;
    }
    return SDLOG_BUF_SIZE - (tail - head);
}

static uint32_t sdlog_free_bytes(uint32_t head, uint32_t tail)
{
    return (SDLOG_BUF_SIZE - 1u) - sdlog_used_bytes(head, tail);
}

static void sdlog_ring_write_bytes_locked(const uint8_t *src, uint32_t len)
{
    uint32_t head = sdlog_head;
    const uint32_t to_end = SDLOG_BUF_SIZE - head;

    if (len <= to_end)
    {
        memcpy(&sdlog_buf[head], src, len);
        head += len;
        if (head == SDLOG_BUF_SIZE)
        {
            head = 0u;
        }
    }
    else
    {
        memcpy(&sdlog_buf[head], src, to_end);
        const uint32_t left = len - to_end;
        memcpy(&sdlog_buf[0], src + to_end, left);
        head = left;
    }

    sdlog_head = head;
}

#if SDLOG_ENABLE_COMPRESSION
static uint32_t sdlog_read_u32_le_unaligned(const uint8_t *p)
{
    uint32_t v;
    memcpy(&v, p, sizeof(v));
    return v;
}
#endif

static uint8_t sdlog_write_var_u32(uint8_t *dst, uint32_t v)
{
    uint8_t n = 0u;
    while (v >= 0x80u)
    {
        dst[n++] = (uint8_t)((v & 0x7Fu) | 0x80u);
        v >>= 7u;
    }
    dst[n++] = (uint8_t)(v & 0x7Fu);
    return n;
}

static uint32_t sdlog_crc32_ieee(const uint8_t *data, uint32_t len)
{
    // CRC-32/ISO-HDLC (IEEE 802.3): poly 0x04C11DB7, refin/refout, init/xorout 0xFFFFFFFF.
    // Nibble table for the reversed polynomial 0xEDB88320.
    static const uint32_t t[16] = {
        0x00000000u, 0x1DB71064u, 0x3B6E20C8u, 0x26D930ACu,
        0x76DC4190u, 0x6B6B51F4u, 0x4DB26158u, 0x5005713Cu,
        0xEDB88320u, 0xF00F9344u, 0xD6D6A3E8u, 0xCB61B38Cu,
        0x9B64C2B0u, 0x86D3D2D4u, 0xA00AE278u, 0xBDBDF21Cu,
    };

    if (data == NULL || len == 0u)
    {
        return 0u;
    }

    uint32_t crc = 0xFFFFFFFFu;
    for (uint32_t i = 0u; i < len; i++)
    {
        crc ^= (uint32_t)data[i];
        crc = (crc >> 4u) ^ t[crc & 0x0Fu];
        crc = (crc >> 4u) ^ t[crc & 0x0Fu];
    }
    return ~crc;
}

static void sdlog_copy_cstr(char *dst, uint32_t dst_len, const char *src)
{
    if (dst == NULL || dst_len == 0u)
    {
        return;
    }

    if (src == NULL)
    {
        src = "";
    }

    uint32_t i = 0u;
    while (i + 1u < dst_len && src[i] != '\0')
    {
        dst[i] = src[i];
        i++;
    }
    dst[i] = '\0';
}

static void sdlog_fill_build_info(sdlog_build_info_t *out)
{
    uint8_t rt_profiler_count = 0u;

    if (out == NULL)
    {
        return;
    }

    memset(out, 0, sizeof(*out));
    out->version = SDLOG_BUILD_INFO_VERSION;
    out->header_size = (uint16_t)sizeof(*out);
    out->schema_version = SDLOG_SCHEMA_VERSION;
    out->config_size = (uint32_t)sizeof(g_config);
    out->config_crc32 = sdlog_crc32_ieee((const uint8_t *)&g_config, (uint32_t)sizeof(g_config));
    out->task_module_count = g_config.profile.task_module_count;
    out->high_rate_div = sdlog_high_rate_divider();
    out->compression_enabled = (uint8_t)(SDLOG_ENABLE_COMPRESSION ? 1u : 0u);
    out->build_dirty = (uint8_t)(ARBATOS_BUILD_DIRTY ? 1u : 0u);
    out->runtime_device_count = robot_config_device_count();
    out->motor_instance_count = motor_instance_count();
    out->controller_count = control_manager_registered_count();
    out->profile_kind = (uint8_t)robot_profile_kind();
    out->board_kind = (uint8_t)robot_board_kind();
    (void)rt_profiler_descriptors(&rt_profiler_count);
    out->rt_profiler_count = rt_profiler_count;
    out->board_can_bus_count = robot_board_can_bus_count();
    out->board_cpu_hz = robot_board_cpu_hz();
    {
        const uint8_t count = g_config.profile.task_module_count;
        const uint8_t limit = (count > ROBOT_TASK_MODULE_MAX) ? ROBOT_TASK_MODULE_MAX : count;
        for (uint8_t i = 0u; i < limit; i++)
        {
            const uint8_t module = g_config.profile.task_modules[i];
            if (module < 32u)
            {
                out->task_module_mask |= (uint32_t)1u << module;
            }
        }
    }

    sdlog_copy_cstr(out->target, (uint32_t)sizeof(out->target), robot_profile_name());
    sdlog_copy_cstr(out->board, (uint32_t)sizeof(out->board), robot_board_name());
    sdlog_copy_cstr(out->git_sha, (uint32_t)sizeof(out->git_sha), ARBATOS_GIT_SHA);
    sdlog_copy_cstr(out->build_date, (uint32_t)sizeof(out->build_date), ARBATOS_BUILD_DATE);
    sdlog_copy_cstr(out->build_time, (uint32_t)sizeof(out->build_time), ARBATOS_BUILD_TIME);
}

static void sdlog_remember_error(uint16_t reason, int32_t error_code)
{
    sdlog_prev_error_reason = reason;
    sdlog_prev_error_code = error_code;
    sdlog_prev_error_bytes = sdlog_bytes_flushed;
}

static int sdlog_append_record_bytes(uint8_t *dst,
                                     uint32_t dst_cap,
                                     uint32_t *inout_len,
                                     uint32_t dt_ms,
                                     uint16_t tag,
                                     const void *payload,
                                     uint16_t len)
{
    if (dst == NULL || inout_len == NULL || payload == NULL || len == 0u)
    {
        return -1;
    }

    uint8_t hdr[16];
    uint32_t hdr_len = 0u;
    hdr_len += (uint32_t)sdlog_write_var_u32(&hdr[hdr_len], dt_ms);
    hdr_len += (uint32_t)sdlog_write_var_u32(&hdr[hdr_len], (uint32_t)tag);
    hdr_len += (uint32_t)sdlog_write_var_u32(&hdr[hdr_len], (uint32_t)len);

    if ((*inout_len + hdr_len + (uint32_t)len) > dst_cap)
    {
        return -2;
    }

    memcpy(&dst[*inout_len], hdr, hdr_len);
    *inout_len += hdr_len;
    memcpy(&dst[*inout_len], payload, (uint32_t)len);
    *inout_len += (uint32_t)len;
    return 0;
}

static int sdlog_index_read(uint32_t *out_next)
{
    if (out_next == NULL)
    {
        return -1;
    }
    *out_next = 0u;

    FIL fp;
    FRESULT r = f_open(&fp, SDLOG_INDEX_FILE_PATH, FA_READ);
    if (r != FR_OK)
    {
        return (int)r;
    }

    sdlog_index_file_t rec;
    UINT br = 0u;
    r = f_read(&fp, &rec, (UINT)sizeof(rec), &br);
    (void)f_close(&fp);
    if (r != FR_OK || br != (UINT)sizeof(rec))
    {
        return -2;
    }
    if (rec.magic != SDLOG_INDEX_MAGIC)
    {
        return -3;
    }

    const uint32_t calc = sdlog_crc32_ieee((const uint8_t *)&rec, 8u);
    if (calc != rec.crc32)
    {
        return -4;
    }
    if (rec.next_idx >= 10000u)
    {
        return -5;
    }

    *out_next = rec.next_idx;
    return 0;
}

static void sdlog_index_write_best_effort(uint32_t next_idx)
{
    if (next_idx >= 10000u)
    {
        // Keep the index bounded to our filename format.
        next_idx = 0u;
    }

    sdlog_index_file_t rec = {0};
    rec.magic = SDLOG_INDEX_MAGIC;
    rec.next_idx = next_idx;
    rec.crc32 = sdlog_crc32_ieee((const uint8_t *)&rec, 8u);

    FIL fp;
    FRESULT r = f_open(&fp, SDLOG_INDEX_FILE_PATH, FA_OPEN_ALWAYS | FA_WRITE);
    if (r != FR_OK)
    {
        return;
    }

    (void)f_lseek(&fp, 0u);
    UINT bw = 0u;
    r = f_write(&fp, &rec, (UINT)sizeof(rec), &bw);
    if (r == FR_OK && bw == (UINT)sizeof(rec))
    {
        (void)f_sync(&fp);
    }
    (void)f_close(&fp);
}

#if SDLOG_ENABLE_COMPRESSION
static uint32_t sdlog_lz4_hash32(uint32_t v)
{
    return (v * 2654435761u) >> (32u - SDLOG_LZ4_HASH_BITS);
}

static int sdlog_lz4_compress_block(const uint8_t *src, uint32_t src_len, uint8_t *dst, uint32_t dst_cap, uint32_t *out_len)
{
    // 函数地图：扫描 4 字节匹配；先写 literal，再写 offset/match；末尾补最后一段 literal。
    if (out_len == NULL)
    {
        return -1;
    }
    *out_len = 0u;

    if (src == NULL || dst == NULL)
    {
        return -1;
    }

    memset(sdlog_lz4_hash, 0xFF, sizeof(sdlog_lz4_hash));

    uint32_t ip = 0u;
    uint32_t anchor = 0u;
    uint32_t op = 0u;

    if (src_len >= 4u)
    {
        const uint32_t mf_limit = src_len - 4u;

        while (ip <= mf_limit)
        {
            const uint32_t h = sdlog_lz4_hash32(sdlog_read_u32_le_unaligned(&src[ip]));
            const uint32_t ref = (uint32_t)sdlog_lz4_hash[h];
            sdlog_lz4_hash[h] = (uint16_t)ip;

            if (ref != 0xFFFFu && (ip - ref) <= 0xFFFFu && memcmp(&src[ref], &src[ip], 4u) == 0)
            {
                const uint32_t lit_len = ip - anchor;

                uint32_t match_len = 4u;
                while ((ip + match_len) < src_len && src[ref + match_len] == src[ip + match_len])
                {
                    match_len++;
                }

                const uint32_t ml = match_len - 4u;

                if (op >= dst_cap)
                {
                    return -2;
                }
                const uint32_t token_pos = op++;

                uint8_t token = 0u;
                token |= (uint8_t)((lit_len < 15u) ? (lit_len << 4u) : (15u << 4u));
                token |= (uint8_t)((ml < 15u) ? ml : 15u);
                dst[token_pos] = token;

                if (lit_len >= 15u)
                {
                    uint32_t left = lit_len - 15u;
                    while (left >= 255u)
                    {
                        if (op >= dst_cap)
                        {
                            return -2;
                        }
                        dst[op++] = 255u;
                        left -= 255u;
                    }
                    if (op >= dst_cap)
                    {
                        return -2;
                    }
                    dst[op++] = (uint8_t)left;
                }

                if ((op + lit_len) > dst_cap)
                {
                    return -2;
                }
                if (lit_len != 0u)
                {
                    memcpy(&dst[op], &src[anchor], lit_len);
                    op += lit_len;
                }

                const uint32_t offset = ip - ref;
                if ((op + 2u) > dst_cap)
                {
                    return -2;
                }
                dst[op++] = (uint8_t)offset;
                dst[op++] = (uint8_t)(offset >> 8u);

                if (ml >= 15u)
                {
                    uint32_t left = ml - 15u;
                    while (left >= 255u)
                    {
                        if (op >= dst_cap)
                        {
                            return -2;
                        }
                        dst[op++] = 255u;
                        left -= 255u;
                    }
                    if (op >= dst_cap)
                    {
                        return -2;
                    }
                    dst[op++] = (uint8_t)left;
                }

                ip += match_len;
                anchor = ip;
                continue;
            }

            ip++;
        }
    }

    // Last literals
    const uint32_t lit_len = src_len - anchor;
    if (op >= dst_cap)
    {
        return -2;
    }
    const uint32_t token_pos = op++;
    dst[token_pos] = (uint8_t)((lit_len < 15u) ? (lit_len << 4u) : (15u << 4u));

    if (lit_len >= 15u)
    {
        uint32_t left = lit_len - 15u;
        while (left >= 255u)
        {
            if (op >= dst_cap)
            {
                return -2;
            }
            dst[op++] = 255u;
            left -= 255u;
        }
        if (op >= dst_cap)
        {
            return -2;
        }
        dst[op++] = (uint8_t)left;
    }

    if ((op + lit_len) > dst_cap)
    {
        return -2;
    }
    if (lit_len != 0u)
    {
        memcpy(&dst[op], &src[anchor], lit_len);
        op += lit_len;
    }

    *out_len = op;
    return 0;
}
#endif

static int sdlog_write_block(const uint8_t *raw, uint32_t raw_len)
{
    if (raw == NULL || raw_len == 0u)
    {
        return 0;
    }

    const uint32_t crc32 = sdlog_crc32_ieee(raw, raw_len);

    uint32_t data_len = raw_len;
    int compressed = 0;
#if SDLOG_ENABLE_COMPRESSION
    const uint64_t compress_start_us = rt_profiler_begin();
    const int compress_result =
        sdlog_lz4_compress_block(raw, raw_len, sdlog_flush_out, (uint32_t)sizeof(sdlog_flush_out), &data_len);
    rt_profiler_end(RT_PROFILER_SDLOG_COMPRESS, compress_start_us);
    if (compress_result == 0 && data_len < raw_len)
    {
        compressed = 1;
    }
    else
    {
        data_len = raw_len;
    }
#endif

    sdlog_block_header_t bh = {0};
    bh.magic = SDLOG_BLOCK_MAGIC;
    bh.flags = (compressed ? SDLOG_BLOCK_FLAG_COMPRESSED : 0u) | SDLOG_BLOCK_FLAG_CRC32;
    bh.header_size = (uint16_t)sizeof(sdlog_block_header_t);
    bh.raw_len = raw_len;
    bh.data_len = data_len;
    bh.reserved = crc32;

    UINT bw = 0u;
    const uint64_t block_write_start_us = rt_profiler_begin();
    FRESULT r = f_write(&sdlog_fp, &bh, (UINT)sizeof(bh), &bw);
    if (r != FR_OK || bw != (UINT)sizeof(bh))
    {
        rt_profiler_end(RT_PROFILER_SDLOG_BLOCK_WRITE, block_write_start_us);
        sdlog_last_error = (r == FR_OK) ? -1 : (int32_t)r;
        sdlog_close_on_error();
        return -1;
    }
    sdlog_bytes_flushed += (uint32_t)bw;

    bw = 0u;
#if SDLOG_ENABLE_COMPRESSION
    const uint8_t *data = compressed ? sdlog_flush_out : raw;
#else
    const uint8_t *data = raw;
#endif
    r = f_write(&sdlog_fp, data, (UINT)data_len, &bw);
    if (r != FR_OK || bw != (UINT)data_len)
    {
        rt_profiler_end(RT_PROFILER_SDLOG_BLOCK_WRITE, block_write_start_us);
        sdlog_last_error = (r == FR_OK) ? -1 : (int32_t)r;
        sdlog_close_on_error();
        return -1;
    }
    rt_profiler_end(RT_PROFILER_SDLOG_BLOCK_WRITE, block_write_start_us);
    sdlog_bytes_flushed += (uint32_t)bw;
    return 0;
}

static uint8_t sdlog_ascii_lower(uint8_t c)
{
    if (c >= (uint8_t)'A' && c <= (uint8_t)'Z')
    {
        return (uint8_t)(c + (uint8_t)('a' - 'A'));
    }
    return c;
}

static FRESULT sdlog_sync_profiled(void)
{
    const uint64_t sync_start_us = rt_profiler_begin();
    const FRESULT r = f_sync(&sdlog_fp);
    rt_profiler_end(RT_PROFILER_SDLOG_SYNC, sync_start_us);
    return r;
}

static int sdlog_parse_log_index_from_name(const char *name, uint32_t *out_idx)
{
    if (name == NULL || out_idx == NULL)
    {
        return 0;
    }

    // Expected format: "sdlog_0000.bin" (case-insensitive)
    if (strlen(name) != 14u)
    {
        return 0;
    }

    const char prefix[] = "sdlog_";
    for (uint32_t i = 0u; i < 6u; i++)
    {
        if (sdlog_ascii_lower((uint8_t)name[i]) != (uint8_t)prefix[i])
        {
            return 0;
        }
    }

    if (name[10] != '.')
    {
        return 0;
    }
    if (sdlog_ascii_lower((uint8_t)name[11]) != (uint8_t)'b' ||
        sdlog_ascii_lower((uint8_t)name[12]) != (uint8_t)'i' ||
        sdlog_ascii_lower((uint8_t)name[13]) != (uint8_t)'n')
    {
        return 0;
    }

    uint32_t idx = 0u;
    for (uint32_t i = 6u; i < 10u; i++)
    {
        const char c = name[i];
        if (c < '0' || c > '9')
        {
            return 0;
        }
        idx = (idx * 10u) + (uint32_t)(c - '0');
    }

    *out_idx = idx;
    return 1;
}

static int sdlog_find_next_log_index(uint32_t *out_next)
{
    if (out_next == NULL)
    {
        return -1;
    }

    DIR dir;
    FILINFO fno;
    FRESULT r = f_opendir(&dir, "0:/");
    if (r != FR_OK)
    {
        return (int)r;
    }

    uint32_t max_idx = 0u;
    uint8_t found = 0u;

    for (;;)
    {
        r = f_readdir(&dir, &fno);
        if (r != FR_OK)
        {
            (void)f_closedir(&dir);
            return (int)r;
        }
        if (fno.fname[0] == '\0')
        {
            break;
        }
        if ((fno.fattrib & AM_DIR) != 0u)
        {
            continue;
        }

        uint32_t idx = 0u;
        if (sdlog_parse_log_index_from_name(fno.fname, &idx))
        {
            if (!found || idx > max_idx)
            {
                max_idx = idx;
                found = 1u;
            }
        }
    }

    (void)f_closedir(&dir);

    *out_next = found ? (max_idx + 1u) : 0u;
    return 0;
}

static int sdlog_open_next_file(void)
{
    // 函数地图：优先读序号索引；索引坏了再扫目录；创建新文件后先写头和启动 META。
    if (!sdcard_is_mounted())
    {
        return -1;
    }

    char path[32];
    uint32_t start = 0u;

    if (sdlog_index_read(&start) != 0)
    {
        // Fallback to the slow path only if the index file is missing/corrupt.
        if (sdlog_find_next_log_index(&start) != 0)
        {
            start = 0u;
        }
    }

    if (start >= 10000u)
    {
        start = 0u;
    }

    for (uint32_t i = start; i < 10000u; i++)
    {
        const int n = snprintf(path, sizeof(path), "0:/sdlog_%04lu.bin", (unsigned long)i);
        if (n <= 0 || (uint32_t)n >= sizeof(path))
        {
            continue;
        }

        const FRESULT r = f_open(&sdlog_fp, path, FA_WRITE | FA_CREATE_NEW);
        if (r == FR_OK)
        {
            sdlog_file_header_t hdr = {0};
            hdr.magic = SDLOG_FILE_MAGIC;
            hdr.header_size = (uint16_t)sizeof(sdlog_file_header_t);
            hdr.boot_tick_ms = bsp_time_get_tick_ms();

            UINT bw = 0u;
            const FRESULT wr0 = f_write(&sdlog_fp, &hdr, (UINT)sizeof(hdr), &bw);
            if (wr0 != FR_OK || bw != (UINT)sizeof(hdr))
            {
                sdlog_last_error = (wr0 == FR_OK) ? -1 : (int32_t)wr0;
                sdlog_remember_error(SDLOG_RESTART_REASON_STARTUP_HEADER, sdlog_last_error);
                (void)f_close(&sdlog_fp);
                sdcard_unmount();
                return -3;
            }

            // Emit a startup META record so the file is >16B even before the first sdlog_poll().
            // This helps distinguish "no flush task running" vs "no log records produced".
            typedef struct __attribute__((packed))
            {
                uint32_t boot_tick_ms;
                uint32_t heap_free;
                uint32_t heap_min_ever_free;
            } sdlog_meta_boot_t;

            const sdlog_meta_boot_t meta = {
                .boot_tick_ms = hdr.boot_tick_ms,
                .heap_free = (uint32_t)xPortGetFreeHeapSize(),
                .heap_min_ever_free = (uint32_t)xPortGetMinimumEverFreeHeapSize(),
            };

            sdlog_build_info_t build_info;
            sdlog_fill_build_info(&build_info);

            const uint16_t prev_error_reason = sdlog_prev_error_reason;
            const int32_t prev_error_code = sdlog_prev_error_code;
            const uint32_t prev_error_bytes = sdlog_prev_error_bytes;
            const sdlog_event_t restart_info = {
                .event_id = SDLOG_EVT_SDLOG_RESTART_INFO,
                .arg0_u16 = prev_error_reason,
                .arg1_u32 = (uint32_t)prev_error_code,
                .arg2_u32 = prev_error_bytes,
            };

            uint8_t raw[512u];
            uint32_t raw_len = 0u;
            int append_status = sdlog_append_record_bytes(raw,
                                                          (uint32_t)sizeof(raw),
                                                          &raw_len,
                                                          0u,
                                                          SDLOG_TAG_META,
                                                          &meta,
                                                          (uint16_t)sizeof(meta));

            if (append_status == 0)
            {
                append_status = sdlog_append_record_bytes(raw,
                                                          (uint32_t)sizeof(raw),
                                                          &raw_len,
                                                          0u,
                                                          SDLOG_TAG_BUILD_INFO,
                                                          &build_info,
                                                          (uint16_t)sizeof(build_info));
            }

            for (uint8_t i = 0u; append_status == 0 && i < robot_config_device_count(); i++)
            {
                robot_config_device_t device;
                sdlog_runtime_device_t runtime_device;

                if (robot_config_device_get(i, &device) == 0u)
                {
                    continue;
                }

                (void)memset(&runtime_device, 0, sizeof(runtime_device));
                runtime_device.version = SDLOG_RUNTIME_DEVICE_VERSION;
                runtime_device.kind = device.kind;
                runtime_device.group = device.group;
                runtime_device.group_index = device.group_index;
                runtime_device.source_id = device.source_id;
                if (device.kind == (uint8_t)ROBOT_CONFIG_DEVICE_KIND_MOTOR)
                {
                    const motor_instance_t *inst = motor_instance_find_by_actuator((actuator_id_e)device.source_id);

                    runtime_device.enabled = motor_instance_enabled(inst);
                    runtime_device.bus = motor_instance_bus(inst);
                }

                append_status = sdlog_append_record_bytes(raw,
                                                          (uint32_t)sizeof(raw),
                                                          &raw_len,
                                                          0u,
                                                          SDLOG_TAG_RUNTIME_DEVICE,
                                                          &runtime_device,
                                                          (uint16_t)sizeof(runtime_device));
            }

            if (append_status == 0 && prev_error_reason != 0u)
            {
                append_status = sdlog_append_record_bytes(raw,
                                                          (uint32_t)sizeof(raw),
                                                          &raw_len,
                                                          0u,
                                                          SDLOG_TAG_EVENT,
                                                          &restart_info,
                                                          (uint16_t)sizeof(restart_info));
            }

            if (append_status != 0)
            {
                sdlog_remember_error(SDLOG_RESTART_REASON_STARTUP_BLOCK, -4);
                (void)f_close(&sdlog_fp);
                return -4;
            }

            if (sdlog_write_block(raw, raw_len) != 0)
            {
                sdlog_remember_error(SDLOG_RESTART_REASON_STARTUP_BLOCK, sdlog_last_error);
                (void)f_close(&sdlog_fp);
                sdcard_unmount();
                return -4;
            }

            const FRESULT sync_r = sdlog_sync_profiled();
            if (sync_r != FR_OK)
            {
                sdlog_last_error = (int32_t)sync_r;
                sdlog_remember_error(SDLOG_RESTART_REASON_STARTUP_SYNC, sdlog_last_error);
                (void)f_close(&sdlog_fp);
                sdcard_unmount();
                return -6;
            }

            sdlog_last_sync_ms = bsp_time_get_tick_ms();
            sdlog_bytes_flushed = 0u;
            sdlog_last_error = 0;
            sdlog_prev_error_reason = 0u;
            sdlog_prev_error_code = 0;
            sdlog_prev_error_bytes = 0u;

            // Best-effort: persist the next index so next boot does not scan 0:/.
            sdlog_index_write_best_effort(i + 1u);

            taskENTER_CRITICAL();
            sdlog_head = 0u;
            sdlog_tail = 0u;
            sdlog_last_tick_ms = hdr.boot_tick_ms;
            sdlog_active = 1u;
            taskEXIT_CRITICAL();
            return 0;
        }
        if (r != FR_EXIST)
        {
            sdlog_last_error = (int32_t)r;
            sdlog_remember_error(SDLOG_RESTART_REASON_OPEN_FAIL, sdlog_last_error);
            sdcard_unmount();
            return (int)r;
        }
    }

    return -2;
}

int sdlog_is_active(void)
{
    return (sdlog_active != 0u) ? 1 : 0;
}

uint32_t sdlog_get_dropped(void)
{
    return sdlog_dropped;
}

int sdlog_start(void)
{
    if (sdlog_active)
    {
        return 0;
    }
    return sdlog_open_next_file();
}

void sdlog_stop(void)
{
    if (!sdlog_active)
    {
        return;
    }

    sdlog_remember_error(SDLOG_RESTART_REASON_STOP, 0);

    // Best-effort: stop accepting new records first, then close the file.
    taskENTER_CRITICAL();
    sdlog_active = 0u;
    sdlog_head = 0u;
    sdlog_tail = 0u;
    sdlog_last_tick_ms = 0u;
    taskEXIT_CRITICAL();

    (void)sdlog_sync_profiled();
    (void)f_close(&sdlog_fp);
}

void sdlog_write(uint16_t tag, const void *payload, uint16_t len)
{
    if (!sdlog_active || payload == NULL || len == 0u)
    {
        return;
    }

    const uint64_t write_start_us = rt_profiler_begin();
    uint8_t hdr[16];
    uint32_t hdr_len = 0u;
    uint32_t total = 0u;

    taskENTER_CRITICAL();
    const uint32_t head = sdlog_head;
    const uint32_t tail = sdlog_tail;

    const uint32_t now_ms = bsp_time_get_tick_ms();
    const uint32_t last_tick = sdlog_last_tick_ms;
    const uint32_t dt = now_ms - last_tick;

    hdr_len += (uint32_t)sdlog_write_var_u32(&hdr[hdr_len], dt);
    hdr_len += (uint32_t)sdlog_write_var_u32(&hdr[hdr_len], (uint32_t)tag);
    hdr_len += (uint32_t)sdlog_write_var_u32(&hdr[hdr_len], (uint32_t)len);
    total = hdr_len + (uint32_t)len;

    if (total >= SDLOG_BUF_SIZE)
    {
        taskEXIT_CRITICAL();
        rt_profiler_end(RT_PROFILER_SDLOG_WRITE, write_start_us);
        return;
    }

    if (sdlog_free_bytes(head, tail) < total)
    {
        sdlog_dropped++;
        taskEXIT_CRITICAL();
        rt_profiler_end(RT_PROFILER_SDLOG_WRITE, write_start_us);
        return;
    }

    sdlog_last_tick_ms = now_ms;
    sdlog_ring_write_bytes_locked(hdr, hdr_len);
    sdlog_ring_write_bytes_locked((const uint8_t *)payload, (uint32_t)len);
    taskEXIT_CRITICAL();
    rt_profiler_end(RT_PROFILER_SDLOG_WRITE, write_start_us);
}

void sdlog_write_isr(uint16_t tag, const void *payload, uint16_t len)
{
    if (!sdlog_active || payload == NULL || len == 0u)
    {
        return;
    }

    uint8_t hdr[16];
    uint32_t hdr_len = 0u;
    uint32_t total = 0u;

    UBaseType_t uxSavedInterruptStatus;
    uxSavedInterruptStatus = taskENTER_CRITICAL_FROM_ISR();
    const uint32_t head = sdlog_head;
    const uint32_t tail = sdlog_tail;

    const uint32_t now_ms = bsp_time_get_tick_ms();
    const uint32_t last_tick = sdlog_last_tick_ms;
    const uint32_t dt = now_ms - last_tick;

    hdr_len += (uint32_t)sdlog_write_var_u32(&hdr[hdr_len], dt);
    hdr_len += (uint32_t)sdlog_write_var_u32(&hdr[hdr_len], (uint32_t)tag);
    hdr_len += (uint32_t)sdlog_write_var_u32(&hdr[hdr_len], (uint32_t)len);
    total = hdr_len + (uint32_t)len;

    if (total >= SDLOG_BUF_SIZE)
    {
        taskEXIT_CRITICAL_FROM_ISR(uxSavedInterruptStatus);
        return;
    }

    if (sdlog_free_bytes(head, tail) < total)
    {
        sdlog_dropped++;
        taskEXIT_CRITICAL_FROM_ISR(uxSavedInterruptStatus);
        return;
    }

    sdlog_last_tick_ms = now_ms;
    sdlog_ring_write_bytes_locked(hdr, hdr_len);
    sdlog_ring_write_bytes_locked((const uint8_t *)payload, (uint32_t)len);
    taskEXIT_CRITICAL_FROM_ISR(uxSavedInterruptStatus);
}

static void sdlog_close_on_error(void)
{
    if (!sdlog_active)
    {
        return;
    }

    sdlog_remember_error(SDLOG_RESTART_REASON_RUNTIME_IO, sdlog_last_error);

    taskENTER_CRITICAL();
    sdlog_active = 0u;
    sdlog_head = 0u;
    sdlog_tail = 0u;
    sdlog_last_tick_ms = 0u;
    taskEXIT_CRITICAL();

    (void)sdlog_sync_profiled();
    (void)f_close(&sdlog_fp);
    sdcard_unmount();
}

void sdlog_poll(void)
{
    if (!sdlog_active)
    {
        return;
    }

    // Flush a bounded number of blocks per poll.
    // When log production rate is high, multiple blocks/poll are needed to avoid ring overflow.
    for (uint32_t i = 0u; i < SDLOG_FLUSH_BLOCKS_PER_POLL; i++)
    {
        uint32_t head;
        uint32_t tail;
        uint32_t used;
        uint32_t chunk;
        uint32_t first;
        uint32_t second;

        taskENTER_CRITICAL();
        head = sdlog_head;
        tail = sdlog_tail;
        used = sdlog_used_bytes(head, tail);
        if (used == 0u)
        {
            taskEXIT_CRITICAL();
            break;
        }

        chunk = used;
        if (chunk > SDLOG_FLUSH_CHUNK_MAX)
        {
            chunk = SDLOG_FLUSH_CHUNK_MAX;
        }

        const uint32_t to_end = SDLOG_BUF_SIZE - tail;
        first = (chunk <= to_end) ? chunk : to_end;
        second = chunk - first;
        taskEXIT_CRITICAL();

        memcpy(&sdlog_flush_in[0], &sdlog_buf[tail], first);
        if (second != 0u)
        {
            memcpy(&sdlog_flush_in[first], &sdlog_buf[0], second);
        }

        if (sdlog_write_block(sdlog_flush_in, chunk) != 0)
        {
            return;
        }

        taskENTER_CRITICAL();
        sdlog_tail = (sdlog_tail + chunk) % SDLOG_BUF_SIZE;
        taskEXIT_CRITICAL();
    }

    const uint32_t now_ms = bsp_time_get_tick_ms();
    if ((uint32_t)(now_ms - sdlog_last_sync_ms) >= SDLOG_SYNC_PERIOD_MS)
    {
        const FRESULT r = sdlog_sync_profiled();
        if (r != FR_OK)
        {
            sdlog_last_error = (int32_t)r;
            sdlog_close_on_error();
            return;
        }
        sdlog_last_sync_ms = now_ms;
    }
}

void sdlog_get_stats(sdlog_stats_t *out)
{
    if (out == NULL)
    {
        return;
    }

    memset(out, 0, sizeof(*out));

    taskENTER_CRITICAL();
    const uint32_t head = sdlog_head;
    const uint32_t tail = sdlog_tail;
    out->active = (sdlog_active != 0u) ? 1u : 0u;
    out->dropped = sdlog_dropped;
    out->ring_used = sdlog_used_bytes(head, tail);
    out->ring_free = sdlog_free_bytes(head, tail);
    out->bytes_flushed = sdlog_bytes_flushed;
    out->last_sync_ms = sdlog_last_sync_ms;
    out->last_error = sdlog_last_error;
    taskEXIT_CRITICAL();
}
