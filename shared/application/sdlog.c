/*
 * SPDX-FileCopyrightText: 2026 陈轩 <2811158416@qq.com>
 * SPDX-License-Identifier: PolyForm-Noncommercial-1.0.0
 * Required Notice: Copyright 2026 陈轩 <2811158416@qq.com>
 *
 * First published in this repository: 2026-04-06
 * Use of this file is governed by the LICENSE file in the repository root.
 */

#include "sdlog.h"

#include <stdio.h>
#include <string.h>

#include "cmsis_os2.h"
#include "bsp_time.h"
#include "sdcard.h"
#include "rt_profiler.h"

#include "fatfs/ff.h"

#include "FreeRTOS.h"
#include "task.h"

#ifndef SDLOG_BUF_SIZE
#define SDLOG_BUF_SIZE (16u * 1024u)
#endif

#ifndef SDLOG_FLUSH_CHUNK_MAX
#define SDLOG_FLUSH_CHUNK_MAX (2u * 1024u)
#endif

#ifndef SDLOG_SYNC_PERIOD_MS
#define SDLOG_SYNC_PERIOD_MS 1000u
#endif

#ifndef SDLOG_FLUSH_BLOCKS_PER_POLL
#define SDLOG_FLUSH_BLOCKS_PER_POLL 4u
#endif

#define SDLOG_ALIGN 4u

static FIL sdlog_fp;
static volatile uint8_t sdlog_active = 0u;
static volatile uint32_t sdlog_dropped = 0u;
static uint32_t sdlog_last_sync_ms = 0u;
static volatile uint32_t sdlog_bytes_flushed = 0u;
static volatile int32_t sdlog_last_error = 0;
static uint32_t sdlog_last_tick_ms = 0u;

__attribute__((section(".ccmram"))) static uint8_t sdlog_buf[SDLOG_BUF_SIZE];
static volatile uint32_t sdlog_head = 0u;
static volatile uint32_t sdlog_tail = 0u;

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

// v2: block compression (LZ4 block format, implemented locally for small blocks).
#define SDLOG_LZ4_HASH_BITS 11u
#define SDLOG_LZ4_HASH_SIZE (1u << SDLOG_LZ4_HASH_BITS)
#define SDLOG_LZ4_MAX_OUTPUT(n) ((n) + ((n) / 255u) + 16u)

static uint8_t sdlog_flush_in[SDLOG_FLUSH_CHUNK_MAX];
static uint8_t sdlog_flush_out[SDLOG_LZ4_MAX_OUTPUT(SDLOG_FLUSH_CHUNK_MAX)];
__attribute__((section(".ccmram"))) static uint16_t sdlog_lz4_hash[SDLOG_LZ4_HASH_SIZE];

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

static uint32_t sdlog_read_u32_le_unaligned(const uint8_t *p)
{
    uint32_t v;
    memcpy(&v, p, sizeof(v));
    return v;
}

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

static uint32_t sdlog_lz4_hash32(uint32_t v)
{
    return (v * 2654435761u) >> (32u - SDLOG_LZ4_HASH_BITS);
}

static int sdlog_lz4_compress_block(const uint8_t *src, uint32_t src_len, uint8_t *dst, uint32_t dst_cap, uint32_t *out_len)
{
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

static int sdlog_write_v2_block(const uint8_t *raw, uint32_t raw_len)
{
    if (raw == NULL || raw_len == 0u)
    {
        return 0;
    }

    const uint32_t crc32 = sdlog_crc32_ieee(raw, raw_len);

    uint32_t data_len = 0u;
    int compressed = 0;
    if (sdlog_lz4_compress_block(raw, raw_len, sdlog_flush_out, (uint32_t)sizeof(sdlog_flush_out), &data_len) == 0 &&
        data_len < raw_len)
    {
        compressed = 1;
    }
    else
    {
        data_len = raw_len;
    }

    sdlog_block_header_t bh = {0};
    bh.magic = SDLOG_BLOCK_MAGIC;
    bh.flags = (compressed ? SDLOG_BLOCK_FLAG_COMPRESSED : 0u) | SDLOG_BLOCK_FLAG_CRC32;
    bh.header_size = (uint16_t)sizeof(sdlog_block_header_t);
    bh.raw_len = raw_len;
    bh.data_len = data_len;
    bh.reserved = crc32;

    UINT bw = 0u;
    FRESULT r = f_write(&sdlog_fp, &bh, (UINT)sizeof(bh), &bw);
    if (r != FR_OK || bw != (UINT)sizeof(bh))
    {
        sdlog_last_error = (r == FR_OK) ? -1 : (int32_t)r;
        sdlog_close_on_error();
        return -1;
    }
    sdlog_bytes_flushed += (uint32_t)bw;

    bw = 0u;
    const uint8_t *data = compressed ? sdlog_flush_out : raw;
    r = f_write(&sdlog_fp, data, (UINT)data_len, &bw);
    if (r != FR_OK || bw != (UINT)data_len)
    {
        sdlog_last_error = (r == FR_OK) ? -1 : (int32_t)r;
        sdlog_close_on_error();
        return -1;
    }
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
            hdr.version = SDLOG_FILE_VERSION;
            hdr.header_size = (uint16_t)sizeof(sdlog_file_header_t);
            hdr.boot_tick_ms = bsp_time_get_tick_ms();

            UINT bw = 0u;
            const FRESULT wr0 = f_write(&sdlog_fp, &hdr, (UINT)sizeof(hdr), &bw);
            if (wr0 != FR_OK || bw != (UINT)sizeof(hdr))
            {
                sdlog_last_error = (wr0 == FR_OK) ? -1 : (int32_t)wr0;
                (void)f_close(&sdlog_fp);
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

#if SDLOG_FILE_VERSION == 2u
            sdlog_record_header_t rh = {0};
            rh.tick_ms = hdr.boot_tick_ms;
            rh.tag = SDLOG_TAG_META;
            rh.len = (uint16_t)sizeof(sdlog_meta_boot_t);

            uint8_t raw[sizeof(sdlog_record_header_t) + sizeof(sdlog_meta_boot_t)];
            memcpy(&raw[0], &rh, sizeof(rh));
            memcpy(&raw[sizeof(rh)], &meta, sizeof(meta));
            const uint32_t raw_len = (uint32_t)sizeof(raw);
#elif SDLOG_FILE_VERSION == 3u
            uint8_t raw[1u + 3u + 3u + sizeof(sdlog_meta_boot_t)];
            uint32_t raw_len = 0u;
            raw_len += (uint32_t)sdlog_write_var_u32(&raw[raw_len], 0u); // dt_ms
            raw_len += (uint32_t)sdlog_write_var_u32(&raw[raw_len], (uint32_t)SDLOG_TAG_META);
            raw_len += (uint32_t)sdlog_write_var_u32(&raw[raw_len], (uint32_t)sizeof(sdlog_meta_boot_t));
            memcpy(&raw[raw_len], &meta, sizeof(meta));
            raw_len += (uint32_t)sizeof(meta);
#else
#error "Unsupported SDLOG_FILE_VERSION"
#endif

            if (sdlog_write_v2_block(raw, raw_len) != 0)
            {
                (void)f_close(&sdlog_fp);
                return -4;
            }

            const FRESULT sync_r = f_sync(&sdlog_fp);
            if (sync_r != FR_OK)
            {
                sdlog_last_error = (int32_t)sync_r;
                (void)f_close(&sdlog_fp);
                return -6;
            }

            sdlog_last_sync_ms = bsp_time_get_tick_ms();
            sdlog_bytes_flushed = 0u;
            sdlog_last_error = 0;

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

    // Best-effort: stop accepting new records first, then close the file.
    taskENTER_CRITICAL();
    sdlog_active = 0u;
    sdlog_head = 0u;
    sdlog_tail = 0u;
    sdlog_last_tick_ms = 0u;
    taskEXIT_CRITICAL();

    (void)f_sync(&sdlog_fp);
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

    taskENTER_CRITICAL();
    sdlog_active = 0u;
    sdlog_head = 0u;
    sdlog_tail = 0u;
    sdlog_last_tick_ms = 0u;
    taskEXIT_CRITICAL();

    (void)f_sync(&sdlog_fp);
    (void)f_close(&sdlog_fp);
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

        if (sdlog_write_v2_block(sdlog_flush_in, chunk) != 0)
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
        const FRESULT r = f_sync(&sdlog_fp);
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
