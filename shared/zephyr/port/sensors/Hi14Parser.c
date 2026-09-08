/* SPDX-License-Identifier: Apache-2.0 */
/* HI14 的有界串口流解析、接收缓冲和可信样本筛选。 */
#include "Hi14Parser.h"

#include <float.h>
#include <math.h>
#include <string.h>

#define HI14_SOF0 0x5Au
#define HI14_SOF1 0xA5u
#define HI14_HI91_TAG 0x91u
#define HI14_QUAT_NORM_SQ_MIN 0.5f
#define HI14_QUAT_NORM_SQ_MAX 1.5f
#define HI14_ACCEL_ABS_MAX_G 16.5f
#define HI14_GYRO_ABS_MAX_DPS 2000.5f
#define HI14_MAG_ABS_MAX_UT 1000.5f
#define HI14_PRESSURE_MIN_PA 1000.0f
#define HI14_PRESSURE_MAX_PA 200000.0f

typedef char Hi14FloatMustBeBinary32[
    (sizeof(float) == 4u && FLT_RADIX == 2 && FLT_MANT_DIG == 24) ? 1 : -1];

enum
{
    HI14_WAIT_SOF0 = 0,
    HI14_WAIT_SOF1,
    HI14_READ_LEN0,
    HI14_READ_LEN1,
    HI14_READ_CRC0,
    HI14_READ_CRC1,
    HI14_READ_PAYLOAD,
};

static uint16_t Hi14ReadU16(const uint8_t *data)
{
    return (uint16_t)data[0] | ((uint16_t)data[1] << 8u);
}

static uint32_t Hi14ReadU32(const uint8_t *data)
{
    return (uint32_t)data[0] |
           ((uint32_t)data[1] << 8u) |
           ((uint32_t)data[2] << 16u) |
           ((uint32_t)data[3] << 24u);
}

static float Hi14ReadFloat(const uint8_t *data)
{
    const uint32_t bits = Hi14ReadU32(data);
    float value;
    memcpy(&value, &bits, sizeof(value));
    return value;
}

static void Hi14RestartFromByte(Hi14Parser *parser, uint8_t byte)
{
    parser->payload_size = 0u;
    parser->payload_pos = 0u;
    parser->expected_crc = 0u;
    parser->computed_crc = 0u;
    parser->state = (byte == HI14_SOF0) ? HI14_WAIT_SOF1 : HI14_WAIT_SOF0;
    if (byte == HI14_SOF0) {
        parser->computed_crc = Hi14Crc16Update(0u, &byte, 1u);
    }
}

static bool Hi14DecodeHi91(const uint8_t payload[HI14_HI91_PAYLOAD_SIZE],
                           Hi14Sample *sample)
{
    if (payload[0] != HI14_HI91_TAG) {
        return false;
    }

    Hi14Sample decoded = {0};
    decoded.status = Hi14ReadU16(&payload[1]);
    decoded.temperature_c = (int8_t)payload[3];
    decoded.pressure_pa = Hi14ReadFloat(&payload[4]);
    decoded.sensor_time_ms = Hi14ReadU32(&payload[8]);
    for (size_t i = 0u; i < 3u; ++i) {
        decoded.accel_g[i] = Hi14ReadFloat(&payload[12u + i * 4u]);
        decoded.gyro_dps[i] = Hi14ReadFloat(&payload[24u + i * 4u]);
        decoded.mag_ut[i] = Hi14ReadFloat(&payload[36u + i * 4u]);
        decoded.rpy_deg[i] = Hi14ReadFloat(&payload[48u + i * 4u]);
    }
    for (size_t i = 0u; i < 4u; ++i) {
        decoded.quat[i] = Hi14ReadFloat(&payload[60u + i * 4u]);
    }

    if (!isfinite(decoded.pressure_pa)) {
        return false;
    }
    for (size_t i = 0u; i < 3u; ++i) {
        if (!isfinite(decoded.accel_g[i]) ||
            !isfinite(decoded.gyro_dps[i]) ||
            !isfinite(decoded.mag_ut[i]) ||
            !isfinite(decoded.rpy_deg[i])) {
            return false;
        }
    }

    float norm_sq = 0.0f;
    for (size_t i = 0u; i < 4u; ++i) {
        if (!isfinite(decoded.quat[i])) {
            return false;
        }
        norm_sq += decoded.quat[i] * decoded.quat[i];
    }
    if (!isfinite(norm_sq) ||
        norm_sq < HI14_QUAT_NORM_SQ_MIN ||
        norm_sq > HI14_QUAT_NORM_SQ_MAX) {
        return false;
    }

    const float inv_norm = 1.0f / sqrtf(norm_sq);
    for (size_t i = 0u; i < 4u; ++i) {
        decoded.quat[i] *= inv_norm;
    }
    if (sample != NULL) {
        *sample = decoded;
    }
    return true;
}

void Hi14ParserInit(Hi14Parser *parser)
{
    if (parser != NULL) {
        memset(parser, 0, sizeof(*parser));
    }
}

uint16_t Hi14Crc16Update(uint16_t crc, const uint8_t *data, size_t size)
{
    if (data == NULL) {
        return crc;
    }

    for (size_t i = 0u; i < size; ++i) {
        crc ^= (uint16_t)data[i] << 8u;
        for (uint8_t bit = 0u; bit < 8u; ++bit) {
            crc = (crc & 0x8000u) != 0u
                      ? (uint16_t)((crc << 1u) ^ 0x1021u)
                      : (uint16_t)(crc << 1u);
        }
    }
    return crc;
}

bool Hi14ParserFeedByte(Hi14Parser *parser, uint8_t byte, Hi14Sample *sample)
{
    if (parser == NULL) {
        return false;
    }

    switch (parser->state) {
        case HI14_WAIT_SOF0:
            if (byte == HI14_SOF0) {
                parser->computed_crc = Hi14Crc16Update(0u, &byte, 1u);
                parser->state = HI14_WAIT_SOF1;
            }
            break;

        case HI14_WAIT_SOF1:
            if (byte == HI14_SOF1) {
                parser->computed_crc = Hi14Crc16Update(parser->computed_crc, &byte, 1u);
                parser->state = HI14_READ_LEN0;
            }
            else {
                Hi14RestartFromByte(parser, byte);
            }
            break;

        case HI14_READ_LEN0:
            parser->payload_size = byte;
            parser->computed_crc = Hi14Crc16Update(parser->computed_crc, &byte, 1u);
            parser->state = HI14_READ_LEN1;
            break;

        case HI14_READ_LEN1:
            parser->payload_size |= (uint16_t)byte << 8u;
            parser->computed_crc = Hi14Crc16Update(parser->computed_crc, &byte, 1u);
            if (parser->payload_size == 0u || parser->payload_size > HI14_MAX_PAYLOAD_SIZE) {
                parser->length_errors++;
                Hi14RestartFromByte(parser, byte);
            }
            else {
                parser->state = HI14_READ_CRC0;
            }
            break;

        case HI14_READ_CRC0:
            parser->expected_crc = byte;
            parser->state = HI14_READ_CRC1;
            break;

        case HI14_READ_CRC1:
            parser->expected_crc |= (uint16_t)byte << 8u;
            parser->payload_pos = 0u;
            parser->state = HI14_READ_PAYLOAD;
            break;

        case HI14_READ_PAYLOAD:
            parser->payload[parser->payload_pos++] = byte;
            parser->computed_crc = Hi14Crc16Update(parser->computed_crc, &byte, 1u);
            if (parser->payload_pos == parser->payload_size) {
                bool accepted = false;
                if (parser->computed_crc != parser->expected_crc) {
                    parser->crc_errors++;
                }
                else if (parser->payload_size != HI14_HI91_PAYLOAD_SIZE ||
                         parser->payload[0] != HI14_HI91_TAG) {
                    parser->unsupported_frames++;
                }
                else if (!Hi14DecodeHi91(parser->payload, sample)) {
                    parser->data_errors++;
                }
                else {
                    parser->valid_frames++;
                    accepted = true;
                }
                Hi14RestartFromByte(parser, byte);
                return accepted;
            }
            break;

        default:
            Hi14ParserInit(parser);
            break;
    }
    return false;
}

size_t Hi14ParserFeed(Hi14Parser *parser,
                      const uint8_t *data,
                      size_t size,
                      Hi14Sample *last_sample)
{
    if (parser == NULL || (data == NULL && size != 0u)) {
        return 0u;
    }

    size_t accepted = 0u;
    Hi14Sample decoded;
    for (size_t i = 0u; i < size; ++i) {
        if (Hi14ParserFeedByte(parser, data[i], &decoded)) {
            accepted++;
            if (last_sample != NULL) {
                *last_sample = decoded;
            }
        }
    }
    return accepted;
}

void Hi14ByteRingInit(Hi14ByteRing *ring)
{
    if (ring != NULL) {
        memset(ring, 0, sizeof(*ring));
    }
}

void Hi14ByteRingDiscard(Hi14ByteRing *ring)
{
    if (ring == NULL) {
        return;
    }
    ring->head = 0u;
    ring->tail = 0u;
    ring->count = 0u;
    ring->overflow_count++;
    ring->epoch++;
}

bool Hi14ByteRingPush(Hi14ByteRing *ring,
                      const uint8_t *data,
                      size_t size,
                      uint32_t receive_tick_ms)
{
    if (ring == NULL || (data == NULL && size != 0u)) {
        return false;
    }
    if (size > HI14_BYTE_RING_CAPACITY) {
        Hi14ByteRingDiscard(ring);
        return false;
    }

    bool uninterrupted = true;
    if (size > (size_t)(HI14_BYTE_RING_CAPACITY - ring->count)) {
        /* 丢掉全部旧字节，调用者看到 epoch 变化后必须同步清解析器。 */
        Hi14ByteRingDiscard(ring);
        uninterrupted = false;
    }
    for (size_t i = 0u; i < size; ++i) {
        ring->bytes[ring->head] = data[i];
        ring->receive_ticks_ms[ring->head] = receive_tick_ms;
        ring->head = (uint16_t)((ring->head + 1u) % HI14_BYTE_RING_CAPACITY);
        ring->count++;
    }
    return uninterrupted;
}

size_t Hi14ByteRingPop(Hi14ByteRing *ring,
                       uint8_t *data,
                       uint32_t *receive_ticks_ms,
                       size_t capacity,
                       uint32_t *epoch)
{
    if (ring == NULL || data == NULL || receive_ticks_ms == NULL || capacity == 0u) {
        return 0u;
    }

    size_t count = ring->count;
    if (count > capacity) {
        count = capacity;
    }
    if (epoch != NULL) {
        *epoch = ring->epoch;
    }
    for (size_t i = 0u; i < count; ++i) {
        data[i] = ring->bytes[ring->tail];
        receive_ticks_ms[i] = ring->receive_ticks_ms[ring->tail];
        ring->tail = (uint16_t)((ring->tail + 1u) % HI14_BYTE_RING_CAPACITY);
        ring->count--;
    }
    return count;
}

void Hi14SampleGateInit(Hi14SampleGate *gate)
{
    if (gate != NULL) {
        memset(gate, 0, sizeof(*gate));
    }
}

static bool Hi14SampleRangesValid(const Hi14Sample *sample)
{
    if (sample == NULL ||
        !isfinite(sample->pressure_pa) ||
        sample->pressure_pa < HI14_PRESSURE_MIN_PA ||
        sample->pressure_pa > HI14_PRESSURE_MAX_PA) {
        return false;
    }
    for (size_t i = 0u; i < 3u; ++i) {
        if (!isfinite(sample->accel_g[i]) || fabsf(sample->accel_g[i]) > HI14_ACCEL_ABS_MAX_G ||
            !isfinite(sample->gyro_dps[i]) || fabsf(sample->gyro_dps[i]) > HI14_GYRO_ABS_MAX_DPS ||
            !isfinite(sample->mag_ut[i]) || fabsf(sample->mag_ut[i]) > HI14_MAG_ABS_MAX_UT ||
            !isfinite(sample->rpy_deg[i])) {
            return false;
        }
    }
    if (fabsf(sample->rpy_deg[0]) > 180.5f ||
        fabsf(sample->rpy_deg[1]) > 90.5f ||
        fabsf(sample->rpy_deg[2]) > 180.5f) {
        return false;
    }
    return true;
}

bool Hi14SampleGateAccept(Hi14SampleGate *gate, const Hi14Sample *sample)
{
    if (gate == NULL || sample == NULL ||
        (sample->status & HI14_UNTRUSTED_STATUS_MASK) != 0u ||
        !Hi14SampleRangesValid(sample)) {
        return false;
    }

    if (gate->initialized == 0u) {
        gate->last_sensor_time_ms = sample->sensor_time_ms;
        gate->initialized = 1u;
        gate->restart_pending = 0u;
        return true;
    }

    const uint32_t delta = sample->sensor_time_ms - gate->last_sensor_time_ms;
    if (delta != 0u && delta < 0x80000000u) {
        gate->last_sensor_time_ms = sample->sensor_time_ms;
        gate->restart_pending = 0u;
        return true;
    }
    if (delta == 0u) {
        return false;
    }

    /* 时间倒退视为可能重启；必须再看到一次递增，才建立新的时间基准。 */
    if (gate->restart_pending != 0u) {
        const uint32_t restart_delta = sample->sensor_time_ms - gate->restart_candidate_ms;
        if (restart_delta != 0u && restart_delta < 0x80000000u) {
            gate->last_sensor_time_ms = sample->sensor_time_ms;
            gate->restart_pending = 0u;
            return true;
        }
    }
    gate->restart_candidate_ms = sample->sensor_time_ms;
    gate->restart_pending = 1u;
    return false;
}
