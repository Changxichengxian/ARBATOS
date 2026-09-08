/* SPDX-License-Identifier: Apache-2.0 */
#ifndef HI14_PARSER_H
#define HI14_PARSER_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define HI14_MAX_PAYLOAD_SIZE 512u
#define HI14_HI91_PAYLOAD_SIZE 76u
#define HI14_BYTE_RING_CAPACITY 512u
/* WB_CONV | ACC_SAT | GYR_SAT | ATT_CONV：任一置位时姿态不可发布。 */
#define HI14_UNTRUSTED_STATUS_MASK 0x00E8u

typedef struct
{
    uint16_t status;
    int8_t temperature_c;
    float pressure_pa;
    uint32_t sensor_time_ms;
    float accel_g[3];
    float gyro_dps[3];
    float mag_ut[3];
    float rpy_deg[3];
    float quat[4];
} Hi14Sample;

typedef struct
{
    uint8_t state;
    uint16_t payload_size;
    uint16_t payload_pos;
    uint16_t expected_crc;
    uint16_t computed_crc;
    uint8_t payload[HI14_MAX_PAYLOAD_SIZE];
    uint32_t valid_frames;
    uint32_t crc_errors;
    uint32_t length_errors;
    uint32_t data_errors;
    uint32_t unsupported_frames;
} Hi14Parser;

typedef struct
{
    uint8_t bytes[HI14_BYTE_RING_CAPACITY];
    uint32_t receive_ticks_ms[HI14_BYTE_RING_CAPACITY];
    uint16_t head;
    uint16_t tail;
    uint16_t count;
    uint32_t overflow_count;
    uint32_t epoch;
} Hi14ByteRing;

typedef struct
{
    uint32_t last_sensor_time_ms;
    uint32_t restart_candidate_ms;
    uint8_t initialized;
    uint8_t restart_pending;
} Hi14SampleGate;

void Hi14ParserInit(Hi14Parser *parser);
/* CRC-16/XMODEM：初值 0，多项式 0x1021，不含帧内 CRC 两字节。 */
uint16_t Hi14Crc16Update(uint16_t crc, const uint8_t *data, size_t size);
/* 逐字节推进 5A A5 + LEN + CRC + payload 状态机，仅返回完整有效 HI91。 */
bool Hi14ParserFeedByte(Hi14Parser *parser, uint8_t byte, Hi14Sample *sample);
size_t Hi14ParserFeed(Hi14Parser *parser,
                      const uint8_t *data,
                      size_t size,
                      Hi14Sample *last_sample);
void Hi14ByteRingInit(Hi14ByteRing *ring);
/* 主动标记接收缺口；消费端必须在 epoch 变化后清解析状态。 */
void Hi14ByteRingDiscard(Hi14ByteRing *ring);
/* 接收侧只复制字节与本地接收时刻；空间不足时先丢弃全部旧字节。 */
bool Hi14ByteRingPush(Hi14ByteRing *ring,
                      const uint8_t *data,
                      size_t size,
                      uint32_t receive_tick_ms);
size_t Hi14ByteRingPop(Hi14ByteRing *ring,
                       uint8_t *data,
                       uint32_t *receive_ticks_ms,
                       size_t capacity,
                       uint32_t *epoch);
void Hi14SampleGateInit(Hi14SampleGate *gate);
/* 发布门槛：状态可信、量程合理、设备时间递增；支持正常 uint32 回卷。 */
bool Hi14SampleGateAccept(Hi14SampleGate *gate, const Hi14Sample *sample);

#endif
