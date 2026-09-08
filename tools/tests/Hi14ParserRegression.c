/* SPDX-License-Identifier: Apache-2.0 */
/* HI14 纯 C 回归：覆盖官方 HI91 样例及串口流重同步边界。 */

#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "Hi14Parser.h"

static const uint8_t GoldenFrame[] = {
    0x5A, 0xA5, 0x4C, 0x00, 0x14, 0xBB, 0x91, 0x08, 0x15, 0x23,
    0x09, 0xA2, 0xC4, 0x47, 0x08, 0x15, 0x1C, 0x00, 0xCC, 0xE8,
    0x61, 0xBE, 0x9A, 0x35, 0x56, 0x3E, 0x65, 0xEA, 0x72, 0x3F,
    0x31, 0xD0, 0x7C, 0xBD, 0x75, 0xDD, 0xC5, 0xBB, 0x6B, 0xD7,
    0x24, 0xBC, 0x89, 0x88, 0xFC, 0x40, 0x01, 0x00, 0x6A, 0x41,
    0xAB, 0x2A, 0x70, 0xC2, 0x96, 0xD4, 0x50, 0x41, 0xED, 0x03,
    0x43, 0x41, 0x41, 0xF4, 0xF4, 0xC2, 0xCC, 0xCA, 0xF8, 0xBE,
    0x73, 0x6A, 0x19, 0xBE, 0xF0, 0x00, 0x1C, 0x3D, 0x8D, 0x37,
    0x5C, 0x3F,
};

static int Check(int condition, const char *message)
{
    if (!condition) {
        (void)fprintf(stderr, "FAIL: %s\n", message);
        return 0;
    }
    return 1;
}

static int Near(float actual, float expected, float tolerance)
{
    return fabsf(actual - expected) <= tolerance;
}

static void FrameUpdateCrc(uint8_t *frame)
{
    const uint16_t payload_size = (uint16_t)frame[2] | ((uint16_t)frame[3] << 8u);
    uint16_t crc = Hi14Crc16Update(0u, frame, 4u);
    crc = Hi14Crc16Update(crc, &frame[6], payload_size);
    frame[4] = (uint8_t)crc;
    frame[5] = (uint8_t)(crc >> 8u);
}

static int TestGoldenFrame(void)
{
    Hi14Parser parser;
    Hi14Sample sample;
    Hi14ParserInit(&parser);

    uint16_t crc = Hi14Crc16Update(0u, GoldenFrame, 4u);
    crc = Hi14Crc16Update(crc, &GoldenFrame[6], HI14_HI91_PAYLOAD_SIZE);
    int ok = Check(crc == 0xBB14u, "official CRC must be 0xBB14");
    ok &= Check(Hi14ParserFeed(&parser, GoldenFrame, sizeof(GoldenFrame), &sample) == 1u,
                "official HI91 frame must decode");
    ok &= Check(sample.status == 0x1508u, "status must decode little-endian");
    ok &= Check(sample.temperature_c == 35, "temperature must decode as int8");
    ok &= Check(sample.sensor_time_ms == 1840392u, "sensor time must match manual");
    ok &= Check(Near(sample.pressure_pa, 100676.0f, 0.5f), "pressure must match manual");
    ok &= Check(Near(sample.accel_g[0], -0.2206f, 0.0002f) &&
                    Near(sample.accel_g[1], 0.2092f, 0.0002f) &&
                    Near(sample.accel_g[2], 0.9489f, 0.0002f),
                "acceleration must match manual");
    ok &= Check(Near(sample.gyro_dps[0], -0.0617f, 0.0002f) &&
                    Near(sample.gyro_dps[1], -0.0060f, 0.0002f) &&
                    Near(sample.gyro_dps[2], -0.0101f, 0.0002f),
                "angular velocity must match manual");
    ok &= Check(Near(sample.mag_ut[0], 7.892f, 0.002f) &&
                    Near(sample.mag_ut[1], 14.625f, 0.002f) &&
                    Near(sample.mag_ut[2], -60.042f, 0.002f),
                "magnetic field must match manual");
    ok &= Check(Near(sample.rpy_deg[0], 13.052f, 0.002f) &&
                    Near(sample.rpy_deg[1], 12.189f, 0.002f) &&
                    Near(sample.rpy_deg[2], -122.477f, 0.002f),
                "Euler angles must match manual");
    ok &= Check(Near(sample.quat[0], -0.4859f, 0.002f) &&
                    Near(sample.quat[1], -0.1498f, 0.002f) &&
                    Near(sample.quat[2], 0.0381f, 0.002f) &&
                    Near(sample.quat[3], 0.8602f, 0.002f),
                "normalized WXYZ quaternion must match manual");
    return ok;
}

static int TestFragmentedAndConcatenated(void)
{
    Hi14Parser parser;
    Hi14Sample sample;
    Hi14ParserInit(&parser);
    int ok = Check(Hi14ParserFeed(&parser, GoldenFrame, 1u, &sample) == 0u,
                   "one header byte must not publish");
    ok &= Check(Hi14ParserFeed(&parser, &GoldenFrame[1], 4u, &sample) == 0u,
                "partial header must not publish");
    ok &= Check(Hi14ParserFeed(&parser,
                               &GoldenFrame[5],
                               sizeof(GoldenFrame) - 5u,
                               &sample) == 1u,
                "fragmented frame must publish once");

    uint8_t joined[sizeof(GoldenFrame) * 2u];
    memcpy(joined, GoldenFrame, sizeof(GoldenFrame));
    memcpy(&joined[sizeof(GoldenFrame)], GoldenFrame, sizeof(GoldenFrame));
    Hi14ParserInit(&parser);
    ok &= Check(Hi14ParserFeed(&parser, joined, sizeof(joined), &sample) == 2u,
                "two adjacent frames must both decode");
    ok &= Check(parser.valid_frames == 2u, "valid frame counter must include both frames");
    return ok;
}

static int TestNoiseBadCrcAndLengthRecovery(void)
{
    Hi14Parser parser;
    Hi14Sample sample;
    uint8_t stream[16u + sizeof(GoldenFrame) * 2u];
    const uint8_t prefix[] = {
        0x00, 0x5A, 0x31, 0xA5,
        0x5A, 0xA5, 0x00, 0x00,
        0x5A, 0xA5, 0x01, 0x02,
    };
    memcpy(stream, prefix, sizeof(prefix));
    memcpy(&stream[sizeof(prefix)], GoldenFrame, sizeof(GoldenFrame));
    stream[sizeof(prefix) + 4u] ^= 0x01u;
    memcpy(&stream[sizeof(prefix) + sizeof(GoldenFrame)], GoldenFrame, sizeof(GoldenFrame));

    Hi14ParserInit(&parser);
    const size_t stream_size = sizeof(prefix) + sizeof(GoldenFrame) * 2u;
    int ok = Check(Hi14ParserFeed(&parser, stream, stream_size, &sample) == 1u,
                   "noise, invalid lengths, and bad CRC must recover to the next good frame");
    ok &= Check(parser.length_errors == 2u, "zero and oversized lengths must be rejected");
    ok &= Check(parser.crc_errors == 1u, "bad CRC must be counted once");
    ok &= Check(parser.valid_frames == 1u, "only the final good frame may publish");
    return ok;
}

static int TestInvalidValuesDoNotPublish(void)
{
    Hi14Parser parser;
    Hi14Sample sample;
    uint8_t frame[sizeof(GoldenFrame)];
    int ok = 1;

    memset(&sample, 0x5A, sizeof(sample));
    Hi14Sample before = sample;
    memcpy(frame, GoldenFrame, sizeof(frame));
    frame[4] ^= 0x01u;
    Hi14ParserInit(&parser);
    ok &= Check(Hi14ParserFeed(&parser, frame, sizeof(frame), &sample) == 0u,
                "bad CRC must be rejected");
    ok &= Check(memcmp(&sample, &before, sizeof(sample)) == 0,
                "bad CRC must not alter caller sample");

    memcpy(frame, GoldenFrame, sizeof(frame));
    frame[6u + 12u] = 0x00u;
    frame[6u + 13u] = 0x00u;
    frame[6u + 14u] = 0xC0u;
    frame[6u + 15u] = 0x7Fu;
    FrameUpdateCrc(frame);
    Hi14ParserInit(&parser);
    ok &= Check(Hi14ParserFeed(&parser, frame, sizeof(frame), &sample) == 0u,
                "NaN measurement must be rejected after a valid CRC");
    ok &= Check(memcmp(&sample, &before, sizeof(sample)) == 0,
                "rejected non-finite frame must not alter caller sample");

    memcpy(frame, GoldenFrame, sizeof(frame));
    memset(&frame[6u + 60u], 0, 16u);
    FrameUpdateCrc(frame);
    Hi14ParserInit(&parser);
    ok &= Check(Hi14ParserFeed(&parser, frame, sizeof(frame), &sample) == 0u,
                "zero quaternion must be rejected");
    ok &= Check(memcmp(&sample, &before, sizeof(sample)) == 0,
                "rejected quaternion must not alter caller sample");
    ok &= Check(parser.data_errors == 1u, "bad quaternion must count as a data error");
    return ok;
}

static int TestStatusRangesAndSensorTime(void)
{
    Hi14Parser parser;
    Hi14Sample sample;
    Hi14SampleGate gate;
    Hi14ParserInit(&parser);
    Hi14SampleGateInit(&gate);
    int ok = Check(Hi14ParserFeed(&parser, GoldenFrame, sizeof(GoldenFrame), &sample) == 1u,
                   "golden frame must remain protocol-valid");
    ok &= Check(!Hi14SampleGateAccept(&gate, &sample),
                "golden status bit 3 must block publication");

    sample.status &= (uint16_t)~HI14_UNTRUSTED_STATUS_MASK;
    sample.sensor_time_ms = UINT32_MAX - 5u;
    ok &= Check(Hi14SampleGateAccept(&gate, &sample), "first trusted sample must publish");
    sample.sensor_time_ms = 3u;
    ok &= Check(Hi14SampleGateAccept(&gate, &sample), "normal uint32 time wrap must publish");
    ok &= Check(!Hi14SampleGateAccept(&gate, &sample), "duplicate device time must not refresh");

    sample.sensor_time_ms = 1u;
    ok &= Check(!Hi14SampleGateAccept(&gate, &sample),
                "first backwards time after a device restart must wait");
    sample.sensor_time_ms = 2u;
    ok &= Check(Hi14SampleGateAccept(&gate, &sample),
                "second increasing time after a device restart must publish");

    const uint16_t rejected_bits[] = {1u << 3u, 1u << 5u, 1u << 6u, 1u << 7u};
    for (size_t i = 0u; i < sizeof(rejected_bits) / sizeof(rejected_bits[0]); ++i) {
        sample.status = rejected_bits[i];
        sample.sensor_time_ms++;
        ok &= Check(!Hi14SampleGateAccept(&gate, &sample),
                    "unconverged or saturated status must block publication");
    }
    sample.status = 0u;
    sample.accel_g[0] = 17.0f;
    sample.sensor_time_ms++;
    ok &= Check(!Hi14SampleGateAccept(&gate, &sample),
                "out-of-range finite acceleration must be rejected before conversion");
    return ok;
}

static int TestRingOverflowResetsStream(void)
{
    Hi14ByteRing ring;
    Hi14Parser parser;
    Hi14Sample sample;
    uint8_t bytes[HI14_BYTE_RING_CAPACITY];
    uint32_t ticks[HI14_BYTE_RING_CAPACITY];
    uint8_t noise[HI14_BYTE_RING_CAPACITY];
    memset(noise, 0xAA, sizeof(noise));
    Hi14ByteRingInit(&ring);
    Hi14ParserInit(&parser);

    int ok = Check(Hi14ByteRingPush(&ring, GoldenFrame, 40u, 100u),
                   "partial frame must fit in the ring");
    uint32_t epoch = 0u;
    size_t count = Hi14ByteRingPop(&ring, bytes, ticks, sizeof(bytes), &epoch);
    ok &= Check(count == 40u && ticks[count - 1u] == 100u,
                "ring must preserve byte count and receive time");
    ok &= Check(Hi14ParserFeed(&parser, bytes, count, &sample) == 0u,
                "partial frame must leave parser waiting");

    ok &= Check(Hi14ByteRingPush(&ring, noise, sizeof(noise), 200u),
                "a full ring written while empty must fit");
    ok &= Check(!Hi14ByteRingPush(&ring, GoldenFrame, sizeof(GoldenFrame), 1234u),
                "new data must report an overflow when stale bytes fill the ring");
    ok &= Check(ring.overflow_count == 1u && ring.epoch == 1u,
                "overflow must advance the discontinuity epoch");

    count = Hi14ByteRingPop(&ring, bytes, ticks, sizeof(bytes), &epoch);
    ok &= Check(epoch == 1u && count == sizeof(GoldenFrame),
                "overflow must discard all stale bytes and keep the new chunk");
    Hi14ParserInit(&parser);
    ok &= Check(Hi14ParserFeed(&parser, bytes, count, &sample) == 1u,
                "parser reset after overflow must decode the next complete frame");
    ok &= Check(ticks[count - 1u] == 1234u,
                "completed frame must retain its receive timestamp");
    return ok;
}

static int TestUnsupportedBoundedFrame(void)
{
    uint8_t frame[6u + HI14_MAX_PAYLOAD_SIZE] = {0x5A, 0xA5, 0x00, 0x02, 0x00, 0x00};
    frame[6] = 0x83u;
    FrameUpdateCrc(frame);

    Hi14Parser parser;
    Hi14Sample sample;
    Hi14ParserInit(&parser);
    int ok = Check(Hi14ParserFeed(&parser, frame, sizeof(frame), &sample) == 0u,
                   "bounded non-HI91 frame must not publish");
    ok &= Check(parser.unsupported_frames == 1u,
                "bounded unsupported frame must be consumed exactly once");
    ok &= Check(Hi14ParserFeed(&parser, GoldenFrame, sizeof(GoldenFrame), &sample) == 1u,
                "parser must recover after a maximum-length unsupported frame");
    return ok;
}

int main(void)
{
    int ok = TestGoldenFrame();
    ok &= TestFragmentedAndConcatenated();
    ok &= TestNoiseBadCrcAndLengthRecovery();
    ok &= TestInvalidValuesDoNotPublish();
    ok &= TestStatusRangesAndSensorTime();
    ok &= TestRingOverflowResetsStream();
    ok &= TestUnsupportedBoundedFrame();
    if (!ok) {
        return 1;
    }
    (void)puts("HI14 parser regression passed");
    return 0;
}
