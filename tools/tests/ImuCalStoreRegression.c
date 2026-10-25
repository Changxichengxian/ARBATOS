/* 主机回归：直接编译真实 ImuCalStore.c；Flash/UID/PWM 均为内存桩，非断电或实板验证。 */
#include <errno.h>
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "ImuCalStore.h"
#include <zephyr/drivers/flash.h>

#define SECTOR 0x20000u
typedef struct {
    uint32_t magic, version, sequence;
    uint8_t uid[12];
    float bias[3], temperature;
    uint32_t crc;
} TestRecord;

static uint8_t Flash[2u * SECTOR];
static const uint8_t DeviceId[12] = { 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12 };
static struct flash_area TestArea;
static size_t Alignment = 4u;
static int FailErase, FailWrite, CorruptWrite;
static unsigned PwmCalls;

void imu_pwm_set(uint16_t pwm) { if (pwm == 0u) PwmCalls++; }

int hwinfo_get_device_id(uint8_t *buffer, size_t length)
{
    if (length != sizeof(DeviceId)) return -1;
    memcpy(buffer, DeviceId, sizeof(DeviceId));
    return (int)sizeof(DeviceId);
}
int flash_area_open(uint8_t id, const struct flash_area **area)
{ if (id != 1u) return -ENODEV; *area = &TestArea; return 0; }
void flash_area_close(const struct flash_area *area) { (void)area; }
int flash_area_read(const struct flash_area *area, off_t offset, void *data, size_t length)
{ if (area != &TestArea || offset < 0 || (size_t)offset + length > sizeof(Flash)) return -EINVAL; memcpy(data, Flash + offset, length); return 0; }
int flash_area_write(const struct flash_area *area, off_t offset, const void *data, size_t length)
{
    if (area != &TestArea || FailWrite) return -EIO;
    if (offset < 0 || (size_t)offset + length > sizeof(Flash)) return -EINVAL;
    memcpy(Flash + offset, data, length);
    if (CorruptWrite) Flash[offset] ^= 1u;
    return 0;
}
int flash_area_erase(const struct flash_area *area, off_t offset, size_t length)
{
    if (area != &TestArea || FailErase) return -EIO;
    if (offset < 0 || (size_t)offset + length > sizeof(Flash)) return -EINVAL;
    memset(Flash + offset, 0xff, length); return 0;
}
size_t flash_area_align(const struct flash_area *area) { return area == &TestArea ? Alignment : 0u; }
uint32_t crc32_ieee(const uint8_t *data, size_t length)
{
    uint32_t crc = 0xffffffffu;
    for (size_t i = 0; i < length; ++i) { crc ^= data[i]; for (unsigned bit = 0; bit < 8; ++bit) crc = (crc >> 1) ^ (0xedb88320u & (0u - (crc & 1u))); }
    return ~crc;
}

#include "../../shared/zephyr/port/sensors/ImuCalStore.c"

static int Check(int ok, const char *what)
{ if (ok) return 1; (void)fprintf(stderr, "FAIL: %s\n", what); return 0; }
#if defined(TEST_PREFLIGHT)
static int Near(float a, float b) { return fabsf(a - b) < 1e-6f; }
static int Vector(const float a[3], const float b[3]) { return Near(a[0], b[0]) && Near(a[1], b[1]) && Near(a[2], b[2]); }
#endif
static void Reset(void)
{
    memset(Flash, 0xff, sizeof(Flash)); Alignment = 4u; FailErase = FailWrite = CorruptWrite = 0;
    PwmCalls = 0u; (void)ImuCalStoreLoad((float[3]){0});
}
#if defined(TEST_PREFLIGHT)
static void Put(unsigned slot, uint32_t version, uint32_t sequence, const float raw[3], float temperature)
{
    TestRecord record = { .magic = 0x4d494341u, .version = version, .sequence = sequence, .temperature = temperature };
    memcpy(record.uid, DeviceId, sizeof(record.uid));
    if (version == 1u) { record.bias[0] = raw[1]; record.bias[1] = -raw[0]; record.bias[2] = raw[2]; }
    else memcpy(record.bias, raw, sizeof(record.bias));
    record.crc = crc32_ieee((const uint8_t *)&record, offsetof(TestRecord, crc));
    memcpy(Flash + slot * SECTOR, &record, sizeof(record));
}
#endif
static int TestAuthorizationAndAlignment(void)
{
    const float offset[3] = { .01f, -.02f, .003f };
#if !defined(TEST_PREFLIGHT)
    Reset();
    if (!Check(ImuCalStoreSave(offset, 40.0f) == -EPERM, "正式固件未授权写入未被拒绝")) return 0;
    return 1;
#else
    for (unsigned i = 0; i < 3; ++i) {
        const size_t alignments[] = { 1u, 4u, 32u }; Reset(); Alignment = alignments[i];
        if (!Check(ImuCalStoreSave(offset, 40.0f) == 0 && ImuCalStoreSequence() == 1u,
                   "允许的 Flash 对齐写入失败") || !Check(PwmCalls == 1u, "写前没有关闭 IMU 加热")) return 0;
    }
    return 1;
#endif
}
#if defined(TEST_PREFLIGHT)
static int TestVersionsAndMapping(void)
{
    const float raw[3] = { .01f, -.02f, .003f }, expected_input[3] = { .01f, -.02f, .003f };
    float expected[3], actual[3]; ImuFrameRotate(expected, expected_input);
    for (uint32_t version = 1u; version <= 3u; ++version) {
        Reset(); Put(0u, version, version, raw, 40.0f);
        if (!Check(ImuCalStoreLoad(actual) == 0 && Vector(actual, expected), "旧/新校准版本没有按当前安装方向转换")) return 0;
    }
    return 1;
}
static int TestDoubleCopyAndFailures(void)
{
    const float old[3] = { .01f, -.02f, .003f }, newer[3] = { -.01f, .02f, -.003f };
    float actual[3], rotated_newer[3]; ImuFrameRotate(rotated_newer, newer);
    Reset(); Put(0u, 3u, 5u, old, 40.0f); Put(1u, 3u, 9u, newer, 40.0f);
    if (!Check(ImuCalStoreLoad(actual) == 0 && ImuCalStoreSequence() == 9u && Vector(actual, rotated_newer), "双副本没有选择最新有效记录")) return 0;
    Reset();
    if (!Check(ImuCalStoreSave(old, 40.0f) == 0 && ImuCalStoreSave(newer, 40.0f) == 0 && ImuCalStoreSequence() == 2u,
               "重复保存没有推进序号")) return 0;
    int *failures[] = { &FailErase, &FailWrite, &CorruptWrite };
    for (unsigned i = 0; i < 3; ++i) {
        Reset(); if (!Check(ImuCalStoreSave(old, 40.0f) == 0, "旧副本准备失败")) return 0;
        *failures[i] = 1;
        if (!Check(ImuCalStoreSave(newer, 40.0f) != 0, "故障写入被错误接受")) return 0;
        if (!Check(ImuCalStoreLoad(actual) == 0 && Vector(actual, old), "写入失败后旧有效记录未保留")) return 0;
        *failures[i] = 0;
    }
    return 1;
}
static int TestRejectedRecordsAndInputs(void)
{
    const float raw[3] = { .01f, -.02f, .003f }, nan_offset[3] = { NAN, 0.0f, 0.0f };
    float actual[3];
    Reset(); Put(0u, 3u, 1u, raw, 40.0f);
    if (!Check(ImuCalStoreLoad(actual) == 0 && ImuCalStoreLoaded(), "有效记录没有标记已加载")) return 0;
    memset(Flash, 0xff, sizeof(Flash));
    if (!Check(ImuCalStoreLoad(actual) == -ENOENT && !ImuCalStoreLoaded(), "失效记录仍报告已加载")) return 0;
    Reset(); Put(0u, 3u, 1u, raw, 40.0f); Flash[offsetof(TestRecord, uid)] ^= 1u;
    if (!Check(ImuCalStoreLoad(actual) == -ENOENT, "错误 UID 记录被接受")) return 0;
    Reset(); Put(0u, 3u, 1u, raw, 40.0f); Flash[offsetof(TestRecord, crc)] ^= 1u;
    if (!Check(ImuCalStoreLoad(actual) == -ENOENT, "错误 CRC 记录被接受")) return 0;
    Reset(); Put(0u, 3u, 1u, raw, 55.0f);
    if (!Check(ImuCalStoreLoad(actual) == -ENOENT, "错误温度记录被接受")) return 0;
    Reset();
    return Check(ImuCalStoreSave(nan_offset, 40.0f) == -ERANGE, "非有限 offset 被接受") &&
           Check(ImuCalStoreSave(raw, NAN) == -ERANGE, "非有限温度被接受");
}
#endif
int main(void)
{
    if (!TestAuthorizationAndAlignment()) return 1;
#if defined(TEST_PREFLIGHT)
    if (!TestVersionsAndMapping() || !TestDoubleCopyAndFailures() || !TestRejectedRecordsAndInputs()) return 1;
#endif
    (void)puts("PASS: ImuCalStore host regression (Flash memory stub; not power-loss or hardware validation)");
    return 0;
}
