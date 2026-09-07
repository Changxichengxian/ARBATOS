#include "ImuCalStore.h"
#include "BspImuPwm.h"
#include "ImuFrame.h"
#include <errno.h>
#include <math.h>
#include <stddef.h>
#include <string.h>
#include <zephyr/drivers/hwinfo.h>
#include <zephyr/drivers/flash.h>
#include <zephyr/storage/flash_map.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/crc.h>

typedef struct {
    uint32_t magic, version, sequence;
    uint8_t uid[12];
    float bias[3], temperature;
    uint32_t crc;
} ImuCalRecord;
/* H723 最后两个 128 KiB 扇区；链接脚本将程序限制在前 768 KiB。 */
#define IMU_CAL_SECTOR_SIZE 0x20000u
BUILD_ASSERT(PARTITION_SIZE(imu_cal_partition) == 2u * IMU_CAL_SECTOR_SIZE);
BUILD_ASSERT(PARTITION_OFFSET(imu_cal_partition) == 0xc0000u);
BUILD_ASSERT(IS_ENABLED(CONFIG_USE_DT_CODE_PARTITION));
BUILD_ASSERT(CONFIG_FLASH_LOAD_OFFSET == 0u && CONFIG_FLASH_LOAD_SIZE == 0xc0000u);
BUILD_ASSERT(sizeof(ImuCalRecord) == 44u && offsetof(ImuCalRecord, crc) == 40u);
static int ActiveSlot = -1;
static uint32_t Sequence;
static atomic_t ImuCalWriting;
K_MUTEX_DEFINE(ImuCalLock);
volatile struct {
    uint32_t loaded, saved, sequence;
    int32_t lastError;
    float temperature;
} ImuCalStoreDiag;

int ImuCalStoreIsWriting(void)
{
    return atomic_get(&ImuCalWriting) != 0;
}

static int ImuCalRead(const struct flash_area *area, unsigned slot, ImuCalRecord *record)
{
    uint8_t uid[12];
    if (hwinfo_get_device_id(uid, sizeof(uid)) != sizeof(uid)) return -ENODEV;
    int ret = flash_area_read(area, slot * IMU_CAL_SECTOR_SIZE, record, sizeof(*record));
    if (ret != 0) return ret;
    if (record->magic != 0x4d494341u || (record->version != 1u && record->version != 2u) ||
        memcmp(uid, record->uid, sizeof(uid)) != 0 ||
        record->crc != crc32_ieee((uint8_t *)record, offsetof(ImuCalRecord, crc))) return -EBADMSG;
    for (unsigned i = 0; i < 3; i++) {
        if (!isfinite(record->bias[i]) || fabsf(record->bias[i]) > 0.0873f) return -ERANGE;
    }
    return isfinite(record->temperature) && record->temperature >= 35 && record->temperature <= 45 ? 0 : -ERANGE;
}

int ImuCalStoreLoad(float offset[3])
{
    k_mutex_lock(&ImuCalLock, K_FOREVER);
    ActiveSlot = -1;
    Sequence = 0;
#if defined(CONFIG_ARBATOS_PREFLIGHT_ONLY)
    uint8_t migrate = 0u;
#endif
    const struct flash_area *area;
    int ret = flash_area_open(PARTITION_ID(imu_cal_partition), &area);
    if (ret == 0) {
        ret = -ENOENT;
        ImuCalRecord records[2];
        for (unsigned slot = 0; slot < 2; slot++) {
            if (ImuCalRead(area, slot, &records[slot]) == 0 &&
                (ActiveSlot < 0 || (int32_t)(records[slot].sequence - Sequence) > 0)) {
                ActiveSlot = slot;
                Sequence = records[slot].sequence;
            }
        }
        if (ActiveSlot >= 0) {
            (void)ImuFrameBiasConvert(offset, records[ActiveSlot].bias, records[ActiveSlot].version);
#if defined(CONFIG_ARBATOS_PREFLIGHT_ONLY)
            migrate = records[ActiveSlot].version != ImuFrameVersion();
#endif
            ImuCalStoreDiag.temperature = records[ActiveSlot].temperature;
            ImuCalStoreDiag.loaded = 1;
            ret = 0;
        }
        flash_area_close(area);
    }
    ImuCalStoreDiag.sequence = Sequence;
    ImuCalStoreDiag.lastError = ret;
    k_mutex_unlock(&ImuCalLock);
#if defined(CONFIG_ARBATOS_PREFLIGHT_ONLY)
    /* 只在无车辆输出的准备固件中更新格式；原校准温度保持不变。 */
    if (ret == 0 && migrate) ImuCalStoreQueue(offset, ImuCalStoreDiag.temperature);
#endif
    return ret;
}

int ImuCalStoreSave(const float offset[3], float temperature)
{
#if !defined(CONFIG_ARBATOS_PREFLIGHT_ONLY)
    /* 单 Bank Flash 擦除会停顿取指。正式车辆任务运行时禁止擦写。 */
    ARG_UNUSED(offset);
    ARG_UNUSED(temperature);
    ImuCalStoreDiag.lastError = -EPERM;
    return -EPERM;
#else
    k_mutex_lock(&ImuCalLock, K_FOREVER);
    int ret = -ERANGE;
    const struct flash_area *area = NULL;
    /* 擦除时 CPU 可能暂停，但硬件 PWM 不会自动停，必须先关加热。 */
    atomic_set(&ImuCalWriting, 1);
    imu_pwm_set(0u);
    ImuCalRecord record = {.magic = 0x4d494341u, .version = ImuFrameVersion(), .sequence = Sequence + 1,
                           .temperature = temperature};
    for (unsigned i = 0; i < 3; i++) {
        if (!isfinite(offset[i]) || fabsf(offset[i]) > 0.0873f) goto done;
        record.bias[i] = offset[i];
    }
    if (!isfinite(temperature) || temperature < 35 || temperature > 45) goto done;
    ret = -ENODEV;
    if (hwinfo_get_device_id(record.uid, sizeof(record.uid)) != sizeof(record.uid)) goto done;
    record.crc = crc32_ieee((uint8_t *)&record, offsetof(ImuCalRecord, crc));
    ret = flash_area_open(PARTITION_ID(imu_cal_partition), &area);
    if (ret != 0) goto done;
    if (flash_area_align(area) != 32u) { ret = -ENOTSUP; goto done; }
    unsigned slot = ActiveSlot == 0 ? 1 : 0;
    uint8_t block[64] __aligned(32);
    BUILD_ASSERT(sizeof(ImuCalRecord) <= sizeof(block));
    memset(block, 0xff, sizeof(block));
    memcpy(block, &record, sizeof(record));
    ret = flash_area_erase(area, slot * IMU_CAL_SECTOR_SIZE, IMU_CAL_SECTOR_SIZE);
    if (ret != 0) goto done;
    ret = flash_area_write(area, slot * IMU_CAL_SECTOR_SIZE, block, sizeof(block));
    if (ret != 0) goto done;
    ImuCalRecord check;
    ret = -EIO;
    if (ImuCalRead(area, slot, &check) != 0 || memcmp(&check, &record, sizeof(record)) != 0) goto done;
    /* 新副本完整写入并读回通过后才替代旧副本；保留另一扇区。 */
    ActiveSlot = slot;
    Sequence = record.sequence;
    ImuCalStoreDiag.saved++;
    ImuCalStoreDiag.sequence = Sequence;
    ImuCalStoreDiag.temperature = temperature;
    ret = 0;
done:
    if (area != NULL) flash_area_close(area);
    atomic_clear(&ImuCalWriting);
    ImuCalStoreDiag.lastError = ret;
    k_mutex_unlock(&ImuCalLock);
    return ret;
#endif
}

#if defined(CONFIG_ARBATOS_PREFLIGHT_ONLY)
struct ImuCalPending { float bias[3], temperature; };
K_MSGQ_DEFINE(ImuCalRequests, sizeof(struct ImuCalPending), 1, 4);
static void ImuCalWorker(void *a, void *b, void *c)
{
    ARG_UNUSED(a); ARG_UNUSED(b); ARG_UNUSED(c);
    struct ImuCalPending pending;
    for (;;) {
        k_msgq_get(&ImuCalRequests, &pending, K_FOREVER);
        (void)ImuCalStoreSave(pending.bias, pending.temperature);
    }
}
K_THREAD_DEFINE(ImuCalThread, 4096, ImuCalWorker, NULL, NULL, NULL, 13, 0, 0);
void ImuCalStoreQueue(const float offset[3], float temperature)
{
    struct ImuCalPending pending = {.temperature = temperature};
    memcpy(pending.bias, offset, sizeof(pending.bias));
    if (k_msgq_put(&ImuCalRequests, &pending, K_NO_WAIT) != 0) ImuCalStoreDiag.lastError = -EBUSY;
}
#endif
