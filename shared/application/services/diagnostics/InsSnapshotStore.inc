/*
 * SPDX-FileCopyrightText: 2026 陈轩 <2811158416@qq.com>
 * SPDX-License-Identifier: Apache-2.0
 *
 * IMU 快照的板级共用实现，由各板 InsTask.c 在 INS_* 数组定义后包含。
 */

#include "BspTime.h"
#include "StateStore.h"

typedef char InsSnapshotFitsStore[
    (sizeof(InsSnapshot) <= STATE_STORE_IMU_BYTES) ? 1 : -1];

static void InsSnapshotPublishCurrent(uint32_t sample_tick_ms, fp32 temperature_c)
{
    InsSnapshot snapshot = {0};

    snapshot.sample_tick_ms = sample_tick_ms;
    memcpy(snapshot.quat, INS_quat, sizeof(snapshot.quat));
    memcpy(snapshot.angle, INS_angle, sizeof(snapshot.angle));
    memcpy(snapshot.gyro, INS_gyro, sizeof(snapshot.gyro));
    memcpy(snapshot.accel, INS_accel, sizeof(snapshot.accel));
    memcpy(snapshot.mag, INS_mag, sizeof(snapshot.mag));
    snapshot.temperature_c = temperature_c;
    (void)StateStoreWrite(STATE_IMU, &snapshot, (uint16_t)sizeof(snapshot));
}

bool_t InsSnapshotRead(InsSnapshot *out)
{
    state_info_t info = {0};

    if (out == NULL)
    {
        return 0;
    }

    if (StateStoreReadSnapshot(STATE_IMU, out, (uint16_t)sizeof(*out), &info) == 0u)
    {
        return 0;
    }

    out->seq = info.seq;
    out->age_ms = BspTimeGetTickMs() - out->sample_tick_ms;
    out->publish_drop_count = info.write_drop_count;
    return 1;
}
