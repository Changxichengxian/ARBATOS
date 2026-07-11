/*
 * SPDX-FileCopyrightText: 2026 Xie Yuhan <2811158416@qq.com>
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef DETECT_COMMON_H
#define DETECT_COMMON_H

#include <stdint.h>
#include <string.h>

#include "RobotConfig.h"
#include "DetectTask.h"

#ifndef DETECT_COMMON_DEFAULT_OFFLINE_MS
#define DETECT_COMMON_DEFAULT_OFFLINE_MS 200u
#endif

#ifndef DETECT_COMMON_DBUS_OFFLINE_MS
#define DETECT_COMMON_DBUS_OFFLINE_MS 100u
#endif

#ifndef DETECT_COMMON_RUNTIME_POLL_MS
#define DETECT_COMMON_RUNTIME_POLL_MS DETECT_CONTROL_TIME
#endif

typedef struct
{
    uint32_t latestTickMs;
    uint32_t previousTickMs;
    uint32_t firstPendingTickMs;
    uint32_t baselineTickMs;
    uint32_t sequence;
    uint8_t received;
    uint8_t pending;
    uint8_t baselineValid;
    uint8_t reserved0;
} DetectReceiptFact;

typedef struct
{
    DetectError working[DETECT_ERROR_COUNT + 1u];
    DetectSnapshot snapshot[2];
    DetectReceiptFact receipt[DETECT_ERROR_COUNT];
    uint32_t publishSeq;
    uint8_t activeIndex;
    uint8_t writerInitialized;
} DetectRuntime;

static inline void DetectCommonInitFromConfig(DetectError *list,
                                               uint8_t count,
                                               const DetectConfig *cfg,
                                               uint32_t now_ms)
{
    if (list == NULL)
    {
        return;
    }

    (void)memset(list, 0, (size_t)count * sizeof(list[0]));

    for (uint8_t i = 0u; i < count; i++)
    {
        const uint8_t enabled = (cfg != NULL) ? (uint8_t)((cfg->enable_mask >> i) & 0x1u) : 1u;

        list[i].enable = enabled;
        list[i].priority = (cfg != NULL) ? cfg->items[i].priority : 0u;
        list[i].set_online_time = (cfg != NULL) ? cfg->items[i].online_time_ms : 0u;
        list[i].set_offline_time = (cfg != NULL) ? cfg->items[i].offline_time_ms : DETECT_COMMON_DEFAULT_OFFLINE_MS;
        list[i].error_exist = enabled;
        list[i].is_lost = enabled;
        list[i].data_is_error = enabled;
        list[i].new_time = now_ms;
        list[i].last_time = now_ms;
        list[i].lost_time = now_ms;
        list[i].work_time = now_ms;
    }

    if (cfg == NULL && (uint8_t)DBUS_TOE < count)
    {
        list[DBUS_TOE].set_offline_time = DETECT_COMMON_DBUS_OFFLINE_MS;
    }
}

/* 调用者负责用极短临界区保护；这里只记录接收事实，不计算健康状态。 */
static inline void DetectCommonHook(DetectReceiptFact *facts,
                                    uint8_t count,
                                    uint8_t toe,
                                    uint32_t now_ms)
{
    DetectReceiptFact *fact;

    if (facts == NULL || toe >= count)
    {
        return;
    }

    fact = &facts[toe];
    if (fact->pending == 0u)
    {
        fact->firstPendingTickMs = now_ms;
    }
    if (fact->received != 0u)
    {
        fact->previousTickMs = fact->latestTickMs;
    }
    else
    {
        fact->previousTickMs = now_ms;
    }
    fact->latestTickMs = now_ms;
    fact->received = 1u;
    fact->pending = 1u;
    fact->sequence++;
}

/* HERO 旧目标从任务启动时起计算首次超时；真实 Hook 已到达时绝不覆盖。 */
static inline void DetectCommonSeedBaseline(DetectReceiptFact *facts,
                                            uint8_t count,
                                            uint32_t now_ms)
{
    if (facts == NULL)
    {
        return;
    }

    for (uint8_t i = 0u; i < count; i++)
    {
        if (facts[i].received == 0u)
        {
            facts[i].baselineTickMs = now_ms;
            facts[i].baselineValid = 1u;
        }
    }
}

/* 调用者负责临界区；取走 pending 后，新 Hook 会进入下一批，不会被初始化清零。 */
static inline void DetectCommonTakeFact(DetectReceiptFact *facts,
                                        DetectReceiptFact *out,
                                        uint8_t count,
                                        uint8_t toe)
{
    if (facts == NULL || out == NULL || toe >= count)
    {
        return;
    }

    *out = facts[toe];
    facts[toe].pending = 0u;
}

static inline void DetectCommonRefreshOne(DetectError *e,
                                          const DetectReceiptFact *fact,
                                          uint32_t now_ms)
{
    uint32_t last_tick_ms = 0u;
    uint8_t have_tick = 0u;

    if (e == NULL || fact == NULL)
    {
        return;
    }

    if (e->enable == 0u || e->set_offline_time == 0u)
    {
        e->is_lost = 0u;
        e->error_exist = 0u;
        return;
    }

    if (fact->pending != 0u)
    {
        const uint8_t was_lost = e->is_lost;
        const uint32_t delta_ms = fact->latestTickMs - fact->previousTickMs;

        e->last_time = (fact->sequence > 1u) ? fact->previousTickMs : e->new_time;
        e->new_time = fact->latestTickMs;
        if (was_lost != 0u)
        {
            e->work_time = fact->firstPendingTickMs;
        }
        if (e->data_is_error_fun != NULL && e->data_is_error_fun())
        {
            e->data_is_error = 1u;
            if (e->solve_data_error_fun != NULL)
            {
                e->solve_data_error_fun();
            }
        }
        else
        {
            e->data_is_error = 0u;
        }
        if (fact->sequence > 1u && delta_ms != 0u)
        {
            e->frequency = 1000.0f / (fp32)delta_ms;
        }
    }

    if (fact->received != 0u)
    {
        last_tick_ms = fact->latestTickMs;
        have_tick = 1u;
    }
    else if (fact->baselineValid != 0u)
    {
        last_tick_ms = fact->baselineTickMs;
        have_tick = 1u;
    }

    if (have_tick == 0u ||
        (uint32_t)(now_ms - last_tick_ms) > (uint32_t)e->set_offline_time)
    {
        e->is_lost = 1u;
        e->error_exist = 1u;
        e->lost_time = now_ms;
        if (e->solve_lost_fun != NULL)
        {
            e->solve_lost_fun();
        }
        return;
    }

    e->is_lost = 0u;
    if ((uint32_t)(now_ms - e->work_time) < (uint32_t)e->set_online_time)
    {
        e->error_exist = 1u;
    }
    else
    {
        e->error_exist = e->data_is_error;
    }
}

static inline void DetectCommonRefreshAll(DetectError *list,
                                          const DetectReceiptFact *facts,
                                          uint8_t count,
                                          uint32_t now_ms)
{
    if (list == NULL || facts == NULL)
    {
        return;
    }

    for (uint8_t i = 0u; i < count; i++)
    {
        DetectCommonRefreshOne(&list[i], &facts[i], now_ms);
    }
}

/* 纯读取：不取当前时间、不修改状态，也不调用任何处理函数。 */
static inline bool_t DetectCommonIsError(const DetectError *list,
                                         uint8_t count,
                                         uint8_t toe)
{
    if (list == NULL || toe >= count)
    {
        return 1u;
    }
    if (list[toe].enable == 0u || list[toe].set_offline_time == 0u)
    {
        return 0u;
    }
    return (bool_t)list[toe].error_exist;
}

/* 只写非活动 bank；调用者写完后再用极短临界区切换 activeIndex。 */
static inline void DetectCommonPublish(DetectSnapshot *snapshot,
                                       const DetectError *list,
                                       uint8_t count,
                                       uint32_t now_ms,
                                       uint32_t seq)
{
    uint16_t errorMask = 0u;
    uint16_t lostMask = 0u;
    uint16_t dataErrorMask = 0u;

    if (snapshot == NULL || list == NULL || count > (uint8_t)DETECT_ERROR_COUNT)
    {
        return;
    }

    for (uint8_t i = 0u; i < count; i++)
    {
        DetectState *state = &snapshot->state[i];
        const DetectError *error = &list[i];

        state->newTimeMs = error->new_time;
        state->lastTimeMs = error->last_time;
        state->lostTimeMs = error->lost_time;
        state->workTimeMs = error->work_time;
        state->offlineTimeMs = error->set_offline_time;
        state->onlineTimeMs = error->set_online_time;
        state->frequencyHz = error->frequency;
        state->enable = error->enable;
        state->priority = error->priority;
        state->errorExist = error->error_exist;
        state->isLost = error->is_lost;
        state->dataIsError = error->data_is_error;
        state->reserved0 = 0u;
        state->reserved1 = 0u;
        state->reserved2 = 0u;

        if (i < 16u)
        {
            const uint16_t bit = (uint16_t)(1u << i);
            if (error->enable != 0u && error->set_offline_time != 0u && error->error_exist != 0u)
            {
                errorMask |= bit;
            }
            if (error->is_lost != 0u)
            {
                lostMask |= bit;
            }
            if (error->data_is_error != 0u)
            {
                dataErrorMask |= bit;
            }
        }
    }
    snapshot->publishTimeMs = now_ms;
    snapshot->errorMask = errorMask;
    snapshot->lostMask = lostMask;
    snapshot->dataErrorMask = dataErrorMask;
    snapshot->seq = (seq != 0u) ? seq : 1u;
    snapshot->valid = 1u;
}

/* 调用者负责临界区；读取不会取时间、推进状态或调用故障处理函数。 */
static inline uint8_t DetectCommonSnapshotRead(const DetectSnapshot *snapshot,
                                               DetectSnapshot *out)
{
    if (out == NULL)
    {
        return 0u;
    }
    if (snapshot == NULL || snapshot->valid == 0u)
    {
        (void)memset(out, 0, sizeof(*out));
        return 0u;
    }

    *out = *snapshot;
    return 1u;
}

static inline uint8_t DetectCommonSummaryRead(const DetectSnapshot *snapshot,
                                              DetectSummary *out)
{
    if (out == NULL)
    {
        return 0u;
    }
    if (snapshot == NULL || snapshot->valid == 0u)
    {
        (void)memset(out, 0, sizeof(*out));
        return 0u;
    }

    out->seq = snapshot->seq;
    out->publishTimeMs = snapshot->publishTimeMs;
    out->errorMask = snapshot->errorMask;
    out->lostMask = snapshot->lostMask;
    out->dataErrorMask = snapshot->dataErrorMask;
    out->valid = snapshot->valid;
    out->reserved0 = 0u;
    return 1u;
}

static inline bool_t DetectCommonSnapshotIsError(const DetectSnapshot *snapshot,
                                                 uint8_t count,
                                                 uint8_t toe)
{
    if (snapshot == NULL || snapshot->valid == 0u)
    {
        return 1u;
    }
    if (toe >= count)
    {
        return 1u;
    }
    return (bool_t)((snapshot->errorMask & (uint16_t)(1u << toe)) != 0u);
}

#endif
