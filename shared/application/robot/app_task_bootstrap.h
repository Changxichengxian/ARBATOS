/*
 * SPDX-FileCopyrightText: 2026 Xie Yuhan <2811158416@qq.com>
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef APP_TASK_BOOTSTRAP_H
#define APP_TASK_BOOTSTRAP_H

#include <stdint.h>

#include "cmsis_os2.h"
#include "robot_task_profile.h"
#include "watch.h"

typedef osThreadId_t (*app_task_create_fn_t)(void);

typedef enum
{
    APP_TASK_BOOTSTRAP_STATE_NONE = 0u,
    APP_TASK_BOOTSTRAP_STATE_ENABLED = 1u,
    APP_TASK_BOOTSTRAP_STATE_CREATED = 2u,
    APP_TASK_BOOTSTRAP_STATE_CREATE_FAILED = 3u,
    APP_TASK_BOOTSTRAP_STATE_MISSING_DESC = 4u,
    APP_TASK_BOOTSTRAP_STATE_BAD_DESC = 5u,
    APP_TASK_BOOTSTRAP_STATE_ALREADY_CREATED = 6u,
} app_task_bootstrap_state_e;

typedef struct
{
    robot_task_module_id_t module_id;
    osThreadId_t *handle;
    app_task_create_fn_t create;
    const char *name;
} app_task_module_desc_t;

#define APP_TASK_MODULE_DESC(module_, handle_, create_) \
    {(module_), (handle_), (create_), robot_profile_module_name((robot_task_module_id_t)(module_))}

static inline const app_task_module_desc_t *app_task_find_module_desc(const app_task_module_desc_t *module_tasks,
                                                                      uint32_t module_task_count,
                                                                      robot_task_module_id_t module_id)
{
    for (uint32_t task_index = 0u; task_index < module_task_count; task_index++)
    {
        if (module_tasks[task_index].module_id == module_id)
        {
            return &module_tasks[task_index];
        }
    }

    return NULL;
}

static inline uint8_t app_create_enabled_module_tasks(const app_task_module_desc_t *module_tasks,
                                                      uint32_t module_task_count)
{
    uint8_t fail_count = 0u;

    watch_task_module_create_reset();

    if (module_tasks == NULL)
    {
        return 1u;
    }

    for (uint8_t profile_index = 0u; profile_index < robot_profile_module_count(); profile_index++)
    {
        const robot_task_module_id_t module_id = robot_profile_module_id_at(profile_index);
        const app_task_module_desc_t *task =
            app_task_find_module_desc(module_tasks, module_task_count, module_id);
        const char *name = robot_profile_module_name(module_id);

        if (task != NULL && task->name != NULL)
        {
            name = task->name;
        }

        if (task == NULL)
        {
            watch_task_module_create_result((uint8_t)module_id,
                                            name,
                                            0u,
                                            (uint8_t)APP_TASK_BOOTSTRAP_STATE_MISSING_DESC);
            fail_count++;
            continue;
        }

        watch_task_module_create_result((uint8_t)module_id,
                                        name,
                                        0u,
                                        (uint8_t)APP_TASK_BOOTSTRAP_STATE_ENABLED);

        if (task->handle == NULL || task->create == NULL)
        {
            watch_task_module_create_result((uint8_t)module_id,
                                            name,
                                            0u,
                                            (uint8_t)APP_TASK_BOOTSTRAP_STATE_BAD_DESC);
            fail_count++;
            continue;
        }
        if (*task->handle != NULL)
        {
            watch_task_module_create_result((uint8_t)module_id,
                                            name,
                                            (uint32_t)(uintptr_t)(*task->handle),
                                            (uint8_t)APP_TASK_BOOTSTRAP_STATE_ALREADY_CREATED);
            continue;
        }

        *task->handle = task->create();
        if (*task->handle == NULL)
        {
            watch_task_module_create_result((uint8_t)module_id,
                                            name,
                                            0u,
                                            (uint8_t)APP_TASK_BOOTSTRAP_STATE_CREATE_FAILED);
            fail_count++;
        }
        else
        {
            watch_task_module_create_result((uint8_t)module_id,
                                            name,
                                            (uint32_t)(uintptr_t)(*task->handle),
                                            (uint8_t)APP_TASK_BOOTSTRAP_STATE_CREATED);
        }

    }

    return fail_count;
}

#endif
