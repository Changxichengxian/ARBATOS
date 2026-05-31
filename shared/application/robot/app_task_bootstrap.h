/*
 * SPDX-FileCopyrightText: 2026 Xie Yuhan <2811158416@qq.com>
 * SPDX-License-Identifier: PolyForm-Noncommercial-1.0.0
 */

#ifndef APP_TASK_BOOTSTRAP_H
#define APP_TASK_BOOTSTRAP_H

#include <stdint.h>

#include "cmsis_os2.h"
#include "robot_task_profile.h"

typedef osThreadId_t (*app_task_create_fn_t)(void);

typedef struct
{
    robot_task_module_id_t module_id;
    osThreadId_t *handle;
    app_task_create_fn_t create;
} app_task_module_desc_t;

static inline void app_create_enabled_module_tasks(const app_task_module_desc_t *module_tasks,
                                                   uint32_t module_task_count)
{
    if (module_tasks == NULL)
    {
        return;
    }

    for (uint32_t i = 0u; i < module_task_count; i++)
    {
        const app_task_module_desc_t *task = &module_tasks[i];

        if (robot_profile_module_id_enabled(task->module_id) == 0u ||
            task->handle == NULL ||
            task->create == NULL)
        {
            continue;
        }
        if (*task->handle != NULL)
        {
            continue;
        }

        *task->handle = task->create();
    }
}

#endif
