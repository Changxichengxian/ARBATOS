/*
 * SPDX-FileCopyrightText: 2026 Xie Yuhan <2811158416@qq.com>
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef APP_TASK_BOOTSTRAP_H
#define APP_TASK_BOOTSTRAP_H

#include <stdint.h>

#include "cmsis_os2.h"
#include "RobotTaskProfile.h"
#include "Watch.h"

typedef osThreadId_t (*AppTaskCreateFn)(void);

typedef enum
{
    APP_TASK_BOOTSTRAP_STATE_NONE = 0u,
    APP_TASK_BOOTSTRAP_STATE_ENABLED = 1u,
    APP_TASK_BOOTSTRAP_STATE_CREATED = 2u,
    APP_TASK_BOOTSTRAP_STATE_CREATE_FAILED = 3u,
    APP_TASK_BOOTSTRAP_STATE_MISSING_DESC = 4u,
    APP_TASK_BOOTSTRAP_STATE_BAD_DESC = 5u,
    APP_TASK_BOOTSTRAP_STATE_ALREADY_CREATED = 6u,
} AppTaskBootstrapState;

typedef struct
{
    RobotTaskModuleId module_id;
    osThreadId_t *handle;
    AppTaskCreateFn create;
    const char *name;
} AppTaskModuleDesc;

#define APP_TASK_MODULE_DESC(module_, handle_, create_) \
    {(module_), (handle_), (create_), RobotProfileModuleName((RobotTaskModuleId)(module_))}

static inline const AppTaskModuleDesc *AppTaskFindModuleDesc(const AppTaskModuleDesc *module_tasks,
                                                                      uint32_t module_task_count,
                                                                      RobotTaskModuleId module_id)
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

static inline void AppTaskModuleClearHandles(const AppTaskModuleDesc *module_tasks,
                                                 uint32_t module_task_count)
{
    if (module_tasks == NULL)
    {
        return;
    }

    for (uint32_t task_index = 0u; task_index < module_task_count; task_index++)
    {
        if (module_tasks[task_index].handle != NULL)
        {
            *module_tasks[task_index].handle = NULL;
        }
    }
}

static inline uint8_t AppCreateEnabledModuleTasks(const AppTaskModuleDesc *module_tasks,
                                                      uint32_t module_task_count)
{
    uint8_t fail_count = 0u;

    WatchTaskModuleCreateReset();

    if (module_tasks == NULL)
    {
        return 1u;
    }

    AppTaskModuleClearHandles(module_tasks, module_task_count);

    for (uint8_t profile_index = 0u; profile_index < RobotProfileModuleCount(); profile_index++)
    {
        const RobotTaskModuleId module_id = RobotProfileModuleIdAt(profile_index);
        const AppTaskModuleDesc *task =
            AppTaskFindModuleDesc(module_tasks, module_task_count, module_id);
        const char *name = RobotProfileModuleName(module_id);

        if (task != NULL && task->name != NULL)
        {
            name = task->name;
        }

        if (task == NULL)
        {
            WatchTaskModuleCreateResult((uint8_t)module_id,
                                            name,
                                            0u,
                                            (uint8_t)APP_TASK_BOOTSTRAP_STATE_MISSING_DESC);
            fail_count++;
            continue;
        }

        WatchTaskModuleCreateResult((uint8_t)module_id,
                                        name,
                                        0u,
                                        (uint8_t)APP_TASK_BOOTSTRAP_STATE_ENABLED);

        if (task->handle == NULL || task->create == NULL)
        {
            WatchTaskModuleCreateResult((uint8_t)module_id,
                                            name,
                                            0u,
                                            (uint8_t)APP_TASK_BOOTSTRAP_STATE_BAD_DESC);
            fail_count++;
            continue;
        }
        if (*task->handle != NULL)
        {
            WatchTaskModuleCreateResult((uint8_t)module_id,
                                            name,
                                            (uint32_t)(uintptr_t)(*task->handle),
                                            (uint8_t)APP_TASK_BOOTSTRAP_STATE_ALREADY_CREATED);
            continue;
        }

        *task->handle = task->create();
        if (*task->handle == NULL)
        {
            WatchTaskModuleCreateResult((uint8_t)module_id,
                                            name,
                                            0u,
                                            (uint8_t)APP_TASK_BOOTSTRAP_STATE_CREATE_FAILED);
            fail_count++;
        }
        else
        {
            WatchTaskModuleCreateResult((uint8_t)module_id,
                                            name,
                                            (uint32_t)(uintptr_t)(*task->handle),
                                            (uint8_t)APP_TASK_BOOTSTRAP_STATE_CREATED);
        }

    }

    return fail_count;
}

#endif
