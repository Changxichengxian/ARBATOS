/*
 * SPDX-FileCopyrightText: 2026 陈轩 <2811158416@qq.com>
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdint.h>

#include <zephyr/devicetree.h>
#include <zephyr/kernel.h>
#include <zephyr/linker/section_tags.h>

#include "FreeRTOS.h"
#include "cmsis_os2.h"

#include "ArbatosRuntime.h"
#include "AppTaskBootstrap.h"
#include "BspBuzzer.h"
#include "BspUsb.h"
#include "ManualInput.h"
#include "RobotConfig.h"
#include "RobotControlRegistry.h"
#include "RobotTaskBuildConfig.h"
#include "RobotTargetTaskHeaders.h"
#include "Watch.h"
#if defined(CONFIG_ARBATOS_SUBBOARD_MUSIC)
#include "SubBoardMusic.h"
#endif

#ifndef ROBOT_WATCH_UPDATE_PERIOD_MS
#define ROBOT_WATCH_UPDATE_PERIOD_MS 1000u
#endif

#if DT_HAS_CHOSEN(zephyr_dtcm)
#define ARB_RUNTIME_STACK_SECTION __dtcm_bss_section
#else
#define ARB_RUNTIME_STACK_SECTION
#endif

#define ARB_STATIC_THREAD(thread_id, prio, stack_words)                                  \
    static struct z_thread_stack_element thread_id##_stack[                              \
        K_KERNEL_STACK_LEN((stack_words) * sizeof(StackType_t))]                         \
        __aligned(Z_KERNEL_STACK_OBJ_ALIGN) ARB_RUNTIME_STACK_SECTION;                    \
    static StaticTask_t thread_id##_control_block;                                       \
    static const osThreadAttr_t thread_id##_attr = {                                     \
        .name = #thread_id,                                                              \
        .priority = (prio),                                                              \
        .stack_mem = thread_id##_stack,                                                  \
        .stack_size = sizeof(thread_id##_stack),                                         \
        .cb_mem = &thread_id##_control_block,                                            \
        .cb_size = sizeof(thread_id##_control_block),                                    \
    }

#define ARB_THREAD_CREATE(thread_id, entry) \
    osThreadNew((osThreadFunc_t)(entry), NULL, &thread_id##_attr)

static osThreadId_t ArbDefaultTaskHandle;
ARB_STATIC_THREAD(defaultTask, osPriorityNormal, ROBOT_RUNTIME_DEFAULT_STACK_WORDS);

#define ROBOT_RUNTIME_TASK(symbol, entry, threadName, priority, stackWords) \
    static osThreadId_t ArbTaskHandle_##symbol;                          \
    ARB_STATIC_THREAD(threadName, priority, stackWords);
#include "RobotTargetTasks.inc"
#undef ROBOT_RUNTIME_TASK

static void ArbDefaultTask(void *argument)
{
    (void)argument;

    WatchInit();
    WatchDiagSetBootStage(WATCH_BOOT_STAGE_DEFAULT_TASK_START);
    WatchDiagSetBootStage(WATCH_BOOT_STAGE_RUN);

    for (;;)
    {
        WatchTaskBeat(WATCH_TASK_DEFAULT);
        WatchUpdate();
        (void)osDelay(ROBOT_WATCH_UPDATE_PERIOD_MS);
    }
}


#define ROBOT_RUNTIME_TASK(symbol, entry, threadName, priority, stackWords) \
    static osThreadId_t ArbCreateTask_##symbol(void)                        \
    {                                                                        \
        return ARB_THREAD_CREATE(threadName, entry);                         \
    }
#include "RobotTargetTasks.inc"
#undef ROBOT_RUNTIME_TASK

static uint8_t ArbCreateModuleTasks(void)
{
    const AppTaskModuleDesc moduleTasks[] = {
#define ROBOT_RUNTIME_TASK(symbol, entry, threadName, priority, stackWords) \
        APP_TASK_MODULE_DESC(ROBOT_TASK_MODULE_##symbol, &ArbTaskHandle_##symbol, ArbCreateTask_##symbol),
#include "RobotTargetTasks.inc"
#undef ROBOT_RUNTIME_TASK
        {ROBOT_TASK_MODULE_NONE, NULL, NULL, NULL},
    };

    return AppCreateEnabledModuleTasks(moduleTasks,
                                       (uint32_t)(sizeof(moduleTasks) / sizeof(moduleTasks[0])));
}

ArbatosRuntimeStatus ArbatosRuntimeStart(void)
{
    static uint8_t attempted;
    static ArbatosRuntimeStatus result = ARBATOS_RUNTIME_OK;

    if (attempted != 0u)
    {
        return (result == ARBATOS_RUNTIME_OK) ? ARBATOS_RUNTIME_ALREADY_STARTED : result;
    }
    attempted = 1u;

#if defined(CONFIG_ARBATOS_PREFLIGHT_ONLY)
    /* 静态准备只启动采样与记录，不创建任何电机或外接舵机任务。 */
    extern int MPreflightStart(void);
    if (RobotControlBootstrapProfileDefaults() == 0u) {
        result = ARBATOS_RUNTIME_CONTROL_CONFIG_FAILED;
        return result;
    }
    ManualInputInit();
    if (MPreflightStart() != 0) {
        result = ARBATOS_RUNTIME_MODULE_TASK_FAILED;
    }
    return result;
#endif

#if defined(CONFIG_ARBATOS_MUSIC_ONLY)
    /* HERO-M 运动映射尚待实车核对；音乐验证不初始化总线和控制输出。 */
    extern int BspBuzzerPlatformInit(void);
    if (BspBuzzerPlatformInit() != 0) {
        result = ARBATOS_RUNTIME_PLATFORM_INIT_FAILED;
        return result;
    }
    BuzzerSetEnable(1u);
    if (SubBoardMusicStart() != 0) {
        result = ARBATOS_RUNTIME_MODULE_TASK_FAILED;
    }
    return result;
#endif

    if (ArbatosPlatformInit() != 0)
    {
        result = ARBATOS_RUNTIME_PLATFORM_INIT_FAILED;
        return result;
    }

    if (RobotControlBootstrapProfileDefaults() == 0u) {
        result = ARBATOS_RUNTIME_CONTROL_CONFIG_FAILED;
        return result;
    }
    ManualInputInit();
    BuzzerSetEnable(1u);

#if ROBOT_TASK_BUILD_CALIBRATION
    cali_param_init();
#endif

#if ROBOT_TASK_BUILD_HOST_LINK
    /* HostLink/VisionLink uses USB CDC even when StartupServiceTask is absent. */
    BspUsbDeviceInit();
#endif

    ArbDefaultTaskHandle = ARB_THREAD_CREATE(defaultTask, ArbDefaultTask);
    if (ArbDefaultTaskHandle == NULL)
    {
        result = ARBATOS_RUNTIME_DEFAULT_TASK_FAILED;
        return result;
    }
    if (ArbCreateModuleTasks() != 0u)
    {
        result = ARBATOS_RUNTIME_MODULE_TASK_FAILED;
        return result;
    }

#if defined(CONFIG_ARBATOS_SUBBOARD_MUSIC)
    if (SubBoardMusicStart() != 0) {
        result = ARBATOS_RUNTIME_MODULE_TASK_FAILED;
    }
#endif

    return result;
}

const char *ArbatosRuntimeStatusName(ArbatosRuntimeStatus status)
{
    switch (status)
    {
        case ARBATOS_RUNTIME_OK:
            return "ok";
        case ARBATOS_RUNTIME_ALREADY_STARTED:
            return "already-started";
        case ARBATOS_RUNTIME_PLATFORM_INIT_FAILED:
            return "platform-init-failed";
        case ARBATOS_RUNTIME_DEFAULT_TASK_FAILED:
            return "default-task-failed";
        case ARBATOS_RUNTIME_MODULE_TASK_FAILED:
            return "module-task-failed";
        case ARBATOS_RUNTIME_CONTROL_CONFIG_FAILED:
            return "control-config-failed";
        default:
            return "unknown";
    }
}
