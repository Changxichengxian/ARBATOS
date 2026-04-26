/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os2.h"
#include <string.h>

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "rc_sbus_task.h"
#include "detect_task.h"
#include "sdlog_task.h"
#include "can_feedback_rx_task.h"
#include "can_command_tx_task.h"
#include "chassis_control_task.h"
#include "gimbal_control_task.h"
#include "INS_task.h"
#include "host_link_task.h"
#include "elrs_task.h"
#include "watch.h"
#include "robot_task_profile.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
osThreadId_t rcSbusTaskHandle;
osThreadId_t detectTaskHandle;
osThreadId_t sdlogTaskHandle;
osThreadId_t canFeedbackRxTaskHandle;
osThreadId_t canCommandTxTaskHandle;
osThreadId_t chassisControlTaskHandle;
osThreadId_t gimbalControlTaskHandle;
osThreadId_t imuTaskHandle;
osThreadId_t hostLinkTaskHandle;
osThreadId_t elrsLinkTaskHandle;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

/* USER CODE END Variables */
osThreadId_t defaultTaskHandle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
static const char *watch_rtos_task_name_from_handle(TaskHandle_t xTask, const char *fallback_name);
static void watch_rtos_copy_task_name(char *dst, uint32_t dst_size, const char *src);

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);

extern void MX_USB_DEVICE_Init(void);
void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

#define APP_THREAD_ATTR(thread_name, prio, stack_words) \
  static const osThreadAttr_t thread_name##_attr = { \
    .name = #thread_name, \
    .priority = (prio), \
    .stack_size = (stack_words) * sizeof(StackType_t), \
  }

#define APP_THREAD_CREATE(thread_name, entry) \
  osThreadNew((osThreadFunc_t)(entry), NULL, &thread_name##_attr)

APP_THREAD_ATTR(defaultTask, osPriorityNormal, 256);
APP_THREAD_ATTR(rcSbusTask, osPriorityAboveNormal, 256);
APP_THREAD_ATTR(healthMonitorTask, osPriorityNormal, 256);
#if BOARD_SD_ENABLE
APP_THREAD_ATTR(sdlogTask, osPriorityLow, 512);
#endif
APP_THREAD_ATTR(canCommandTxTask, osPriorityAboveNormal, 256);
APP_THREAD_ATTR(canFeedbackRxTask, osPriorityHigh, 256);
APP_THREAD_ATTR(chassisControlTask, osPriorityAboveNormal, 512);
APP_THREAD_ATTR(gimbalControlTask, osPriorityHigh, 1024);
APP_THREAD_ATTR(hostLinkTask, osPriorityNormal, 128);
APP_THREAD_ATTR(elrsLinkTask, osPriorityAboveNormal, 256);
APP_THREAD_ATTR(imuFusionTask, osPriorityRealtime, 1024);

/* GetIdleTaskMemory prototype (linked to static allocation support) */
void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize );

/* GetTimerTaskMemory prototype (linked to static allocation support) */
void vApplicationGetTimerTaskMemory( StaticTask_t **ppxTimerTaskTCBBuffer, StackType_t **ppxTimerTaskStackBuffer, uint32_t *pulTimerTaskStackSize );

/* USER CODE BEGIN GET_IDLE_TASK_MEMORY */
static StaticTask_t xIdleTaskTCBBuffer;
static StackType_t xIdleStack[configMINIMAL_STACK_SIZE];

void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize )
{
  *ppxIdleTaskTCBBuffer = &xIdleTaskTCBBuffer;
  *ppxIdleTaskStackBuffer = &xIdleStack[0];
  *pulIdleTaskStackSize = configMINIMAL_STACK_SIZE;
  /* place for user code */
}
/* USER CODE END GET_IDLE_TASK_MEMORY */

/* USER CODE BEGIN GET_TIMER_TASK_MEMORY */
static StaticTask_t xTimerTaskTCBBuffer;
static StackType_t xTimerStack[configTIMER_TASK_STACK_DEPTH];

void vApplicationGetTimerTaskMemory( StaticTask_t **ppxTimerTaskTCBBuffer, StackType_t **ppxTimerTaskStackBuffer, uint32_t *pulTimerTaskStackSize )
{
  *ppxTimerTaskTCBBuffer = &xTimerTaskTCBBuffer;
  *ppxTimerTaskStackBuffer = &xTimerStack[0];
  *pulTimerTaskStackSize = configTIMER_TASK_STACK_DEPTH;
  /* place for user code */
}
/* USER CODE END GET_TIMER_TASK_MEMORY */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  defaultTaskHandle = APP_THREAD_CREATE(defaultTask, StartDefaultTask);

  /* USER CODE BEGIN RTOS_THREADS */
  rcSbusTaskHandle = APP_THREAD_CREATE(rcSbusTask, rc_sbus_task);

  detectTaskHandle = APP_THREAD_CREATE(healthMonitorTask, health_monitor_task);

#if BOARD_SD_ENABLE
  sdlogTaskHandle = APP_THREAD_CREATE(sdlogTask, sdlog_task);
#endif

  canCommandTxTaskHandle = APP_THREAD_CREATE(canCommandTxTask, can_command_tx_task);

  canFeedbackRxTaskHandle = APP_THREAD_CREATE(canFeedbackRxTask, can_feedback_rx_task);

  if (robot_profile_need_classic_chassis_control_task())
  {
    chassisControlTaskHandle = APP_THREAD_CREATE(chassisControlTask, chassis_control_task);
  }
  else
  {
    chassisControlTaskHandle = NULL;
  }

  if (robot_profile_need_single_gimbal_control_task())
  {
    gimbalControlTaskHandle = APP_THREAD_CREATE(gimbalControlTask, gimbal_control_task);
  }
  else
  {
    gimbalControlTaskHandle = NULL;
  }

  hostLinkTaskHandle = APP_THREAD_CREATE(hostLinkTask, host_link_task);

  elrsLinkTaskHandle = APP_THREAD_CREATE(elrsLinkTask, elrs_link_task);

  imuTaskHandle = APP_THREAD_CREATE(imuFusionTask, imu_fusion_task);
  /* USER CODE END RTOS_THREADS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  (void)argument;
  /* init code for USB_DEVICE */
  watch_diag_set_boot_stage(WATCH_BOOT_STAGE_DEFAULT_TASK_START);
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN StartDefaultTask */
  watch_diag_set_boot_stage(WATCH_BOOT_STAGE_RUN);
  /* Infinite loop */
  for(;;)
  {
    watch_task_beat(WATCH_TASK_DEFAULT);
    osDelay(1);
  }
  /* USER CODE END StartDefaultTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

static const char *watch_rtos_task_name_from_handle(TaskHandle_t xTask, const char *fallback_name)
{
  if (xTask == (TaskHandle_t)defaultTaskHandle)
  {
    return "defaultTask";
  }
  if (xTask == (TaskHandle_t)rcSbusTaskHandle)
  {
    return "rcSbusTask";
  }
  if (xTask == (TaskHandle_t)detectTaskHandle)
  {
    return "healthMonitorTask";
  }
#if BOARD_SD_ENABLE
  if (xTask == (TaskHandle_t)sdlogTaskHandle)
  {
    return "sdlogTask";
  }
#endif
  if (xTask == (TaskHandle_t)canCommandTxTaskHandle)
  {
    return "canCommandTxTask";
  }
  if (xTask == (TaskHandle_t)canFeedbackRxTaskHandle)
  {
    return "canFeedbackRxTask";
  }
  if (xTask == (TaskHandle_t)chassisControlTaskHandle)
  {
    return "chassisControlTask";
  }
  if (xTask == (TaskHandle_t)gimbalControlTaskHandle)
  {
    return "gimbalControlTask";
  }
  if (xTask == (TaskHandle_t)hostLinkTaskHandle)
  {
    return "hostLinkTask";
  }
  if (xTask == (TaskHandle_t)elrsLinkTaskHandle)
  {
    return "elrsLinkTask";
  }
  if (xTask == (TaskHandle_t)imuTaskHandle)
  {
    return "imuFusionTask";
  }

  if (fallback_name != NULL && fallback_name[0] != '\0')
  {
    return fallback_name;
  }

  return "?";
}

static void watch_rtos_copy_task_name(char *dst, uint32_t dst_size, const char *src)
{
  if (dst == NULL || dst_size == 0u)
  {
    return;
  }

  (void)memset(dst, 0, dst_size);
  if (src != NULL)
  {
    (void)strncpy(dst, src, (size_t)dst_size - 1u);
  }
}

void vApplicationStackOverflowHook(TaskHandle_t xTask, char *pcTaskName)
{
  g_watch.rtos.fatal_reason = 1U;
  g_watch.rtos.fatal_task_handle = (uint32_t)xTask;
  watch_rtos_copy_task_name(g_watch.rtos.fatal_task_name,
                                (uint32_t)sizeof(g_watch.rtos.fatal_task_name),
                                watch_rtos_task_name_from_handle(xTask, pcTaskName));

  if ((CoreDebug->DHCSR & CoreDebug_DHCSR_C_DEBUGEN_Msk) != 0U)
  {
    __BKPT(0);
  }

  taskDISABLE_INTERRUPTS();
  for (;;)
  {
    __NOP();
  }
}

void vApplicationMallocFailedHook(void)
{
  const TaskHandle_t current_task = xTaskGetCurrentTaskHandle();

  g_watch.rtos.fatal_reason = 2U;
  g_watch.rtos.fatal_task_handle = (uint32_t)current_task;
  watch_rtos_copy_task_name(g_watch.rtos.fatal_task_name,
                                (uint32_t)sizeof(g_watch.rtos.fatal_task_name),
                                watch_rtos_task_name_from_handle(current_task,
                                                                     pcTaskGetTaskName(NULL)));

  if ((CoreDebug->DHCSR & CoreDebug_DHCSR_C_DEBUGEN_Msk) != 0U)
  {
    __BKPT(0);
  }

  taskDISABLE_INTERRUPTS();
  for (;;)
  {
    __NOP();
  }
}

/* USER CODE END Application */
