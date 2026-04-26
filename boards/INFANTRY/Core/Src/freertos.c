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
#include "cmsis_os.h"
#include <string.h>

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "rc_sbus_task.h"
#include "detect_task.h"
#include "sdlog_task.h"
#include "can_rx_task.h"
#include "can_tx_task.h"
#include "chassis_task.h"
#include "gimbal_task.h"
#include "INS_task.h"
#include "usb_task.h"
#include "elrs_task.h"
#include "app_watch.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
osThreadId rcSbusTaskHandle;
osThreadId detectTaskHandle;
osThreadId sdlogTaskHandle;
osThreadId canRxTaskHandle;
osThreadId canTxTaskHandle;
osThreadId chassisTaskHandle;
osThreadId gimbalTaskHandle;
osThreadId imuTaskHandle;
osThreadId usbTaskHandle;
osThreadId uart1ElrsTaskHandle;

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
osThreadId defaultTaskHandle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
static const char *app_watch_rtos_task_name_from_handle(TaskHandle_t xTask, const char *fallback_name);
static void app_watch_rtos_copy_task_name(char *dst, uint32_t dst_size, const char *src);

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void const * argument);

extern void MX_USB_DEVICE_Init(void);
void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

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
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 256);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  osThreadDef(rcSbusTask, rc_sbus_task, osPriorityAboveNormal, 0, 256);
  rcSbusTaskHandle = osThreadCreate(osThread(rcSbusTask), NULL);

  osThreadDef(detectTask, detect_task, osPriorityNormal, 0, 256);
  detectTaskHandle = osThreadCreate(osThread(detectTask), NULL);

#if BOARD_SD_ENABLE
  osThreadDef(sdlogTask, sdlog_task, osPriorityLow, 0, 512);
  sdlogTaskHandle = osThreadCreate(osThread(sdlogTask), NULL);
#endif

  osThreadDef(canTxTask, can_tx_task, osPriorityAboveNormal, 0, 256);
  canTxTaskHandle = osThreadCreate(osThread(canTxTask), NULL);

  osThreadDef(canRxTask, can_rx_task, osPriorityHigh, 0, 256);
  canRxTaskHandle = osThreadCreate(osThread(canRxTask), NULL);

  osThreadDef(chassisTask, chassis_task, osPriorityAboveNormal, 0, 512);
  chassisTaskHandle = osThreadCreate(osThread(chassisTask), NULL);

  osThreadDef(gimbalTask, gimbal_task, osPriorityHigh, 0, 1024);
  gimbalTaskHandle = osThreadCreate(osThread(gimbalTask), NULL);

  osThreadDef(usbTask, usb_task, osPriorityNormal, 0, 128);
  usbTaskHandle = osThreadCreate(osThread(usbTask), NULL);

  osThreadDef(uart1ElrsTask, uart1_elrs_task, osPriorityAboveNormal, 0, 256);
  uart1ElrsTaskHandle = osThreadCreate(osThread(uart1ElrsTask), NULL);

  osThreadDef(imuTask, INS_task, osPriorityRealtime, 0, 1024);
  imuTaskHandle = osThreadCreate(osThread(imuTask), NULL);
  /* USER CODE END RTOS_THREADS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void const * argument)
{
  (void)argument;
  /* init code for USB_DEVICE */
  app_watch_diag_set_boot_stage(APP_WATCH_BOOT_STAGE_DEFAULT_TASK_START);
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN StartDefaultTask */
  app_watch_diag_set_boot_stage(APP_WATCH_BOOT_STAGE_RUN);
  /* Infinite loop */
  for(;;)
  {
    app_watch_task_beat(APP_WATCH_TASK_DEFAULT);
    osDelay(1);
  }
  /* USER CODE END StartDefaultTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

static const char *app_watch_rtos_task_name_from_handle(TaskHandle_t xTask, const char *fallback_name)
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
    return "detectTask";
  }
#if BOARD_SD_ENABLE
  if (xTask == (TaskHandle_t)sdlogTaskHandle)
  {
    return "sdlogTask";
  }
#endif
  if (xTask == (TaskHandle_t)canTxTaskHandle)
  {
    return "canTxTask";
  }
  if (xTask == (TaskHandle_t)canRxTaskHandle)
  {
    return "canRxTask";
  }
  if (xTask == (TaskHandle_t)chassisTaskHandle)
  {
    return "chassisTask";
  }
  if (xTask == (TaskHandle_t)gimbalTaskHandle)
  {
    return "gimbalTask";
  }
  if (xTask == (TaskHandle_t)usbTaskHandle)
  {
    return "usbTask";
  }
  if (xTask == (TaskHandle_t)uart1ElrsTaskHandle)
  {
    return "uart1ElrsTask";
  }
  if (xTask == (TaskHandle_t)imuTaskHandle)
  {
    return "imuTask";
  }

  if (fallback_name != NULL && fallback_name[0] != '\0')
  {
    return fallback_name;
  }

  return "?";
}

static void app_watch_rtos_copy_task_name(char *dst, uint32_t dst_size, const char *src)
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
  g_app_watch.rtos.fatal_reason = 1U;
  g_app_watch.rtos.fatal_task_handle = (uint32_t)xTask;
  app_watch_rtos_copy_task_name(g_app_watch.rtos.fatal_task_name,
                                (uint32_t)sizeof(g_app_watch.rtos.fatal_task_name),
                                app_watch_rtos_task_name_from_handle(xTask, pcTaskName));

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

  g_app_watch.rtos.fatal_reason = 2U;
  g_app_watch.rtos.fatal_task_handle = (uint32_t)current_task;
  app_watch_rtos_copy_task_name(g_app_watch.rtos.fatal_task_name,
                                (uint32_t)sizeof(g_app_watch.rtos.fatal_task_name),
                                app_watch_rtos_task_name_from_handle(current_task,
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
