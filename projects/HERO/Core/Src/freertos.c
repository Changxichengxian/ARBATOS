/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os2.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include "calibrate_task.h"
#include "can_command_tx_task.h"
#include "can_feedback_rx_task.h"
#include "chassis_control_task.h"
#include "detect_task.h"
#include "gimbal_control_task.h"
#include "INS_task.h"
#include "status_led_task.h"
#include "rc_sbus_task.h"
#include "referee_rx_task.h"
#include "host_link_task.h"
#include "elrs_task.h"
#include "battery_monitor_task.h"
#include "servo_control_task.h"
#include "startup_service_task.h"
#include "sdlog_task.h"
#include "watch.h"
#include "robot_task_profile.h"
#include "control_manager.h"
#include "wheelleg_mit_task.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

osThreadId_t calibrate_tast_handle;
osThreadId_t canCommandTxTaskHandle;
osThreadId_t canFeedbackRxTaskHandle;
osThreadId_t chassisControlTaskHandle;
osThreadId_t detect_handle;
osThreadId_t gimbalControlTaskHandle;
osThreadId_t imuTaskHandle;
osThreadId_t statusLedTaskHandle;
osThreadId_t rc_sbus_task_handle;
osThreadId_t refereeRxTaskHandle;
osThreadId_t host_link_task_handle;
osThreadId_t elrs_link_thread_handle;
osThreadId_t batteryMonitorTaskHandle;
osThreadId_t servoControlTaskHandle;
osThreadId_t sdlog_task_handle;
osThreadId_t wheellegMitTaskHandle;


/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

/* FreeRTOS heap storage.
 * - Placed in CCM RAM to save main SRAM (0x2000_0000) for DMA-capable buffers.
 * - IMPORTANT: CCM is CPU-only; do not use pvPortMalloc() for DMA buffers. */
#if (configAPPLICATION_ALLOCATED_HEAP == 1)
__attribute__((section(".ccmram"))) __attribute__((aligned(8))) uint8_t ucHeap[configTOTAL_HEAP_SIZE];
#endif

/* USER CODE END Variables */
osThreadId_t startupServiceTaskHandle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

extern void MX_USB_DEVICE_Init(void);
void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

#define APP_STATIC_THREAD(thread_id, entry, prio, stack_words) \
  static StaticTask_t thread_id##_tcb; \
  static StackType_t thread_id##_stack[(stack_words)]; \
  static const osThreadAttr_t thread_id##_attr = { \
    .name = #thread_id, \
    .priority = (prio), \
    .stack_mem = thread_id##_stack, \
    .stack_size = sizeof(thread_id##_stack), \
    .cb_mem = &thread_id##_tcb, \
    .cb_size = sizeof(thread_id##_tcb), \
  }

#define APP_STATIC_THREAD_CREATE(thread_id, entry) \
  osThreadNew((osThreadFunc_t)(entry), NULL, &thread_id##_attr)

APP_STATIC_THREAD(startupServiceTask, startup_service_task, osPriorityNormal, 512);
APP_STATIC_THREAD(cali, calibrate_task, osPriorityNormal, 512);
APP_STATIC_THREAD(chassisControlTask, chassis_control_task, osPriorityAboveNormal, 512);
APP_STATIC_THREAD(wheellegMitTask, wheelleg_mit_task, osPriorityAboveNormal, 768);
APP_STATIC_THREAD(canCommandTxTask, can_command_tx_task, osPriorityAboveNormal, 256);
APP_STATIC_THREAD(canFeedbackRxTask, can_feedback_rx_task, osPriorityHigh, 256);
APP_STATIC_THREAD(RCSBUS, rc_sbus_task, osPriorityAboveNormal, 256);
APP_STATIC_THREAD(healthMonitorTask, health_monitor_task, osPriorityNormal, 256);
APP_STATIC_THREAD(gimbalControlTask, gimbal_control_task, osPriorityHigh, 1024);
APP_STATIC_THREAD(imuFusionTask, imu_fusion_task, osPriorityRealtime, 1024);
APP_STATIC_THREAD(statusLedTask, status_led_task, osPriorityNormal, 256);
APP_STATIC_THREAD(refereeRxTask, referee_rx_task, osPriorityNormal, 128);
// HostLinkTask now also owns AUX image-remote parsing and manual-input updates.
// 128 words (512B) is too tight once image traffic starts flowing.
APP_STATIC_THREAD(HostLinkTask, host_link_task, osPriorityNormal, 512);
APP_STATIC_THREAD(ELRS_LINK, elrs_link_task, osPriorityAboveNormal, 256);
APP_STATIC_THREAD(batteryMonitorTask, battery_monitor_task, osPriorityNormal, 128);
APP_STATIC_THREAD(servoControlTask, servo_control_task, osPriorityNormal, 128);
APP_STATIC_THREAD(SDLOG, sdlog_task, osPriorityLow, 512);

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
  control_manager_init();

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
  startupServiceTaskHandle = APP_STATIC_THREAD_CREATE(startupServiceTask, startup_service_task);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
    calibrate_tast_handle = APP_STATIC_THREAD_CREATE(cali, calibrate_task);

    if (robot_profile_need_classic_chassis_control_task())
    {
      chassisControlTaskHandle = APP_STATIC_THREAD_CREATE(chassisControlTask, chassis_control_task);
    }
    else
    {
      chassisControlTaskHandle = NULL;
    }

    if (robot_profile_is_wheelleg_mit())
    {
      wheellegMitTaskHandle = APP_STATIC_THREAD_CREATE(wheellegMitTask, wheelleg_mit_task);
    }
    else
    {
      wheellegMitTaskHandle = NULL;
    }

    canCommandTxTaskHandle = APP_STATIC_THREAD_CREATE(canCommandTxTask, can_command_tx_task);

    // CAN RX dispatcher: consume ISR ring, update motor measures and offline detect.
    canFeedbackRxTaskHandle = APP_STATIC_THREAD_CREATE(canFeedbackRxTask, can_feedback_rx_task);

    // USART3 SBUS/DBUS RX: consume ISR ring, parse frames, update manual_input_state_t.
    rc_sbus_task_handle = APP_STATIC_THREAD_CREATE(RCSBUS, rc_sbus_task);

    detect_handle = APP_STATIC_THREAD_CREATE(healthMonitorTask, health_monitor_task);

    if (robot_profile_need_single_gimbal_control_task())
    {
      gimbalControlTaskHandle = APP_STATIC_THREAD_CREATE(gimbalControlTask, gimbal_control_task);
    }
    else
    {
      gimbalControlTaskHandle = NULL;
    }

    imuTaskHandle = APP_STATIC_THREAD_CREATE(imuFusionTask, imu_fusion_task);

    statusLedTaskHandle = APP_STATIC_THREAD_CREATE(statusLedTask, status_led_task);


    refereeRxTaskHandle = APP_STATIC_THREAD_CREATE(refereeRxTask, referee_rx_task);


    host_link_task_handle = APP_STATIC_THREAD_CREATE(HostLinkTask, host_link_task);

    // ELRS(CRSF) RX on aux link: high-priority, interrupt-driven parser (updates manual_input_state_t).
    elrs_link_thread_handle = APP_STATIC_THREAD_CREATE(ELRS_LINK, elrs_link_task);

    batteryMonitorTaskHandle = APP_STATIC_THREAD_CREATE(batteryMonitorTask, battery_monitor_task);

    servoControlTaskHandle = APP_STATIC_THREAD_CREATE(servoControlTask, servo_control_task);

    // TF/SD logger: low priority, flush buffered binary records to FatFs.
    sdlog_task_handle = APP_STATIC_THREAD_CREATE(SDLOG, sdlog_task);



  /* USER CODE END RTOS_THREADS */

}

/* USER CODE BEGIN Header_startup_service_task */
/**
  * @brief  Function implementing the startup service thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_startup_service_task */
__weak void startup_service_task(void const * argument)
{
  /* init code for USB_DEVICE */
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN startup_service_task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END startup_service_task */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
void vApplicationStackOverflowHook(TaskHandle_t xTask, char *pcTaskName)
{
  (void)xTask;

  g_watch.rtos.fatal_reason = 1U;
  g_watch.rtos.fatal_task_handle = (uint32_t)xTask;
  for (uint32_t i = 0; i < (uint32_t)sizeof(g_watch.rtos.fatal_task_name); i++)
  {
    const char c = (pcTaskName != NULL) ? pcTaskName[i] : '\0';
    g_watch.rtos.fatal_task_name[i] = c;
    if (c == '\0')
    {
      break;
    }
  }
  g_watch.rtos.fatal_task_name[sizeof(g_watch.rtos.fatal_task_name) - 1] = '\0';

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
  g_watch.rtos.fatal_reason = 2U;
  g_watch.rtos.fatal_task_handle = (uint32_t)xTaskGetCurrentTaskHandle();
  {
    const char *name = pcTaskGetTaskName(NULL);
    for (uint32_t i = 0; i < (uint32_t)sizeof(g_watch.rtos.fatal_task_name); i++)
    {
      const char c = (name != NULL) ? name[i] : '\0';
      g_watch.rtos.fatal_task_name[i] = c;
      if (c == '\0')
      {
        break;
      }
    }
    g_watch.rtos.fatal_task_name[sizeof(g_watch.rtos.fatal_task_name) - 1] = '\0';
  }

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
