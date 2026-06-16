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
#include "timers.h"
#include "main.h"
#include "cmsis_os2.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include "config.h"
#include "robot_task_build_config.h"
#if ROBOT_TASK_BUILD_CALIBRATION
#include "calibrate_task.h"
#endif
#if ROBOT_TASK_BUILD_CAN_COMMAND_TX
#include "can_command_tx_task.h"
#endif
#if ROBOT_TASK_BUILD_CAN_FEEDBACK_RX
#include "can_feedback_rx_task.h"
#endif
#if ROBOT_TASK_BUILD_CLASSIC_CHASSIS
#include "chassis_control_task.h"
#endif
#if ROBOT_TASK_BUILD_HEALTH_MONITOR
#include "detect_task.h"
#endif
#if ROBOT_TASK_BUILD_ANY_GIMBAL
#include "gimbal_control_task.h"
#endif
#if ROBOT_TASK_BUILD_IMU
#include "INS_task.h"
#endif
#if ROBOT_TASK_BUILD_STATUS_LED
#include "status_led_task.h"
#endif
#if ROBOT_TASK_BUILD_RC_SBUS
#include "rc_sbus_task.h"
#endif
#if ROBOT_TASK_BUILD_REFEREE_RX
#include "referee_rx_task.h"
#endif
#if ROBOT_TASK_BUILD_HOST_LINK
#include "host_link_task.h"
#endif
#if ROBOT_TASK_BUILD_ELRS_LINK
#include "elrs_task.h"
#endif
#if ROBOT_TASK_BUILD_BATTERY_MONITOR
#include "battery_monitor_task.h"
#endif
#if ROBOT_TASK_BUILD_SERVO
#include "servo_control_task.h"
#endif
#if ROBOT_TASK_BUILD_STARTUP_SERVICE
#include "startup_service_task.h"
#endif
#if ROBOT_TASK_BUILD_SDLOG
#include "sdlog_task.h"
#endif
#include "watch.h"
#include "app_task_bootstrap.h"
#include "control_manager.h"
#include "robot_fault_guard.h"
#include "robot_control_registry.h"
#if ROBOT_TASK_BUILD_WHEELLEG_MIT
#include "wheelleg_mit_task.h"
#endif
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
#if ROBOT_TASK_BUILD_CALIBRATION
osThreadId_t calibrate_tast_handle;
#endif
#if ROBOT_TASK_BUILD_CAN_COMMAND_TX
osThreadId_t canCommandTxTaskHandle;
#endif
#if ROBOT_TASK_BUILD_CAN_FEEDBACK_RX
osThreadId_t canFeedbackRxTaskHandle;
#endif
#if ROBOT_TASK_BUILD_CLASSIC_CHASSIS
osThreadId_t chassisControlTaskHandle;
#endif
#if ROBOT_TASK_BUILD_HEALTH_MONITOR
osThreadId_t detect_handle;
#endif
#if ROBOT_TASK_BUILD_ANY_GIMBAL
osThreadId_t gimbalControlTaskHandle;
#endif
#if ROBOT_TASK_BUILD_IMU
osThreadId_t imuTaskHandle;
#endif
#if ROBOT_TASK_BUILD_STATUS_LED
osThreadId_t statusLedTaskHandle;
#endif
#if ROBOT_TASK_BUILD_RC_SBUS
osThreadId_t rc_sbus_task_handle;
#endif
#if ROBOT_TASK_BUILD_REFEREE_RX
osThreadId_t refereeRxTaskHandle;
#endif
#if ROBOT_TASK_BUILD_HOST_LINK
osThreadId_t host_link_task_handle;
#endif
#if ROBOT_TASK_BUILD_ELRS_LINK
osThreadId_t elrs_link_thread_handle;
#endif
#if ROBOT_TASK_BUILD_BATTERY_MONITOR
osThreadId_t batteryMonitorTaskHandle;
#endif
#if ROBOT_TASK_BUILD_SERVO
osThreadId_t servoControlTaskHandle;
#endif
#if ROBOT_TASK_BUILD_SDLOG
osThreadId_t sdlog_task_handle;
#endif
#if ROBOT_TASK_BUILD_WHEELLEG_MIT
osThreadId_t wheellegMitTaskHandle;
#endif


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
#if ROBOT_TASK_BUILD_STARTUP_SERVICE
osThreadId_t startupServiceTaskHandle;
#endif

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

#if ROBOT_TASK_BUILD_STARTUP_SERVICE
APP_STATIC_THREAD(startupServiceTask, startup_service_task, osPriorityNormal, 512);
#endif
#if ROBOT_TASK_BUILD_CALIBRATION
APP_STATIC_THREAD(cali, calibrate_task, osPriorityNormal, 512);
#endif
#if ROBOT_TASK_BUILD_CLASSIC_CHASSIS
APP_STATIC_THREAD(chassisControlTask, chassis_control_task, osPriorityAboveNormal, 512);
#endif
#if ROBOT_TASK_BUILD_WHEELLEG_MIT
APP_STATIC_THREAD(wheellegMitTask, wheelleg_mit_task, osPriorityAboveNormal, 768);
#endif
#if ROBOT_TASK_BUILD_CAN_COMMAND_TX
APP_STATIC_THREAD(canCommandTxTask, can_command_tx_task, osPriorityAboveNormal, 384);
#endif
#if ROBOT_TASK_BUILD_CAN_FEEDBACK_RX
APP_STATIC_THREAD(canFeedbackRxTask, can_feedback_rx_task, osPriorityHigh, 256);
#endif
#if ROBOT_TASK_BUILD_RC_SBUS
APP_STATIC_THREAD(RCSBUS, rc_sbus_task, osPriorityAboveNormal, 256);
#endif
#if ROBOT_TASK_BUILD_HEALTH_MONITOR
APP_STATIC_THREAD(healthMonitorTask, health_monitor_task, osPriorityNormal, 256);
#endif
#if ROBOT_TASK_BUILD_ANY_GIMBAL
APP_STATIC_THREAD(gimbalControlTask, gimbal_control_task, osPriorityHigh, 1024);
#endif
#if ROBOT_TASK_BUILD_IMU
APP_STATIC_THREAD(imuFusionTask, imu_fusion_task, osPriorityRealtime, 1024);
#endif
#if ROBOT_TASK_BUILD_STATUS_LED
APP_STATIC_THREAD(statusLedTask, status_led_task, osPriorityNormal, 256);
#endif
#if ROBOT_TASK_BUILD_REFEREE_RX
APP_STATIC_THREAD(refereeRxTask, referee_rx_task, osPriorityNormal, 128);
#endif
// HostLinkTask now also owns AUX image-remote parsing and manual-input updates.
// 128 words (512B) is too tight once image traffic starts flowing.
#if ROBOT_TASK_BUILD_HOST_LINK
APP_STATIC_THREAD(HostLinkTask, host_link_task, osPriorityNormal, 512);
#endif
#if ROBOT_TASK_BUILD_ELRS_LINK
APP_STATIC_THREAD(ELRS_LINK, elrs_link_task, osPriorityAboveNormal, 256);
#endif
#if ROBOT_TASK_BUILD_BATTERY_MONITOR
APP_STATIC_THREAD(batteryMonitorTask, battery_monitor_task, osPriorityNormal, 128);
#endif
#if ROBOT_TASK_BUILD_SERVO
APP_STATIC_THREAD(servoControlTask, servo_control_task, osPriorityNormal, 128);
#endif
#if ROBOT_TASK_BUILD_SDLOG
APP_STATIC_THREAD(SDLOG, sdlog_task, osPriorityLow, 512);
#endif

#define APP_TASK_MATCH_HANDLE(task_, handle_, name_) \
  do { \
    if ((handle_) != NULL && (task_) == (TaskHandle_t)(handle_)) \
    { \
      return (name_); \
    } \
  } while (0)

static const char *app_task_name_from_handle(TaskHandle_t task)
{
  if (task == NULL)
  {
    return "NULL";
  }

#if ROBOT_TASK_BUILD_STARTUP_SERVICE
  APP_TASK_MATCH_HANDLE(task, startupServiceTaskHandle, "startupServiceTask");
#endif
#if ROBOT_TASK_BUILD_CALIBRATION
  APP_TASK_MATCH_HANDLE(task, calibrate_tast_handle, "cali");
#endif
#if ROBOT_TASK_BUILD_CLASSIC_CHASSIS
  APP_TASK_MATCH_HANDLE(task, chassisControlTaskHandle, "chassisControlTask");
#endif
#if ROBOT_TASK_BUILD_WHEELLEG_MIT
  APP_TASK_MATCH_HANDLE(task, wheellegMitTaskHandle, "wheellegMitTask");
#endif
#if ROBOT_TASK_BUILD_CAN_COMMAND_TX
  APP_TASK_MATCH_HANDLE(task, canCommandTxTaskHandle, "canCommandTxTask");
#endif
#if ROBOT_TASK_BUILD_CAN_FEEDBACK_RX
  APP_TASK_MATCH_HANDLE(task, canFeedbackRxTaskHandle, "canFeedbackRxTask");
#endif
#if ROBOT_TASK_BUILD_RC_SBUS
  APP_TASK_MATCH_HANDLE(task, rc_sbus_task_handle, "RCSBUS");
#endif
#if ROBOT_TASK_BUILD_HEALTH_MONITOR
  APP_TASK_MATCH_HANDLE(task, detect_handle, "healthMonitorTask");
#endif
#if ROBOT_TASK_BUILD_ANY_GIMBAL
  APP_TASK_MATCH_HANDLE(task, gimbalControlTaskHandle, "gimbalControlTask");
#endif
#if ROBOT_TASK_BUILD_IMU
  APP_TASK_MATCH_HANDLE(task, imuTaskHandle, "imuFusionTask");
#endif
#if ROBOT_TASK_BUILD_STATUS_LED
  APP_TASK_MATCH_HANDLE(task, statusLedTaskHandle, "statusLedTask");
#endif
#if ROBOT_TASK_BUILD_REFEREE_RX
  APP_TASK_MATCH_HANDLE(task, refereeRxTaskHandle, "refereeRxTask");
#endif
#if ROBOT_TASK_BUILD_HOST_LINK
  APP_TASK_MATCH_HANDLE(task, host_link_task_handle, "HostLinkTask");
#endif
#if ROBOT_TASK_BUILD_ELRS_LINK
  APP_TASK_MATCH_HANDLE(task, elrs_link_thread_handle, "ELRS_LINK");
#endif
#if ROBOT_TASK_BUILD_BATTERY_MONITOR
  APP_TASK_MATCH_HANDLE(task, batteryMonitorTaskHandle, "batteryMonitorTask");
#endif
#if ROBOT_TASK_BUILD_SERVO
  APP_TASK_MATCH_HANDLE(task, servoControlTaskHandle, "servoControlTask");
#endif
#if ROBOT_TASK_BUILD_SDLOG
  APP_TASK_MATCH_HANDLE(task, sdlog_task_handle, "SDLOG");
#endif

  if (task == xTaskGetIdleTaskHandle())
  {
    return "IDLE";
  }

  if (task == xTimerGetTimerDaemonTaskHandle())
  {
    return "TIMER";
  }

  return "UNKNOWN";
}

#undef APP_TASK_MATCH_HANDLE

#if ROBOT_TASK_BUILD_STARTUP_SERVICE
static osThreadId_t app_create_startup_service_task(void)
{
  return APP_STATIC_THREAD_CREATE(startupServiceTask, startup_service_task);
}
#endif

#if ROBOT_TASK_BUILD_CALIBRATION
static osThreadId_t app_create_calibration_task(void)
{
  return APP_STATIC_THREAD_CREATE(cali, calibrate_task);
}
#endif

#if ROBOT_TASK_BUILD_CLASSIC_CHASSIS
static osThreadId_t app_create_chassis_control_task(void)
{
  return APP_STATIC_THREAD_CREATE(chassisControlTask, chassis_control_task);
}
#endif

#if ROBOT_TASK_BUILD_WHEELLEG_MIT
static osThreadId_t app_create_wheelleg_mit_task(void)
{
  return APP_STATIC_THREAD_CREATE(wheellegMitTask, wheelleg_mit_task);
}
#endif

#if ROBOT_TASK_BUILD_CAN_COMMAND_TX
static osThreadId_t app_create_can_command_tx_task(void)
{
  return APP_STATIC_THREAD_CREATE(canCommandTxTask, can_command_tx_task);
}
#endif

#if ROBOT_TASK_BUILD_CAN_FEEDBACK_RX
static osThreadId_t app_create_can_feedback_rx_task(void)
{
  return APP_STATIC_THREAD_CREATE(canFeedbackRxTask, can_feedback_rx_task);
}
#endif

#if ROBOT_TASK_BUILD_RC_SBUS
static osThreadId_t app_create_rc_sbus_task(void)
{
  return APP_STATIC_THREAD_CREATE(RCSBUS, rc_sbus_task);
}
#endif

#if ROBOT_TASK_BUILD_HEALTH_MONITOR
static osThreadId_t app_create_health_monitor_task(void)
{
  return APP_STATIC_THREAD_CREATE(healthMonitorTask, health_monitor_task);
}
#endif

#if ROBOT_TASK_BUILD_SINGLE_GIMBAL
static osThreadId_t app_create_single_gimbal_task(void)
{
  return APP_STATIC_THREAD_CREATE(gimbalControlTask, gimbal_control_task);
}
#endif

#if ROBOT_TASK_BUILD_DUAL_YAW_GIMBAL
static osThreadId_t app_create_dual_yaw_gimbal_task(void)
{
  return APP_STATIC_THREAD_CREATE(gimbalControlTask, dual_yaw_gimbal_control_task);
}
#endif

#if ROBOT_TASK_BUILD_IMU
static osThreadId_t app_create_imu_task(void)
{
  return APP_STATIC_THREAD_CREATE(imuFusionTask, imu_fusion_task);
}
#endif

#if ROBOT_TASK_BUILD_STATUS_LED
static osThreadId_t app_create_status_led_task(void)
{
  return APP_STATIC_THREAD_CREATE(statusLedTask, status_led_task);
}
#endif

#if ROBOT_TASK_BUILD_REFEREE_RX
static osThreadId_t app_create_referee_rx_task(void)
{
  return APP_STATIC_THREAD_CREATE(refereeRxTask, referee_rx_task);
}
#endif

#if ROBOT_TASK_BUILD_HOST_LINK
static osThreadId_t app_create_host_link_task(void)
{
  return APP_STATIC_THREAD_CREATE(HostLinkTask, host_link_task);
}
#endif

#if ROBOT_TASK_BUILD_ELRS_LINK
static osThreadId_t app_create_elrs_link_task(void)
{
  return APP_STATIC_THREAD_CREATE(ELRS_LINK, elrs_link_task);
}
#endif

#if ROBOT_TASK_BUILD_BATTERY_MONITOR
static osThreadId_t app_create_battery_monitor_task(void)
{
  return APP_STATIC_THREAD_CREATE(batteryMonitorTask, battery_monitor_task);
}
#endif

#if ROBOT_TASK_BUILD_SERVO
static osThreadId_t app_create_servo_control_task(void)
{
  return APP_STATIC_THREAD_CREATE(servoControlTask, servo_control_task);
}
#endif

#if ROBOT_TASK_BUILD_SDLOG
static osThreadId_t app_create_sdlog_task(void)
{
  return APP_STATIC_THREAD_CREATE(SDLOG, sdlog_task);
}
#endif

static void app_create_module_tasks(void)
{
  static const app_task_module_desc_t module_tasks[] =
  {
#if ROBOT_TASK_BUILD_STARTUP_SERVICE
    {ROBOT_TASK_MODULE_STARTUP_SERVICE, &startupServiceTaskHandle, app_create_startup_service_task},
#endif
#if ROBOT_TASK_BUILD_CALIBRATION
    {ROBOT_TASK_MODULE_CALIBRATION, &calibrate_tast_handle, app_create_calibration_task},
#endif
#if ROBOT_TASK_BUILD_CLASSIC_CHASSIS
    {ROBOT_TASK_MODULE_CLASSIC_CHASSIS, &chassisControlTaskHandle, app_create_chassis_control_task},
#endif
#if ROBOT_TASK_BUILD_WHEELLEG_MIT
    {ROBOT_TASK_MODULE_WHEELLEG_MIT, &wheellegMitTaskHandle, app_create_wheelleg_mit_task},
#endif
#if ROBOT_TASK_BUILD_CAN_COMMAND_TX
    {ROBOT_TASK_MODULE_CAN_COMMAND_TX, &canCommandTxTaskHandle, app_create_can_command_tx_task},
#endif
#if ROBOT_TASK_BUILD_CAN_FEEDBACK_RX
    {ROBOT_TASK_MODULE_CAN_FEEDBACK_RX, &canFeedbackRxTaskHandle, app_create_can_feedback_rx_task},
#endif
#if ROBOT_TASK_BUILD_RC_SBUS
    {ROBOT_TASK_MODULE_RC_SBUS, &rc_sbus_task_handle, app_create_rc_sbus_task},
#endif
#if ROBOT_TASK_BUILD_HEALTH_MONITOR
    {ROBOT_TASK_MODULE_HEALTH_MONITOR, &detect_handle, app_create_health_monitor_task},
#endif
#if ROBOT_TASK_BUILD_SINGLE_GIMBAL
    {ROBOT_TASK_MODULE_SINGLE_GIMBAL, &gimbalControlTaskHandle, app_create_single_gimbal_task},
#endif
#if ROBOT_TASK_BUILD_DUAL_YAW_GIMBAL
    {ROBOT_TASK_MODULE_DUAL_YAW_GIMBAL, &gimbalControlTaskHandle, app_create_dual_yaw_gimbal_task},
#endif
#if ROBOT_TASK_BUILD_IMU
    {ROBOT_TASK_MODULE_IMU, &imuTaskHandle, app_create_imu_task},
#endif
#if ROBOT_TASK_BUILD_STATUS_LED
    {ROBOT_TASK_MODULE_STATUS_LED, &statusLedTaskHandle, app_create_status_led_task},
#endif
#if ROBOT_TASK_BUILD_REFEREE_RX
    {ROBOT_TASK_MODULE_REFEREE_RX, &refereeRxTaskHandle, app_create_referee_rx_task},
#endif
#if ROBOT_TASK_BUILD_HOST_LINK
    {ROBOT_TASK_MODULE_HOST_LINK, &host_link_task_handle, app_create_host_link_task},
#endif
#if ROBOT_TASK_BUILD_ELRS_LINK
    {ROBOT_TASK_MODULE_ELRS_LINK, &elrs_link_thread_handle, app_create_elrs_link_task},
#endif
#if ROBOT_TASK_BUILD_BATTERY_MONITOR
    {ROBOT_TASK_MODULE_BATTERY_MONITOR, &batteryMonitorTaskHandle, app_create_battery_monitor_task},
#endif
#if ROBOT_TASK_BUILD_SERVO
    {ROBOT_TASK_MODULE_SERVO, &servoControlTaskHandle, app_create_servo_control_task},
#endif
#if ROBOT_TASK_BUILD_SDLOG
    {ROBOT_TASK_MODULE_SDLOG, &sdlog_task_handle, app_create_sdlog_task},
#endif
  };

  app_create_enabled_module_tasks(module_tasks, (uint32_t)(sizeof(module_tasks) / sizeof(module_tasks[0])));
}

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
  robot_control_bootstrap_profile_defaults();

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

  /* USER CODE BEGIN RTOS_THREADS */
  app_create_module_tasks();
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
  (void)pcTaskName;

  robot_fault_enter_safe_state_ex((uint32_t)ROBOT_FAULT_REASON_STACK_OVERFLOW,
                                  0u,
                                  0u,
                                  (uint32_t)xTask,
                                  app_task_name_from_handle(xTask));
  robot_fault_halt_forever();
}

void vApplicationMallocFailedHook(void)
{
  TaskHandle_t current_task = xTaskGetCurrentTaskHandle();
  robot_fault_enter_safe_state_ex((uint32_t)ROBOT_FAULT_REASON_MALLOC_FAILED,
                                  0u,
                                  0u,
                                  (uint32_t)current_task,
                                  pcTaskGetTaskName(NULL));
  robot_fault_halt_forever();
}

/* USER CODE END Application */
