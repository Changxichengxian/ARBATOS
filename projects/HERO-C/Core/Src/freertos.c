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

#include "RobotConfig.h"
#include "RobotTaskBuildConfig.h"
#if ROBOT_TASK_BUILD_CALIBRATION
#include "CalibrateTask.h"
#endif
#if ROBOT_TASK_BUILD_CAN_COMMAND_TX
#include "CanTxTask.h"
#endif
#if ROBOT_TASK_BUILD_CAN_FEEDBACK_RX
#include "CanRxTask.h"
#endif
#if ROBOT_TASK_BUILD_CLASSIC_CHASSIS
#include "ChassisControlTask.h"
#endif
#if ROBOT_TASK_BUILD_HEALTH_MONITOR
#include "DetectTask.h"
#endif
#if ROBOT_TASK_BUILD_ANY_GIMBAL
#include "GimbalControlTask.h"
#endif
#if ROBOT_TASK_BUILD_IMU
#include "InsTask.h"
#endif
#if ROBOT_TASK_BUILD_STATUS_LED
#include "StatusLedTask.h"
#endif
#if ROBOT_TASK_BUILD_RC_SBUS
#include "RcSbusTask.h"
#endif
#if ROBOT_TASK_BUILD_REFEREE_RX
#include "RefereeRxTask.h"
#endif
#if ROBOT_TASK_BUILD_HOST_LINK
#include "HostLinkTask.h"
#endif
#if ROBOT_TASK_BUILD_ELRS_LINK
#include "ElrsTask.h"
#endif
#if ROBOT_TASK_BUILD_BATTERY_MONITOR
#include "BatteryMonitorTask.h"
#endif
#if ROBOT_TASK_BUILD_SERVO
#include "ServoControlTask.h"
#endif
#if ROBOT_TASK_BUILD_STARTUP_SERVICE
#include "StartupServiceTask.h"
#endif
#if ROBOT_TASK_BUILD_SDLOG
#include "SdLogTask.h"
#endif
#include "Watch.h"
#include "AppTaskBootstrap.h"
#include "ControlMgr.h"
#include "RobotFaultGuard.h"
#include "RobotControlRegistry.h"
#if ROBOT_TASK_BUILD_WHEELLEG_MIT
#include "WheelLegMitTask.h"
#endif
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
#if ROBOT_TASK_BUILD_CALIBRATION
osThreadId_t CalibrateTastHandle;
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
osThreadId_t DetectHandle;
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
osThreadId_t RcSbusTaskHandle;
#endif
#if ROBOT_TASK_BUILD_REFEREE_RX
osThreadId_t refereeRxTaskHandle;
#endif
#if ROBOT_TASK_BUILD_HOST_LINK
osThreadId_t HostLinkTaskHandle;
#endif
#if ROBOT_TASK_BUILD_ELRS_LINK
osThreadId_t ElrsLinkThreadHandle;
#endif
#if ROBOT_TASK_BUILD_BATTERY_MONITOR
osThreadId_t batteryMonitorTaskHandle;
#endif
#if ROBOT_TASK_BUILD_SERVO
osThreadId_t servoControlTaskHandle;
#endif
#if ROBOT_TASK_BUILD_SDLOG
osThreadId_t SdLogTask_handle;
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
APP_STATIC_THREAD(startupServiceTask, StartupServiceTask, osPriorityNormal, 512);
#endif
#if ROBOT_TASK_BUILD_CALIBRATION
APP_STATIC_THREAD(cali, CalibrateTask, osPriorityNormal, 512);
#endif
#if ROBOT_TASK_BUILD_CLASSIC_CHASSIS
APP_STATIC_THREAD(chassisControlTask, ChassisControlTask, osPriorityAboveNormal, 768);
#endif
#if ROBOT_TASK_BUILD_WHEELLEG_MIT
APP_STATIC_THREAD(wheellegMitTask, WheelLegMitTask, osPriorityAboveNormal, 768);
#endif
#if ROBOT_TASK_BUILD_CAN_COMMAND_TX
APP_STATIC_THREAD(canCommandTxTask, CanTxTask, osPriorityAboveNormal, 256);
#endif
#if ROBOT_TASK_BUILD_CAN_FEEDBACK_RX
APP_STATIC_THREAD(canFeedbackRxTask, CanRxTask, osPriorityHigh, 256);
#endif
#if ROBOT_TASK_BUILD_RC_SBUS
APP_STATIC_THREAD(RCSBUS, RcSbusTask, osPriorityAboveNormal, 256);
#endif
#if ROBOT_TASK_BUILD_HEALTH_MONITOR
APP_STATIC_THREAD(healthMonitorTask, HealthMonitorTask, osPriorityNormal, 256);
#endif
#if ROBOT_TASK_BUILD_ANY_GIMBAL
APP_STATIC_THREAD(gimbalControlTask, GimbalControlTask, osPriorityHigh, 1024);
#endif
#if ROBOT_TASK_BUILD_IMU
APP_STATIC_THREAD(imuFusionTask, ImuFusionTask, osPriorityRealtime, 1024);
#endif
#if ROBOT_TASK_BUILD_STATUS_LED
APP_STATIC_THREAD(statusLedTask, StatusLedTask, osPriorityNormal, 256);
#endif
#if ROBOT_TASK_BUILD_REFEREE_RX
APP_STATIC_THREAD(refereeRxTask, RefereeRxTask, osPriorityNormal, 128);
#endif
// HostLinkTask now also owns AUX image-remote parsing and manual-input updates.
// 128 words (512B) is too tight once image traffic starts flowing.
#if ROBOT_TASK_BUILD_HOST_LINK
APP_STATIC_THREAD(HostLinkTask, HostLinkTask, osPriorityNormal, 512);
#endif
#if ROBOT_TASK_BUILD_ELRS_LINK
APP_STATIC_THREAD(ELRS_LINK, ElrsLinkTask, osPriorityAboveNormal, 256);
#endif
#if ROBOT_TASK_BUILD_BATTERY_MONITOR
APP_STATIC_THREAD(batteryMonitorTask, BatteryMonitorTask, osPriorityNormal, 128);
#endif
#if ROBOT_TASK_BUILD_SERVO
APP_STATIC_THREAD(servoControlTask, ServoControlTask, osPriorityNormal, 192);
#endif
#if ROBOT_TASK_BUILD_SDLOG
APP_STATIC_THREAD(SDLOG, SdLogTask, osPriorityLow, 512);
#endif

#if ROBOT_TASK_BUILD_STARTUP_SERVICE
static osThreadId_t AppCreateStartupServiceTask(void)
{
  return APP_STATIC_THREAD_CREATE(startupServiceTask, StartupServiceTask);
}
#endif

#if ROBOT_TASK_BUILD_CALIBRATION
static osThreadId_t AppCreateCalibrationTask(void)
{
  return APP_STATIC_THREAD_CREATE(cali, CalibrateTask);
}
#endif

#if ROBOT_TASK_BUILD_CLASSIC_CHASSIS
static osThreadId_t AppCreateChassisControlTask(void)
{
  return APP_STATIC_THREAD_CREATE(chassisControlTask, ChassisControlTask);
}
#endif

#if ROBOT_TASK_BUILD_WHEELLEG_MIT
static osThreadId_t AppCreateWheelLegMitTask(void)
{
  return APP_STATIC_THREAD_CREATE(wheellegMitTask, WheelLegMitTask);
}
#endif

#if ROBOT_TASK_BUILD_CAN_COMMAND_TX
static osThreadId_t AppCreateCanTxTask(void)
{
  return APP_STATIC_THREAD_CREATE(canCommandTxTask, CanTxTask);
}
#endif

#if ROBOT_TASK_BUILD_CAN_FEEDBACK_RX
static osThreadId_t AppCreateCanRxTask(void)
{
  return APP_STATIC_THREAD_CREATE(canFeedbackRxTask, CanRxTask);
}
#endif

#if ROBOT_TASK_BUILD_RC_SBUS
static osThreadId_t AppCreateRcSbusTask(void)
{
  return APP_STATIC_THREAD_CREATE(RCSBUS, RcSbusTask);
}
#endif

#if ROBOT_TASK_BUILD_HEALTH_MONITOR
static osThreadId_t AppCreateHealthMonitorTask(void)
{
  return APP_STATIC_THREAD_CREATE(healthMonitorTask, HealthMonitorTask);
}
#endif

#if ROBOT_TASK_BUILD_SINGLE_GIMBAL
static osThreadId_t AppCreateSingleGimbalTask(void)
{
  return APP_STATIC_THREAD_CREATE(gimbalControlTask, GimbalControlTask);
}
#endif

#if ROBOT_TASK_BUILD_DUAL_YAW_GIMBAL
static osThreadId_t AppCreateDualYawGimbalTask(void)
{
  return APP_STATIC_THREAD_CREATE(gimbalControlTask, DualYawGimbalControlTask);
}
#endif

#if ROBOT_TASK_BUILD_IMU
static osThreadId_t AppCreateImuTask(void)
{
  return APP_STATIC_THREAD_CREATE(imuFusionTask, ImuFusionTask);
}
#endif

#if ROBOT_TASK_BUILD_STATUS_LED
static osThreadId_t AppCreateStatusLedTask(void)
{
  return APP_STATIC_THREAD_CREATE(statusLedTask, StatusLedTask);
}
#endif

#if ROBOT_TASK_BUILD_REFEREE_RX
static osThreadId_t AppCreateRefereeRxTask(void)
{
  return APP_STATIC_THREAD_CREATE(refereeRxTask, RefereeRxTask);
}
#endif

#if ROBOT_TASK_BUILD_HOST_LINK
static osThreadId_t AppCreateHostLinkTask(void)
{
  return APP_STATIC_THREAD_CREATE(HostLinkTask, HostLinkTask);
}
#endif

#if ROBOT_TASK_BUILD_ELRS_LINK
static osThreadId_t AppCreateElrsLinkTask(void)
{
  return APP_STATIC_THREAD_CREATE(ELRS_LINK, ElrsLinkTask);
}
#endif

#if ROBOT_TASK_BUILD_BATTERY_MONITOR
static osThreadId_t AppCreateBatteryMonitorTask(void)
{
  return APP_STATIC_THREAD_CREATE(batteryMonitorTask, BatteryMonitorTask);
}
#endif

#if ROBOT_TASK_BUILD_SERVO
static osThreadId_t AppCreateServoControlTask(void)
{
  return APP_STATIC_THREAD_CREATE(servoControlTask, ServoControlTask);
}
#endif

#if ROBOT_TASK_BUILD_SDLOG
static osThreadId_t AppCreateSdLogTask(void)
{
  return APP_STATIC_THREAD_CREATE(SDLOG, SdLogTask);
}
#endif

static void AppCreateModuleTasks(void)
{
  static const AppTaskModuleDesc module_tasks[] =
  {
#if ROBOT_TASK_BUILD_STARTUP_SERVICE
    {ROBOT_TASK_MODULE_STARTUP_SERVICE, &startupServiceTaskHandle, AppCreateStartupServiceTask},
#endif
#if ROBOT_TASK_BUILD_CALIBRATION
    {ROBOT_TASK_MODULE_CALIBRATION, &CalibrateTastHandle, AppCreateCalibrationTask},
#endif
#if ROBOT_TASK_BUILD_CLASSIC_CHASSIS
    {ROBOT_TASK_MODULE_CLASSIC_CHASSIS, &chassisControlTaskHandle, AppCreateChassisControlTask},
#endif
#if ROBOT_TASK_BUILD_WHEELLEG_MIT
    {ROBOT_TASK_MODULE_WHEELLEG_MIT, &wheellegMitTaskHandle, AppCreateWheelLegMitTask},
#endif
#if ROBOT_TASK_BUILD_CAN_COMMAND_TX
    {ROBOT_TASK_MODULE_CAN_COMMAND_TX, &canCommandTxTaskHandle, AppCreateCanTxTask},
#endif
#if ROBOT_TASK_BUILD_CAN_FEEDBACK_RX
    {ROBOT_TASK_MODULE_CAN_FEEDBACK_RX, &canFeedbackRxTaskHandle, AppCreateCanRxTask},
#endif
#if ROBOT_TASK_BUILD_RC_SBUS
    {ROBOT_TASK_MODULE_RC_SBUS, &RcSbusTaskHandle, AppCreateRcSbusTask},
#endif
#if ROBOT_TASK_BUILD_HEALTH_MONITOR
    {ROBOT_TASK_MODULE_HEALTH_MONITOR, &DetectHandle, AppCreateHealthMonitorTask},
#endif
#if ROBOT_TASK_BUILD_SINGLE_GIMBAL
    {ROBOT_TASK_MODULE_SINGLE_GIMBAL, &gimbalControlTaskHandle, AppCreateSingleGimbalTask},
#endif
#if ROBOT_TASK_BUILD_DUAL_YAW_GIMBAL
    {ROBOT_TASK_MODULE_DUAL_YAW_GIMBAL, &gimbalControlTaskHandle, AppCreateDualYawGimbalTask},
#endif
#if ROBOT_TASK_BUILD_IMU
    {ROBOT_TASK_MODULE_IMU, &imuTaskHandle, AppCreateImuTask},
#endif
#if ROBOT_TASK_BUILD_STATUS_LED
    {ROBOT_TASK_MODULE_STATUS_LED, &statusLedTaskHandle, AppCreateStatusLedTask},
#endif
#if ROBOT_TASK_BUILD_REFEREE_RX
    {ROBOT_TASK_MODULE_REFEREE_RX, &refereeRxTaskHandle, AppCreateRefereeRxTask},
#endif
#if ROBOT_TASK_BUILD_HOST_LINK
    {ROBOT_TASK_MODULE_HOST_LINK, &HostLinkTaskHandle, AppCreateHostLinkTask},
#endif
#if ROBOT_TASK_BUILD_ELRS_LINK
    {ROBOT_TASK_MODULE_ELRS_LINK, &ElrsLinkThreadHandle, AppCreateElrsLinkTask},
#endif
#if ROBOT_TASK_BUILD_BATTERY_MONITOR
    {ROBOT_TASK_MODULE_BATTERY_MONITOR, &batteryMonitorTaskHandle, AppCreateBatteryMonitorTask},
#endif
#if ROBOT_TASK_BUILD_SERVO
    {ROBOT_TASK_MODULE_SERVO, &servoControlTaskHandle, AppCreateServoControlTask},
#endif
#if ROBOT_TASK_BUILD_SDLOG
    {ROBOT_TASK_MODULE_SDLOG, &SdLogTask_handle, AppCreateSdLogTask},
#endif
  };

  AppCreateEnabledModuleTasks(module_tasks, (uint32_t)(sizeof(module_tasks) / sizeof(module_tasks[0])));
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
  RobotControlBootstrapProfileDefaults();

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
  AppCreateModuleTasks();
  /* USER CODE END RTOS_THREADS */

}

/* USER CODE BEGIN Header_StartupServiceTask */
/**
  * @brief  Function implementing the startup service thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartupServiceTask */
__weak void StartupServiceTask(void const * argument)
{
  /* init code for USB_DEVICE */
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN StartupServiceTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartupServiceTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
void vApplicationStackOverflowHook(TaskHandle_t xTask, char *pcTaskName)
{
  RobotFaultEnterSafeStateEx((uint32_t)ROBOT_FAULT_REASON_STACK_OVERFLOW,
                                  0u,
                                  0u,
                                  (uint32_t)xTask,
                                  pcTaskName);
  RobotFaultHaltForever();
}

void vApplicationMallocFailedHook(void)
{
  TaskHandle_t current_task = RobotFaultCurrentTaskHandle();
  RobotFaultEnterSafeStateEx((uint32_t)ROBOT_FAULT_REASON_MALLOC_FAILED,
                                  0u,
                                  0u,
                                  (uint32_t)(uintptr_t)current_task,
                                  RobotFaultTaskNameOrUnknown(current_task));
  RobotFaultHaltForever();
}

/* USER CODE END Application */
