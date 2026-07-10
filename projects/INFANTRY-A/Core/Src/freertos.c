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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "RobotConfig.h"
#include "RobotTaskBuildConfig.h"
#if ROBOT_TASK_BUILD_RC_SBUS
#include "RcSbusTask.h"
#endif
#if ROBOT_TASK_BUILD_HEALTH_MONITOR
#include "DetectTask.h"
#endif
#if ROBOT_TASK_BUILD_SDLOG && BOARD_SD_ENABLE
#include "SdLogTask.h"
#endif
#if ROBOT_TASK_BUILD_CAN_FEEDBACK_RX
#include "CanRxTask.h"
#endif
#if ROBOT_TASK_BUILD_CAN_COMMAND_TX
#include "CanTxTask.h"
#endif
#if ROBOT_TASK_BUILD_CLASSIC_CHASSIS
#include "ChassisControlTask.h"
#endif
#if ROBOT_TASK_BUILD_ANY_GIMBAL
#include "GimbalControlTask.h"
#endif
#if ROBOT_TASK_BUILD_IMU
#include "InsTask.h"
#endif
#if ROBOT_TASK_BUILD_HOST_LINK
#include "HostLinkTask.h"
#endif
#if ROBOT_TASK_BUILD_ELRS_LINK
#include "ElrsTask.h"
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
#if ROBOT_TASK_BUILD_RC_SBUS
osThreadId_t rcSbusTaskHandle;
#endif
#if ROBOT_TASK_BUILD_HEALTH_MONITOR
osThreadId_t detectTaskHandle;
#endif
#if ROBOT_TASK_BUILD_SDLOG && BOARD_SD_ENABLE
osThreadId_t sdlogTaskHandle;
#endif
#if ROBOT_TASK_BUILD_CAN_FEEDBACK_RX
osThreadId_t canFeedbackRxTaskHandle;
#endif
#if ROBOT_TASK_BUILD_CAN_COMMAND_TX
osThreadId_t canCommandTxTaskHandle;
#endif
#if ROBOT_TASK_BUILD_CLASSIC_CHASSIS
osThreadId_t chassisControlTaskHandle;
#endif
#if ROBOT_TASK_BUILD_WHEELLEG_MIT
osThreadId_t wheellegMitTaskHandle;
#endif
#if ROBOT_TASK_BUILD_ANY_GIMBAL
osThreadId_t gimbalControlTaskHandle;
#endif
#if ROBOT_TASK_BUILD_IMU
osThreadId_t imuTaskHandle;
#endif
#if ROBOT_TASK_BUILD_HOST_LINK
osThreadId_t hostLinkTaskHandle;
#endif
#if ROBOT_TASK_BUILD_ELRS_LINK
osThreadId_t elrsLinkTaskHandle;
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

/* USER CODE END Variables */
osThreadId_t defaultTaskHandle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
static const char *WatchRtosTaskNameFromHandle(TaskHandle_t xTask, const char *fallback_name);

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
#if ROBOT_TASK_BUILD_RC_SBUS
APP_THREAD_ATTR(rcSbusTask, osPriorityAboveNormal, 256);
#endif
#if ROBOT_TASK_BUILD_HEALTH_MONITOR
APP_THREAD_ATTR(healthMonitorTask, osPriorityNormal, 256);
#endif
#if ROBOT_TASK_BUILD_SDLOG && BOARD_SD_ENABLE
APP_THREAD_ATTR(sdlogTask, osPriorityLow, 512);
#endif
#if ROBOT_TASK_BUILD_CAN_COMMAND_TX
APP_THREAD_ATTR(canCommandTxTask, osPriorityAboveNormal, 256);
#endif
#if ROBOT_TASK_BUILD_CAN_FEEDBACK_RX
APP_THREAD_ATTR(canFeedbackRxTask, osPriorityHigh, 256);
#endif
#if ROBOT_TASK_BUILD_CLASSIC_CHASSIS
APP_THREAD_ATTR(chassisControlTask, osPriorityAboveNormal, 512);
#endif
#if ROBOT_TASK_BUILD_WHEELLEG_MIT
APP_THREAD_ATTR(wheellegMitTask, osPriorityAboveNormal, 768);
#endif
#if ROBOT_TASK_BUILD_ANY_GIMBAL
APP_THREAD_ATTR(gimbalControlTask, osPriorityHigh, 1024);
#endif
#if ROBOT_TASK_BUILD_HOST_LINK
APP_THREAD_ATTR(hostLinkTask, osPriorityNormal, 128);
#endif
#if ROBOT_TASK_BUILD_ELRS_LINK
APP_THREAD_ATTR(elrsLinkTask, osPriorityAboveNormal, 256);
#endif
#if ROBOT_TASK_BUILD_IMU
APP_THREAD_ATTR(imuFusionTask, osPriorityRealtime, 1024);
#endif

#if ROBOT_TASK_BUILD_RC_SBUS
static osThreadId_t AppCreateRcSbusTask(void)
{
  return APP_THREAD_CREATE(rcSbusTask, RcSbusTask);
}
#endif

#if ROBOT_TASK_BUILD_HEALTH_MONITOR
static osThreadId_t AppCreateHealthMonitorTask(void)
{
  return APP_THREAD_CREATE(healthMonitorTask, HealthMonitorTask);
}
#endif

#if ROBOT_TASK_BUILD_SDLOG && BOARD_SD_ENABLE
static osThreadId_t AppCreateSdLogTask(void)
{
  return APP_THREAD_CREATE(sdlogTask, SdLogTask);
}
#endif

#if ROBOT_TASK_BUILD_CAN_COMMAND_TX
static osThreadId_t AppCreateCanTxTask(void)
{
  return APP_THREAD_CREATE(canCommandTxTask, CanTxTask);
}
#endif

#if ROBOT_TASK_BUILD_CAN_FEEDBACK_RX
static osThreadId_t AppCreateCanRxTask(void)
{
  return APP_THREAD_CREATE(canFeedbackRxTask, CanRxTask);
}
#endif

#if ROBOT_TASK_BUILD_CLASSIC_CHASSIS
static osThreadId_t AppCreateChassisControlTask(void)
{
  return APP_THREAD_CREATE(chassisControlTask, ChassisControlTask);
}
#endif

#if ROBOT_TASK_BUILD_WHEELLEG_MIT
static osThreadId_t AppCreateWheelLegMitTask(void)
{
  return APP_THREAD_CREATE(wheellegMitTask, WheelLegMitTask);
}
#endif

#if ROBOT_TASK_BUILD_SINGLE_GIMBAL
static osThreadId_t AppCreateSingleGimbalTask(void)
{
  return APP_THREAD_CREATE(gimbalControlTask, GimbalControlTask);
}
#endif

#if ROBOT_TASK_BUILD_DUAL_YAW_GIMBAL
static osThreadId_t AppCreateDualYawGimbalTask(void)
{
  return APP_THREAD_CREATE(gimbalControlTask, DualYawGimbalControlTask);
}
#endif

#if ROBOT_TASK_BUILD_HOST_LINK
static osThreadId_t AppCreateHostLinkTask(void)
{
  return APP_THREAD_CREATE(hostLinkTask, HostLinkTask);
}
#endif

#if ROBOT_TASK_BUILD_ELRS_LINK
static osThreadId_t AppCreateElrsLinkTask(void)
{
  return APP_THREAD_CREATE(elrsLinkTask, ElrsLinkTask);
}
#endif

#if ROBOT_TASK_BUILD_IMU
static osThreadId_t AppCreateImuTask(void)
{
  return APP_THREAD_CREATE(imuFusionTask, ImuFusionTask);
}
#endif

static void AppCreateModuleTasks(void)
{
  static const AppTaskModuleDesc module_tasks[] =
  {
#if ROBOT_TASK_BUILD_RC_SBUS
    {ROBOT_TASK_MODULE_RC_SBUS, &rcSbusTaskHandle, AppCreateRcSbusTask},
#endif
#if ROBOT_TASK_BUILD_HEALTH_MONITOR
    {ROBOT_TASK_MODULE_HEALTH_MONITOR, &detectTaskHandle, AppCreateHealthMonitorTask},
#endif
#if ROBOT_TASK_BUILD_SDLOG && BOARD_SD_ENABLE
    {ROBOT_TASK_MODULE_SDLOG, &sdlogTaskHandle, AppCreateSdLogTask},
#endif
#if ROBOT_TASK_BUILD_CAN_COMMAND_TX
    {ROBOT_TASK_MODULE_CAN_COMMAND_TX, &canCommandTxTaskHandle, AppCreateCanTxTask},
#endif
#if ROBOT_TASK_BUILD_CAN_FEEDBACK_RX
    {ROBOT_TASK_MODULE_CAN_FEEDBACK_RX, &canFeedbackRxTaskHandle, AppCreateCanRxTask},
#endif
#if ROBOT_TASK_BUILD_CLASSIC_CHASSIS
    {ROBOT_TASK_MODULE_CLASSIC_CHASSIS, &chassisControlTaskHandle, AppCreateChassisControlTask},
#endif
#if ROBOT_TASK_BUILD_WHEELLEG_MIT
    {ROBOT_TASK_MODULE_WHEELLEG_MIT, &wheellegMitTaskHandle, AppCreateWheelLegMitTask},
#endif
#if ROBOT_TASK_BUILD_SINGLE_GIMBAL
    {ROBOT_TASK_MODULE_SINGLE_GIMBAL, &gimbalControlTaskHandle, AppCreateSingleGimbalTask},
#endif
#if ROBOT_TASK_BUILD_DUAL_YAW_GIMBAL
    {ROBOT_TASK_MODULE_DUAL_YAW_GIMBAL, &gimbalControlTaskHandle, AppCreateDualYawGimbalTask},
#endif
#if ROBOT_TASK_BUILD_HOST_LINK
    {ROBOT_TASK_MODULE_HOST_LINK, &hostLinkTaskHandle, AppCreateHostLinkTask},
#endif
#if ROBOT_TASK_BUILD_ELRS_LINK
    {ROBOT_TASK_MODULE_ELRS_LINK, &elrsLinkTaskHandle, AppCreateElrsLinkTask},
#endif
#if ROBOT_TASK_BUILD_IMU
    {ROBOT_TASK_MODULE_IMU, &imuTaskHandle, AppCreateImuTask},
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
  defaultTaskHandle = APP_THREAD_CREATE(defaultTask, StartDefaultTask);

  /* USER CODE BEGIN RTOS_THREADS */
  AppCreateModuleTasks();
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
  WatchDiagSetBootStage(WATCH_BOOT_STAGE_DEFAULT_TASK_START);
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN StartDefaultTask */
  WatchDiagSetBootStage(WATCH_BOOT_STAGE_RUN);
  /* Infinite loop */
  for(;;)
  {
    WatchTaskBeat(WATCH_TASK_DEFAULT);
    osDelay(1);
  }
  /* USER CODE END StartDefaultTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

static const char *WatchRtosTaskNameFromHandle(TaskHandle_t xTask, const char *fallback_name)
{
  if (xTask == (TaskHandle_t)defaultTaskHandle)
  {
    return "defaultTask";
  }
#if ROBOT_TASK_BUILD_RC_SBUS
  if (xTask == (TaskHandle_t)rcSbusTaskHandle)
  {
    return "rcSbusTask";
  }
#endif
#if ROBOT_TASK_BUILD_HEALTH_MONITOR
  if (xTask == (TaskHandle_t)detectTaskHandle)
  {
    return "healthMonitorTask";
  }
#endif
#if ROBOT_TASK_BUILD_SDLOG && BOARD_SD_ENABLE
  if (xTask == (TaskHandle_t)sdlogTaskHandle)
  {
    return "sdlogTask";
  }
#endif
#if ROBOT_TASK_BUILD_CAN_COMMAND_TX
  if (xTask == (TaskHandle_t)canCommandTxTaskHandle)
  {
    return "canCommandTxTask";
  }
#endif
#if ROBOT_TASK_BUILD_CAN_FEEDBACK_RX
  if (xTask == (TaskHandle_t)canFeedbackRxTaskHandle)
  {
    return "canFeedbackRxTask";
  }
#endif
#if ROBOT_TASK_BUILD_CLASSIC_CHASSIS
  if (xTask == (TaskHandle_t)chassisControlTaskHandle)
  {
    return "chassisControlTask";
  }
#endif
#if ROBOT_TASK_BUILD_WHEELLEG_MIT
  if (xTask == (TaskHandle_t)wheellegMitTaskHandle)
  {
    return "wheellegMitTask";
  }
#endif
#if ROBOT_TASK_BUILD_ANY_GIMBAL
  if (xTask == (TaskHandle_t)gimbalControlTaskHandle)
  {
    return "gimbalControlTask";
  }
#endif
#if ROBOT_TASK_BUILD_HOST_LINK
  if (xTask == (TaskHandle_t)hostLinkTaskHandle)
  {
    return "hostLinkTask";
  }
#endif
#if ROBOT_TASK_BUILD_ELRS_LINK
  if (xTask == (TaskHandle_t)elrsLinkTaskHandle)
  {
    return "elrsLinkTask";
  }
#endif
#if ROBOT_TASK_BUILD_IMU
  if (xTask == (TaskHandle_t)imuTaskHandle)
  {
    return "imuFusionTask";
  }
#endif

  if (fallback_name != NULL && fallback_name[0] != '\0')
  {
    return fallback_name;
  }

  return "?";
}

void vApplicationStackOverflowHook(TaskHandle_t xTask, char *pcTaskName)
{
  RobotFaultEnterSafeStateEx((uint32_t)ROBOT_FAULT_REASON_STACK_OVERFLOW,
                                  0u,
                                  0u,
                                  (uint32_t)xTask,
                                  WatchRtosTaskNameFromHandle(xTask, pcTaskName));
  RobotFaultHaltForever();
}

void vApplicationMallocFailedHook(void)
{
  const TaskHandle_t current_task = RobotFaultCurrentTaskHandle();

  RobotFaultEnterSafeStateEx((uint32_t)ROBOT_FAULT_REASON_MALLOC_FAILED,
                                  0u,
                                  0u,
                                  (uint32_t)(uintptr_t)current_task,
                                  WatchRtosTaskNameFromHandle(current_task,
                                                                    RobotFaultTaskNameOrUnknown(current_task)));
  RobotFaultHaltForever();
}

/* USER CODE END Application */
