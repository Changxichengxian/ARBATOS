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
#include "rc_sbus_task.h"
#include "detect_task.h"
#include "sdlog_task.h"
#include "can_feedback_rx_task.h"
#include "can_command_tx_task.h"
#include "chassis_control_task.h"
#include "gimbal_control_task.h"
#include "INS_task.h"
#include "config.h"
#include "app_task_bootstrap.h"
#include "control_manager.h"
#include "wheelleg_mit_task.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
osThreadId_t rcSbusTaskHandle;
osThreadId_t detectTaskHandle;
osThreadId_t sdlogTaskHandle;
osThreadId_t canFeedbackRxTaskHandle;
osThreadId_t canCommandTxTaskHandle;
osThreadId_t chassisControlTaskHandle;
osThreadId_t wheellegMitTaskHandle;
osThreadId_t gimbalControlTaskHandle;
osThreadId_t imuTaskHandle;

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

APP_THREAD_ATTR(defaultTask, osPriorityNormal, 512);
APP_THREAD_ATTR(rcSbusTask, osPriorityAboveNormal, 256);
APP_THREAD_ATTR(healthMonitorTask, osPriorityNormal, 256);
APP_THREAD_ATTR(sdlogTask, osPriorityLow, 512);
APP_THREAD_ATTR(canCommandTxTask, osPriorityAboveNormal, 256);
APP_THREAD_ATTR(canFeedbackRxTask, osPriorityHigh, 256);
APP_THREAD_ATTR(chassisControlTask, osPriorityAboveNormal, 512);
APP_THREAD_ATTR(wheellegMitTask, osPriorityAboveNormal, 768);
APP_THREAD_ATTR(gimbalControlTask, osPriorityHigh, 512);
APP_THREAD_ATTR(imuFusionTask, osPriorityRealtime, 1024);

static osThreadId_t app_create_rc_sbus_task(void)
{
  return APP_THREAD_CREATE(rcSbusTask, rc_sbus_task);
}

static osThreadId_t app_create_health_monitor_task(void)
{
  return APP_THREAD_CREATE(healthMonitorTask, health_monitor_task);
}

static osThreadId_t app_create_sdlog_task(void)
{
  return APP_THREAD_CREATE(sdlogTask, sdlog_task);
}

static osThreadId_t app_create_can_command_tx_task(void)
{
  return APP_THREAD_CREATE(canCommandTxTask, can_command_tx_task);
}

static osThreadId_t app_create_can_feedback_rx_task(void)
{
  return APP_THREAD_CREATE(canFeedbackRxTask, can_feedback_rx_task);
}

static osThreadId_t app_create_chassis_control_task(void)
{
  return APP_THREAD_CREATE(chassisControlTask, chassis_control_task);
}

static osThreadId_t app_create_wheelleg_mit_task(void)
{
  return APP_THREAD_CREATE(wheellegMitTask, wheelleg_mit_task);
}

static osThreadId_t app_create_single_gimbal_task(void)
{
  return APP_THREAD_CREATE(gimbalControlTask, gimbal_control_task);
}

static osThreadId_t app_create_dual_yaw_gimbal_task(void)
{
  return APP_THREAD_CREATE(gimbalControlTask, dual_yaw_gimbal_control_task);
}

static osThreadId_t app_create_imu_task(void)
{
  return APP_THREAD_CREATE(imuFusionTask, imu_fusion_task);
}

static void app_clear_module_task_handles(void)
{
  rcSbusTaskHandle = NULL;
  detectTaskHandle = NULL;
  sdlogTaskHandle = NULL;
  canCommandTxTaskHandle = NULL;
  canFeedbackRxTaskHandle = NULL;
  chassisControlTaskHandle = NULL;
  wheellegMitTaskHandle = NULL;
  gimbalControlTaskHandle = NULL;
  imuTaskHandle = NULL;
}

static void app_create_module_tasks(void)
{
  static const app_task_module_desc_t module_tasks[] =
  {
    {ROBOT_TASK_MODULE_RC_SBUS, &rcSbusTaskHandle, app_create_rc_sbus_task},
    {ROBOT_TASK_MODULE_HEALTH_MONITOR, &detectTaskHandle, app_create_health_monitor_task},
    {ROBOT_TASK_MODULE_SDLOG, &sdlogTaskHandle, app_create_sdlog_task},
    {ROBOT_TASK_MODULE_CAN_COMMAND_TX, &canCommandTxTaskHandle, app_create_can_command_tx_task},
    {ROBOT_TASK_MODULE_CAN_FEEDBACK_RX, &canFeedbackRxTaskHandle, app_create_can_feedback_rx_task},
    {ROBOT_TASK_MODULE_CLASSIC_CHASSIS, &chassisControlTaskHandle, app_create_chassis_control_task},
    {ROBOT_TASK_MODULE_WHEELLEG_MIT, &wheellegMitTaskHandle, app_create_wheelleg_mit_task},
    {ROBOT_TASK_MODULE_SINGLE_GIMBAL, &gimbalControlTaskHandle, app_create_single_gimbal_task},
    {ROBOT_TASK_MODULE_DUAL_YAW_GIMBAL, &gimbalControlTaskHandle, app_create_dual_yaw_gimbal_task},
    {ROBOT_TASK_MODULE_IMU, &imuTaskHandle, app_create_imu_task},
  };

  app_clear_module_task_handles();

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
  defaultTaskHandle = APP_THREAD_CREATE(defaultTask, StartDefaultTask);

  /* USER CODE BEGIN RTOS_THREADS */
  app_create_module_tasks();
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
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN StartDefaultTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartDefaultTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */
