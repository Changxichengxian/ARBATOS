/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

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
osThreadId_t KeyTaskHandle;
StackType_t KeyTaskBuffer[128];
StaticTask_t KeyTaskControlBlock;
osThreadId_t LcdTaskHandle;
StackType_t LcdTaskBuffer[256];
StaticTask_t LcdTaskControlBlock;
osThreadId_t ImuTaskHandle;
StackType_t ImuTaskBuffer[1024];
StaticTask_t ImuTaskControlBlock;
osThreadId_t FunTestHandle;
StackType_t FunTestBuffer[128];
StaticTask_t FunTestControlBlock;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);
void KeyTask_Entry(void *argument);
void LcdTask_Entry(void *argument);
void ImuTask_Entry(void *argument);
void FunTest_Entry(void *argument);

extern void MX_USB_DEVICE_Init(void);
void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

#define APP_THREAD_ATTR(thread_id, prio, stack_words) \
  static const osThreadAttr_t thread_id##_attr = { \
    .name = #thread_id, \
    .priority = (prio), \
    .stack_size = (stack_words) * sizeof(StackType_t), \
  }

#define APP_STATIC_THREAD_ATTR(thread_id, prio, stack_buffer, control_block) \
  static const osThreadAttr_t thread_id##_attr = { \
    .name = #thread_id, \
    .priority = (prio), \
    .stack_mem = (stack_buffer), \
    .stack_size = sizeof(stack_buffer), \
    .cb_mem = &(control_block), \
    .cb_size = sizeof(control_block), \
  }

#define APP_THREAD_CREATE(thread_id, entry) \
  osThreadNew((osThreadFunc_t)(entry), NULL, &thread_id##_attr)

APP_THREAD_ATTR(defaultTask, osPriorityNormal, 128);
APP_STATIC_THREAD_ATTR(KeyTask, osPriorityRealtime, KeyTaskBuffer, KeyTaskControlBlock);
APP_STATIC_THREAD_ATTR(LcdTask, osPriorityNormal, LcdTaskBuffer, LcdTaskControlBlock);
APP_STATIC_THREAD_ATTR(ImuTask, osPriorityHigh, ImuTaskBuffer, ImuTaskControlBlock);
APP_STATIC_THREAD_ATTR(FunTest, osPriorityBelowNormal, FunTestBuffer, FunTestControlBlock);

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
  KeyTaskHandle = APP_THREAD_CREATE(KeyTask, KeyTask_Entry);
  LcdTaskHandle = APP_THREAD_CREATE(LcdTask, LcdTask_Entry);
  ImuTaskHandle = APP_THREAD_CREATE(ImuTask, ImuTask_Entry);
  FunTestHandle = APP_THREAD_CREATE(FunTest, FunTest_Entry);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
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

/* USER CODE BEGIN Header_KeyTask_Entry */
/**
* @brief Function implementing the KeyTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_KeyTask_Entry */
__weak void KeyTask_Entry(void *argument)
{
  (void)argument;
  /* USER CODE BEGIN KeyTask_Entry */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END KeyTask_Entry */
}

/* USER CODE BEGIN Header_LcdTask_Entry */
/**
* @brief Function implementing the LcdTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_LcdTask_Entry */
__weak void LcdTask_Entry(void *argument)
{
  (void)argument;
  /* USER CODE BEGIN LcdTask_Entry */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END LcdTask_Entry */
}

/* USER CODE BEGIN Header_ImuTask_Entry */
/**
* @brief Function implementing the ImuTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_ImuTask_Entry */
__weak void ImuTask_Entry(void *argument)
{
  (void)argument;
  /* USER CODE BEGIN ImuTask_Entry */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END ImuTask_Entry */
}

/* USER CODE BEGIN Header_FunTest_Entry */
/**
* @brief Function implementing the FunTest thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_FunTest_Entry */
__weak void FunTest_Entry(void *argument)
{
  (void)argument;
  /* USER CODE BEGIN FunTest_Entry */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END FunTest_Entry */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */
