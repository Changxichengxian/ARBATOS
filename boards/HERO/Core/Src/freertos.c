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
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include "calibrate_task.h"
#include "can_tx_task.h"
#include "can_rx_task.h"
#include "chassis_task.h"
#include "detect_task.h"
#include "gimbal_task.h"
#include "INS_task.h"
#include "led_flow_task.h"
#include "rc_sbus_task.h"
#include "referee_usart_task.h"
#include "usb_task.h"
#include "elrs_task.h"
#include "voltage_task.h"
#include "servo_task.h"
#include "sdlog_task.h"
#include "app_watch.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

osThreadId calibrate_tast_handle;
osThreadId can_tx_task_handle;
osThreadId can_rx_task_handle;
osThreadId chassisTaskHandle;
osThreadId detect_handle;
osThreadId gimbalTaskHandle;
osThreadId imuTaskHandle;
osThreadId led_RGB_flow_handle;
osThreadId rc_sbus_task_handle;
osThreadId referee_usart_task_handle;
osThreadId usb_task_handle;
osThreadId uart1_elrs_thread_handle;
osThreadId battery_voltage_handle;
osThreadId servo_task_handle;
osThreadId sdlog_task_handle;


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
osThreadId testHandle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
   
/* USER CODE END FunctionPrototypes */

void test_task(void const * argument);

extern void MX_USB_DEVICE_Init(void);
void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

#define APP_STATIC_THREAD(name, entry, priority, stack_words) \
  static StaticTask_t name##_tcb; \
  static StackType_t name##_stack[(stack_words)]; \
  osThreadStaticDef(name, entry, priority, 0, stack_words, name##_stack, &name##_tcb)

APP_STATIC_THREAD(test, test_task, osPriorityNormal, 512);
APP_STATIC_THREAD(cali, calibrate_task, osPriorityNormal, 512);
APP_STATIC_THREAD(ChassisTask, chassis_task, osPriorityAboveNormal, 512);
APP_STATIC_THREAD(CANTX, can_tx_task, osPriorityAboveNormal, 256);
APP_STATIC_THREAD(CANRX, can_rx_task, osPriorityHigh, 256);
APP_STATIC_THREAD(RCSBUS, rc_sbus_task, osPriorityAboveNormal, 256);
APP_STATIC_THREAD(DETECT, detect_task, osPriorityNormal, 256);
APP_STATIC_THREAD(gimbalTask, gimbal_task, osPriorityHigh, 1024);
APP_STATIC_THREAD(imuTask, INS_task, osPriorityRealtime, 1024);
APP_STATIC_THREAD(led, led_RGB_flow_task, osPriorityNormal, 256);
APP_STATIC_THREAD(REFEREE, referee_usart_task, osPriorityNormal, 128);
APP_STATIC_THREAD(USBTask, usb_task, osPriorityNormal, 128);
APP_STATIC_THREAD(UART1_ELRS, uart1_elrs_task, osPriorityAboveNormal, 256);
APP_STATIC_THREAD(BATTERY_VOLTAGE, battery_voltage_task, osPriorityNormal, 128);
APP_STATIC_THREAD(SERVO, servo_task, osPriorityNormal, 128);
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
  /* definition and creation of test */
  // CubeMX default thread: runs MX_USB_DEVICE_Init().
  // USB init uses noticeable stack; keep this larger to avoid overflow.
  testHandle = osThreadCreate(osThread(test), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
    calibrate_tast_handle = osThreadCreate(osThread(cali), NULL);

    chassisTaskHandle = osThreadCreate(osThread(ChassisTask), NULL);

    can_tx_task_handle = osThreadCreate(osThread(CANTX), NULL);

    // CAN RX dispatcher: consume ISR ring, update motor measures and offline detect.
    can_rx_task_handle = osThreadCreate(osThread(CANRX), NULL);

    // USART3 SBUS/DBUS RX: consume ISR ring, parse frames, update RC_ctrl_t.
    rc_sbus_task_handle = osThreadCreate(osThread(RCSBUS), NULL);

    detect_handle = osThreadCreate(osThread(DETECT), NULL);

    gimbalTaskHandle = osThreadCreate(osThread(gimbalTask), NULL);

    imuTaskHandle = osThreadCreate(osThread(imuTask), NULL);

    led_RGB_flow_handle = osThreadCreate(osThread(led), NULL);


    referee_usart_task_handle = osThreadCreate(osThread(REFEREE), NULL);


    usb_task_handle = osThreadCreate(osThread(USBTask), NULL);

    // UART1 ELRS(CRSF) RX: high-priority, interrupt-driven parser (updates RC_ctrl_t).
    uart1_elrs_thread_handle = osThreadCreate(osThread(UART1_ELRS), NULL);

    battery_voltage_handle = osThreadCreate(osThread(BATTERY_VOLTAGE), NULL);

    servo_task_handle = osThreadCreate(osThread(SERVO), NULL);

    // TF/SD logger: low priority, flush buffered binary records to FatFs.
    sdlog_task_handle = osThreadCreate(osThread(SDLOG), NULL);



  /* USER CODE END RTOS_THREADS */

}

/* USER CODE BEGIN Header_test_task */
/**
  * @brief  Function implementing the test thread.
  * @param  argument: Not used 
  * @retval None
  */
/* USER CODE END Header_test_task */
__weak void test_task(void const * argument)
{
  /* init code for USB_DEVICE */
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN test_task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END test_task */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
void vApplicationStackOverflowHook(TaskHandle_t xTask, char *pcTaskName)
{
  (void)xTask;

  g_app_watch.rtos.fatal_reason = 1U;
  g_app_watch.rtos.fatal_task_handle = (uint32_t)xTask;
  for (uint32_t i = 0; i < (uint32_t)sizeof(g_app_watch.rtos.fatal_task_name); i++)
  {
    const char c = (pcTaskName != NULL) ? pcTaskName[i] : '\0';
    g_app_watch.rtos.fatal_task_name[i] = c;
    if (c == '\0')
    {
      break;
    }
  }
  g_app_watch.rtos.fatal_task_name[sizeof(g_app_watch.rtos.fatal_task_name) - 1] = '\0';

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
  g_app_watch.rtos.fatal_reason = 2U;
  g_app_watch.rtos.fatal_task_handle = (uint32_t)xTaskGetCurrentTaskHandle();
  {
    const char *name = pcTaskGetTaskName(NULL);
    for (uint32_t i = 0; i < (uint32_t)sizeof(g_app_watch.rtos.fatal_task_name); i++)
    {
      const char c = (name != NULL) ? name[i] : '\0';
      g_app_watch.rtos.fatal_task_name[i] = c;
      if (c == '\0')
      {
        break;
      }
    }
    g_app_watch.rtos.fatal_task_name[sizeof(g_app_watch.rtos.fatal_task_name) - 1] = '\0';
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
