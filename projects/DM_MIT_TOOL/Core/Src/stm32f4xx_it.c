/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32f4xx_it.c
  * @brief   Interrupt Service Routines.
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
#include "main.h"
#include "stm32f4xx_it.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "spi.h"
#include "watch.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
 
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

typedef struct
{
  uint32_t r0;
  uint32_t r1;
  uint32_t r2;
  uint32_t r3;
  uint32_t r12;
  uint32_t lr;
  uint32_t pc;
  uint32_t psr;
  uint32_t exc_return;
  uint32_t msp;
  uint32_t psp;
  uint32_t cfsr;
  uint32_t hfsr;
  uint32_t dfsr;
  uint32_t afsr;
  uint32_t mmfar;
  uint32_t bfar;
  uint32_t icsr;
  uint32_t shcsr;
  uint32_t control;
  uint32_t stack_ptr;
  uint32_t basic_ptr;
  uint32_t stack_dump[16];
} hardfault_info_t;

volatile hardfault_info_t g_hardfault_info;

void HardFault_HandlerC(uint32_t *stack, uint32_t exc_return);

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern PCD_HandleTypeDef hpcd_USB_OTG_FS;
extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;
extern DMA_HandleTypeDef hdma_usart1_tx;
extern DMA_HandleTypeDef hdma_usart1_rx;
extern DMA_HandleTypeDef hdma_usart6_rx;
extern DMA_HandleTypeDef hdma_usart6_tx;
extern UART_HandleTypeDef huart1;
extern TIM_HandleTypeDef htim6;

/* USER CODE BEGIN EV */
extern DMA_HandleTypeDef hdma_spi2_rx;
extern DMA_HandleTypeDef hdma_spi2_tx;
extern DMA_HandleTypeDef hdma_tim4_ch3;
extern DMA_HandleTypeDef hdma_usart3_rx;
extern DMA_HandleTypeDef hdma_spi1_tx;

/* USER CODE END EV */

/******************************************************************************/
/*           Cortex-M4 Processor Interruption and Exception Handlers          */
/******************************************************************************/
/**
  * @brief This function handles Non maskable interrupt.
  */
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

  /* USER CODE END NonMaskableInt_IRQn 0 */
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */

  /* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
  * @brief This function handles Hard fault interrupt.
  */
#if defined(__CC_ARM)
__asm void HardFault_Handler(void)
{
  IMPORT HardFault_HandlerC

  /* USER CODE BEGIN HardFault_IRQn 0 */
  /* USER CODE END HardFault_IRQn 0 */

  TST lr, #4
  ITE EQ
  MRSEQ r0, MSP
  MRSNE r0, PSP
  MOV r1, lr
  B HardFault_HandlerC
}
#else
__attribute__((naked)) void HardFault_Handler(void)
{
  /* USER CODE BEGIN HardFault_IRQn 0 */
  /* USER CODE END HardFault_IRQn 0 */

  __asm volatile(
      "tst lr, #4            \n"
      "ite eq                \n"
      "mrseq r0, msp         \n"
      "mrsne r0, psp         \n"
      "mov r1, lr            \n"
      "b HardFault_HandlerC  \n");
}
#endif

void HardFault_HandlerC(uint32_t *stack, uint32_t exc_return)
{
  __disable_irq();

  // On Cortex-M4F, EXC_RETURN bit4 indicates whether FP context is stacked.
  // Depending on lazy stacking/toolchain, the passed SP can point either to:
  // - basic frame: R0-R3,R12,LR,PC,xPSR
  // - extended frame: S0-S15,FPSCR,reserved, then basic frame (18 words before R0)
  // We pick the frame that "looks valid" (Thumb PC + xPSR.T set).
  const uint32_t *frame0 = stack;
  const uint32_t *frame1 = stack + 18;

  const uint32_t pc0 = frame0[6];
  const uint32_t psr0 = frame0[7];
  const uint32_t pc1 = frame1[6];
  const uint32_t psr1 = frame1[7];

  const uint32_t xpsr_t_bit = (1UL << 24);
  const uint8_t valid0 = ((pc0 & 1U) != 0U) && ((psr0 & xpsr_t_bit) != 0U);
  const uint8_t valid1 = ((pc1 & 1U) != 0U) && ((psr1 & xpsr_t_bit) != 0U);

  const uint32_t *basic = frame0;
  if ((exc_return & (1UL << 4)) == 0U)
  {
    if (valid1 && !valid0)
    {
      basic = frame1;
    }
    else if (!valid1 && valid0)
    {
      basic = frame0;
    }
    else if (valid1 && valid0)
    {
      // Prefer Flash range if both look valid.
      const uint8_t pc0_in_flash = (pc0 >= 0x08000000U) && (pc0 < 0x08100000U);
      const uint8_t pc1_in_flash = (pc1 >= 0x08000000U) && (pc1 < 0x08100000U);
      if (pc1_in_flash && !pc0_in_flash)
      {
        basic = frame1;
      }
      else if (pc0_in_flash && !pc1_in_flash)
      {
        basic = frame0;
      }
      else
      {
        basic = frame1;
      }
    }
    else
    {
      basic = frame1;
    }
  }

  g_hardfault_info.r0 = basic[0];
  g_hardfault_info.r1 = basic[1];
  g_hardfault_info.r2 = basic[2];
  g_hardfault_info.r3 = basic[3];
  g_hardfault_info.r12 = basic[4];
  g_hardfault_info.lr = basic[5];
  g_hardfault_info.pc = basic[6];
  g_hardfault_info.psr = basic[7];

  g_hardfault_info.exc_return = exc_return;
  g_hardfault_info.msp = __get_MSP();
  g_hardfault_info.psp = __get_PSP();

  g_hardfault_info.cfsr = SCB->CFSR;
  g_hardfault_info.hfsr = SCB->HFSR;
  g_hardfault_info.dfsr = SCB->DFSR;
  g_hardfault_info.afsr = SCB->AFSR;
  g_hardfault_info.mmfar = SCB->MMFAR;
  g_hardfault_info.bfar = SCB->BFAR;
  g_hardfault_info.icsr = SCB->ICSR;
  g_hardfault_info.shcsr = SCB->SHCSR;
  g_hardfault_info.control = __get_CONTROL();
  g_hardfault_info.stack_ptr = (uint32_t)stack;
  g_hardfault_info.basic_ptr = (uint32_t)basic;

  for (uint32_t i = 0; i < (uint32_t)(sizeof(g_hardfault_info.stack_dump) / sizeof(g_hardfault_info.stack_dump[0])); i++)
  {
    g_hardfault_info.stack_dump[i] = 0U;
  }
  const uint32_t sp = (uint32_t)stack;
  const uint8_t sp_aligned = ((sp & 0x3U) == 0U);
  const uint8_t sp_in_sram = (sp >= 0x20000000U) && (sp < 0x20040000U);
  const uint8_t sp_in_ccm = (sp >= 0x10000000U) && (sp < 0x10020000U);
  if (sp_aligned && (sp_in_sram || sp_in_ccm))
  {
    for (uint32_t i = 0; i < (uint32_t)(sizeof(g_hardfault_info.stack_dump) / sizeof(g_hardfault_info.stack_dump[0])); i++)
    {
      g_hardfault_info.stack_dump[i] = stack[i];
    }
  }

  g_watch.fault.hardfault_valid = 1U;
  g_watch.fault.hardfault_r0 = g_hardfault_info.r0;
  g_watch.fault.hardfault_r1 = g_hardfault_info.r1;
  g_watch.fault.hardfault_r2 = g_hardfault_info.r2;
  g_watch.fault.hardfault_r3 = g_hardfault_info.r3;
  g_watch.fault.hardfault_r12 = g_hardfault_info.r12;
  g_watch.fault.hardfault_lr = g_hardfault_info.lr;
  g_watch.fault.hardfault_pc = g_hardfault_info.pc;
  g_watch.fault.hardfault_psr = g_hardfault_info.psr;
  g_watch.fault.hardfault_exc_return = g_hardfault_info.exc_return;
  g_watch.fault.hardfault_msp = g_hardfault_info.msp;
  g_watch.fault.hardfault_psp = g_hardfault_info.psp;
  g_watch.fault.hardfault_cfsr = g_hardfault_info.cfsr;
  g_watch.fault.hardfault_hfsr = g_hardfault_info.hfsr;
  g_watch.fault.hardfault_dfsr = g_hardfault_info.dfsr;
  g_watch.fault.hardfault_afsr = g_hardfault_info.afsr;
  g_watch.fault.hardfault_mmfar = g_hardfault_info.mmfar;
  g_watch.fault.hardfault_bfar = g_hardfault_info.bfar;
  g_watch.fault.hardfault_icsr = g_hardfault_info.icsr;
  g_watch.fault.hardfault_shcsr = g_hardfault_info.shcsr;
  g_watch.fault.hardfault_control = g_hardfault_info.control;
  g_watch.fault.hardfault_stack_ptr = g_hardfault_info.stack_ptr;
  g_watch.fault.hardfault_basic_ptr = g_hardfault_info.basic_ptr;
  for (uint32_t i = 0; i < (uint32_t)(sizeof(g_watch.fault.hardfault_stack_dump) / sizeof(g_watch.fault.hardfault_stack_dump[0])); i++)
  {
    g_watch.fault.hardfault_stack_dump[i] = g_hardfault_info.stack_dump[i];
  }

  if ((CoreDebug->DHCSR & CoreDebug_DHCSR_C_DEBUGEN_Msk) != 0U)
  {
    __BKPT(0);
  }

  while (1)
  {
    __NOP();
  }
}

/**
  * @brief This function handles Memory management fault.
  */
void MemManage_Handler(void)
{
  /* USER CODE BEGIN MemoryManagement_IRQn 0 */

  /* USER CODE END MemoryManagement_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_MemoryManagement_IRQn 0 */
    /* USER CODE END W1_MemoryManagement_IRQn 0 */
  }
}

/**
  * @brief This function handles Pre-fetch fault, memory access fault.
  */
void BusFault_Handler(void)
{
  /* USER CODE BEGIN BusFault_IRQn 0 */

  /* USER CODE END BusFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_BusFault_IRQn 0 */
    /* USER CODE END W1_BusFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Undefined instruction or illegal state.
  */
void UsageFault_Handler(void)
{
  /* USER CODE BEGIN UsageFault_IRQn 0 */

  /* USER CODE END UsageFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_UsageFault_IRQn 0 */
    /* USER CODE END W1_UsageFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Debug monitor.
  */
void DebugMon_Handler(void)
{
  /* USER CODE BEGIN DebugMonitor_IRQn 0 */

  /* USER CODE END DebugMonitor_IRQn 0 */
  /* USER CODE BEGIN DebugMonitor_IRQn 1 */

  /* USER CODE END DebugMonitor_IRQn 1 */
}

/******************************************************************************/
/* STM32F4xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f4xx.s).                    */
/******************************************************************************/

/**
  * @brief This function handles EXTI line0 interrupt.
  */
void EXTI0_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI0_IRQn 0 */

  /* USER CODE END EXTI0_IRQn 0 */
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_0);
  /* USER CODE BEGIN EXTI0_IRQn 1 */

  /* USER CODE END EXTI0_IRQn 1 */
}

/**
  * @brief This function handles EXTI line3 interrupt.
  */
void EXTI3_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI3_IRQn 0 */
  watch_irq_hit(WATCH_IRQ_IST8310_EXTI);

  /* USER CODE END EXTI3_IRQn 0 */
  HAL_GPIO_EXTI_IRQHandler(DRDY_IST8310_Pin);
  /* USER CODE BEGIN EXTI3_IRQn 1 */

  /* USER CODE END EXTI3_IRQn 1 */
}

/**
  * @brief This function handles EXTI line4 interrupt.
  */
void EXTI4_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI4_IRQn 0 */
  watch_irq_hit(WATCH_IRQ_IMU_EXTI);

  /* USER CODE END EXTI4_IRQn 0 */
  HAL_GPIO_EXTI_IRQHandler(INT1_ACCEL_Pin);
  /* USER CODE BEGIN EXTI4_IRQn 1 */

  /* USER CODE END EXTI4_IRQn 1 */
}

/**
  * @brief This function handles CAN1 RX0 interrupts.
  */
void CAN1_RX0_IRQHandler(void)
{
  /* USER CODE BEGIN CAN1_RX0_IRQn 0 */
  watch_irq_hit(WATCH_IRQ_CAN1_RX0);

  /* USER CODE END CAN1_RX0_IRQn 0 */
  HAL_CAN_IRQHandler(&hcan1);
  /* USER CODE BEGIN CAN1_RX0_IRQn 1 */

  /* USER CODE END CAN1_RX0_IRQn 1 */
}

/**
  * @brief This function handles EXTI line[9:5] interrupts.
  */
void EXTI9_5_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI9_5_IRQn 0 */
  watch_irq_hit(WATCH_IRQ_IMU_EXTI);

  /* USER CODE END EXTI9_5_IRQn 0 */
  HAL_GPIO_EXTI_IRQHandler(INT1_GYRO_Pin);
  /* USER CODE BEGIN EXTI9_5_IRQn 1 */

  /* USER CODE END EXTI9_5_IRQn 1 */
}

/**
  * @brief This function handles USART1 global interrupt.
  */
void USART1_IRQHandler(void)
{
  /* USER CODE BEGIN USART1_IRQn 0 */
  watch_irq_hit(WATCH_IRQ_USART1);

  /* USER CODE END USART1_IRQn 0 */
  HAL_UART_IRQHandler(&huart1);
  /* USER CODE BEGIN USART1_IRQn 1 */

  /* USER CODE END USART1_IRQn 1 */
}

/**
  * @brief This function handles TIM6 global interrupt, DAC1 and DAC2 underrun error interrupts.
  */
void TIM6_DAC_IRQHandler(void)
{
  /* USER CODE BEGIN TIM6_DAC_IRQn 0 */
  watch_irq_hit(WATCH_IRQ_TIM6_DAC);

  /* USER CODE END TIM6_DAC_IRQn 0 */
  HAL_TIM_IRQHandler(&htim6);
  /* USER CODE BEGIN TIM6_DAC_IRQn 1 */

  /* USER CODE END TIM6_DAC_IRQn 1 */
}

/**
  * @brief This function handles DMA2 stream1 global interrupt.
  */
void DMA2_Stream1_IRQHandler(void)
{
  /* USER CODE BEGIN DMA2_Stream1_IRQn 0 */

  /* USER CODE END DMA2_Stream1_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_usart6_rx);
  /* USER CODE BEGIN DMA2_Stream1_IRQn 1 */

  /* USER CODE END DMA2_Stream1_IRQn 1 */
}

/**
  * @brief This function handles CAN2 RX0 interrupts.
  */
void CAN2_RX0_IRQHandler(void)
{
  /* USER CODE BEGIN CAN2_RX0_IRQn 0 */
  watch_irq_hit(WATCH_IRQ_CAN2_RX0);

  /* USER CODE END CAN2_RX0_IRQn 0 */
  HAL_CAN_IRQHandler(&hcan2);
  /* USER CODE BEGIN CAN2_RX0_IRQn 1 */

  /* USER CODE END CAN2_RX0_IRQn 1 */
}

/**
  * @brief This function handles USB On The Go FS global interrupt.
  */
void OTG_FS_IRQHandler(void)
{
  /* USER CODE BEGIN OTG_FS_IRQn 0 */
  watch_irq_hit(WATCH_IRQ_OTG_FS);

  /* USER CODE END OTG_FS_IRQn 0 */
  HAL_PCD_IRQHandler(&hpcd_USB_OTG_FS);
  /* USER CODE BEGIN OTG_FS_IRQn 1 */

  /* USER CODE END OTG_FS_IRQn 1 */
}

/**
  * @brief This function handles DMA2 stream5 global interrupt.
  */
void DMA2_Stream5_IRQHandler(void)
{
  /* USER CODE BEGIN DMA2_Stream5_IRQn 0 */
  watch_irq_hit(WATCH_IRQ_DMA_USART1_RX);

  /* USER CODE END DMA2_Stream5_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_usart1_rx);
  /* USER CODE BEGIN DMA2_Stream5_IRQn 1 */

  /* USER CODE END DMA2_Stream5_IRQn 1 */
}

/**
  * @brief This function handles DMA2 stream6 global interrupt.
  */
void DMA2_Stream6_IRQHandler(void)
{
  /* USER CODE BEGIN DMA2_Stream6_IRQn 0 */

  /* USER CODE END DMA2_Stream6_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_usart6_tx);
  /* USER CODE BEGIN DMA2_Stream6_IRQn 1 */

  /* USER CODE END DMA2_Stream6_IRQn 1 */
}

/**
  * @brief This function handles DMA2 stream7 global interrupt.
  */
void DMA2_Stream7_IRQHandler(void)
{
  /* USER CODE BEGIN DMA2_Stream7_IRQn 0 */

  /* USER CODE END DMA2_Stream7_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_usart1_tx);
  /* USER CODE BEGIN DMA2_Stream7_IRQn 1 */

  /* USER CODE END DMA2_Stream7_IRQn 1 */
}

/**
  * @brief This function handles DMA1 stream1 global interrupt.
  */
void DMA1_Stream1_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Stream1_IRQn 0 */

  /* USER CODE END DMA1_Stream1_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_usart3_rx);
  /* USER CODE BEGIN DMA1_Stream1_IRQn 1 */

  /* USER CODE END DMA1_Stream1_IRQn 1 */
}

/**
  * @brief This function handles DMA1 stream3 global interrupt.
  */
void DMA1_Stream3_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Stream3_IRQn 0 */

  /* USER CODE END DMA1_Stream3_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_spi2_rx);
  /* USER CODE BEGIN DMA1_Stream3_IRQn 1 */

  /* USER CODE END DMA1_Stream3_IRQn 1 */
}

/**
  * @brief This function handles DMA1 stream4 global interrupt.
  */
void DMA1_Stream4_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Stream4_IRQn 0 */

  /* USER CODE END DMA1_Stream4_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_spi2_tx);
  /* USER CODE BEGIN DMA1_Stream4_IRQn 1 */

  /* USER CODE END DMA1_Stream4_IRQn 1 */
}

/**
  * @brief This function handles DMA1 stream7 global interrupt.
  */
void DMA1_Stream7_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Stream7_IRQn 0 */

  /* USER CODE END DMA1_Stream7_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_tim4_ch3);
  /* USER CODE BEGIN DMA1_Stream7_IRQn 1 */

  /* USER CODE END DMA1_Stream7_IRQn 1 */
}

/**
  * @brief This function handles DMA2 stream3 global interrupt.
  */
void DMA2_Stream3_IRQHandler(void)
{
  /* USER CODE BEGIN DMA2_Stream3_IRQn 0 */

  /* USER CODE END DMA2_Stream3_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_spi1_tx);
  /* USER CODE BEGIN DMA2_Stream3_IRQn 1 */

  /* USER CODE END DMA2_Stream3_IRQn 1 */
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
