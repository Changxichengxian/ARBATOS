#ifndef TEST_ROBOT_FAULT_SOC_H
#define TEST_ROBOT_FAULT_SOC_H

#include <stdint.h>

typedef struct { uint32_t CFSR, HFSR, DFSR, AFSR, MMFAR, BFAR, ICSR, SHCSR; } TestScb;
typedef struct { uint32_t APB2ENR, APB1LENR; } TestRcc;
typedef struct { uint32_t CCER, BDTR; } TestTim;
typedef struct { uint32_t CR1; } TestUsart;

extern TestScb TestScbInstance;
extern TestRcc TestRccInstance;
extern TestTim TestTim1Instance, TestTim2Instance, TestTim3Instance, TestTim12Instance;
extern TestUsart TestUsart2Instance, TestUsart3Instance;
extern uint32_t TestIpsr, TestMsp, TestPsp, TestControl;
extern unsigned TestDmbCount, TestDsbCount, TestIrqDisableCount;

#define SCB (&TestScbInstance)
#define RCC (&TestRccInstance)
#define TIM1 (&TestTim1Instance)
#define TIM2 (&TestTim2Instance)
#define TIM3 (&TestTim3Instance)
#define TIM12 (&TestTim12Instance)
#define USART2 (&TestUsart2Instance)
#define USART3 (&TestUsart3Instance)
#define RCC_APB2ENR_TIM1EN (1u << 0)
#define RCC_APB1LENR_TIM2EN (1u << 0)
#define RCC_APB1LENR_TIM3EN (1u << 1)
#define RCC_APB1LENR_TIM12EN (1u << 2)
#define RCC_APB1LENR_USART2EN (1u << 3)
#define RCC_APB1LENR_USART3EN (1u << 4)
#define TIM_BDTR_MOE (1u << 15)
#define USART_CR1_TE (1u << 3)

uint32_t __get_IPSR(void);
uint32_t __get_MSP(void);
uint32_t __get_PSP(void);
uint32_t __get_CONTROL(void);
void __disable_irq(void);
void __DMB(void);
void __DSB(void);
void __NOP(void);
void NVIC_SystemReset(void);

#endif
