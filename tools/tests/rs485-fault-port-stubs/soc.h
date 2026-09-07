#ifndef RS485_FAULT_PORT_TEST_SOC_H
#define RS485_FAULT_PORT_TEST_SOC_H

#include <stdint.h>

typedef struct
{
    volatile uint32_t CR1;
    volatile uint32_t CR2;
    volatile uint32_t CR3;
    volatile uint32_t BRR;
    volatile uint32_t GTPR;
    volatile uint32_t RTOR;
    volatile uint32_t RQR;
    volatile uint32_t ISR;
    volatile uint32_t ICR;
    volatile uint32_t RDR;
    volatile uint32_t TDR;
    volatile uint32_t PRESC;
} USART_TypeDef;

typedef struct { volatile uint32_t APB1LENR; } Rs485TestRcc;
extern Rs485TestRcc Rs485TestRccRegs;
extern USART_TypeDef Rs485TestUarts[2];
extern uint32_t Rs485TestPrimask;

#define RCC (&Rs485TestRccRegs)
#define RCC_APB1LENR_USART2EN (1u << 17)
#define RCC_APB1LENR_USART3EN (1u << 18)
#define USART_CR1_UE (1u << 0)
#define USART_CR1_TE (1u << 3)
#define USART_CR3_DEM (1u << 14)
#define USART_ICR_TCCF (1u << 6)
#define USART_ISR_TXE_TXFNF (1u << 7)
#define USART_ISR_TC (1u << 6)
#define __DSB() do {} while (0)
static inline uint32_t __get_PRIMASK(void) { return Rs485TestPrimask; }

#endif
