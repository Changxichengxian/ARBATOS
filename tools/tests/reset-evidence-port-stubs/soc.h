#ifndef RESET_EVIDENCE_PORT_TEST_SOC_H
#define RESET_EVIDENCE_PORT_TEST_SOC_H

#include <stdint.h>

typedef struct
{
    volatile uint32_t CR1;
} ResetEvidenceTestPwr;

typedef struct
{
    volatile uint32_t AHB4ENR;
    volatile uint32_t RSR;
} ResetEvidenceTestRcc;

typedef struct
{
    volatile uint32_t CCR;
} ResetEvidenceTestScb;

extern ResetEvidenceTestPwr ResetEvidenceTestPwrRegs;
extern ResetEvidenceTestRcc ResetEvidenceTestRccRegs;
extern ResetEvidenceTestScb ResetEvidenceTestScbRegs;
extern uint8_t ResetEvidenceTestBackupSram[512];

#define PWR (&ResetEvidenceTestPwrRegs)
#define RCC (&ResetEvidenceTestRccRegs)
#define SCB (&ResetEvidenceTestScbRegs)
#define D3_BKPSRAM_BASE ((uintptr_t)ResetEvidenceTestBackupSram)
#define PWR_CR1_DBP (1u << 8)
#define RCC_AHB4ENR_BKPRAMEN (1u << 28)
#define RCC_RSR_RMVF (1u << 16)
#define SCB_CCR_DC_Msk (1u << 16)

static inline void __DSB(void) {}
static inline void __DMB(void) {}
static inline uint32_t __get_PRIMASK(void) { return 0u; }
static inline void __disable_irq(void) {}
static inline void __enable_irq(void) {}
static inline void SCB_InvalidateDCache_by_Addr(uint32_t *address, int32_t size)
{
    (void)address;
    (void)size;
}
static inline void SCB_CleanDCache_by_Addr(uint32_t *address, int32_t size)
{
    (void)address;
    (void)size;
}

#endif
