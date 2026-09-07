/* SPDX-License-Identifier: Apache-2.0 */
#include "RobotFaultZephyr.h"
#include "BspResetEvidence.h"
#include "BspCan.h"
#include "CanTxTask.h"
#include "RobotFaultTypes.h"
#include "Watch.h"

#include <soc.h>
#include <zephyr/fatal.h>
#include <zephyr/init.h>
#include <zephyr/kernel.h>
#include <zephyr/arch/arm/nmi.h>

/* 致命入口前不能尝试日志锁或扫描异常栈；摘要由本模块自行保存。 */
#if defined(__ZEPHYR__) && (defined(CONFIG_EXCEPTION_DUMP_HOOK) || \
    defined(CONFIG_EXCEPTION_DEBUG) || defined(CONFIG_EXCEPTION_STACK_TRACE) || \
    defined(CONFIG_ASSERT_VERBOSE) || CONFIG_FAULT_DUMP != 0)
#error "Fatal path must not log before saving evidence and stopping outputs"
#endif

static BspResetEvidenceRecord FaultRecord;
static BspResetEvidenceRecord FaultShadow;
static volatile uint32_t FaultShadowValid;
static volatile uint32_t FaultEntered;
static uint8_t FaultBootLocked;
static volatile uint32_t FaultLastTickMs;

void RobotFaultZephyrObserveTime(uint32_t tickMs)
{
    FaultLastTickMs = tickMs;
}

static void FaultRegistersRead(BspResetEvidenceRecord *record)
{
    record->ipsr = __get_IPSR();
    record->msp = __get_MSP();
    record->psp = __get_PSP();
    record->cfsr = SCB->CFSR;
    record->hfsr = SCB->HFSR;
    record->dfsr = SCB->DFSR;
    record->afsr = SCB->AFSR;
    record->mmfar = SCB->MMFAR;
    record->bfar = SCB->BFAR;
    record->icsr = SCB->ICSR;
    record->shcsr = SCB->SHCSR;
    record->control = __get_CONTROL();
}

extern void __real_z_arm_fault(uint32_t msp, uint32_t psp, uint32_t excReturn,
                               _callee_saved_t *callee);

void __wrap_z_arm_fault(uint32_t msp, uint32_t psp, uint32_t excReturn,
                        _callee_saved_t *callee)
{
    /* Zephyr 会清 CFSR；这里只保存原值，不改变其可恢复异常处理。 */
    FaultShadowValid = 0u;
    FaultRegistersRead(&FaultShadow);
    FaultShadow.msp = msp;
    FaultShadow.psp = psp;
    FaultShadow.excReturn = excReturn;
    FaultShadow.stackPtr = ((excReturn & 4u) != 0u) ? psp : msp;
    __DMB();
    FaultShadowValid = 1u;
    __real_z_arm_fault(msp, psp, excReturn, callee);
    /* 可恢复异常返回后，后续软件 panic 不能误用这份快照。 */
    FaultShadowValid = 0u;
}

static void FaultPwmStop(void)
{
#if defined(CONFIG_SOC_STM32H723XX)
    /* 只触碰本板已使能时钟的定时器，不进入 PWM 驱动和内核锁。 */
    if ((RCC->APB2ENR & RCC_APB2ENR_TIM1EN) != 0u) {
        TIM1->CCER = 0u;
        TIM1->BDTR &= ~TIM_BDTR_MOE;
    }
    if ((RCC->APB1LENR & RCC_APB1LENR_TIM2EN) != 0u) { TIM2->CCER = 0u; }
    if ((RCC->APB1LENR & RCC_APB1LENR_TIM3EN) != 0u) { TIM3->CCER = 0u; }
    if ((RCC->APB1LENR & RCC_APB1LENR_TIM12EN) != 0u) { TIM12->CCER = 0u; }
#endif
    __DSB();
}

static void FaultRs485Stop(void)
{
#if defined(CONFIG_SOC_STM32H723XX)
    /* 原始停机帧尝试结束后关闭发送器，防止旧 UART 字节继续输出。 */
    if ((RCC->APB1LENR & RCC_APB1LENR_USART2EN) != 0u) { USART2->CR1 &= ~USART_CR1_TE; }
    if ((RCC->APB1LENR & RCC_APB1LENR_USART3EN) != 0u) { USART3->CR1 &= ~USART_CR1_TE; }
#endif
}

static __attribute__((noreturn)) void FaultReset(void)
{
    __DSB();
    NVIC_SystemReset();
    for (;;) { __NOP(); }
}

static __attribute__((noreturn)) void FaultFinish(uint32_t reason, uint32_t arg0,
                                                uint32_t arg1, const struct arch_esf *esf)
{
    __disable_irq();
    if (FaultEntered != 0u) {
        /* 停机或记录本身发生二次异常时，不递归访问可能已损坏的总线。 */
        FaultReset();
    }
    FaultEntered = 1u;
    if (FaultShadowValid != 0u && esf != NULL) {
        FaultRecord = FaultShadow;
    } else {
        FaultRegistersRead(&FaultRecord);
    }
    FaultRecord.reason = reason;
    FaultRecord.arg0 = arg0;
    FaultRecord.arg1 = arg1;
    /* 最近一次正常取时刻的缓存，避免异常路径获取内核时钟锁。 */
    FaultRecord.tickMs = FaultLastTickMs;
    FaultRecord.bootStage = WatchDiagBootStageGet();
    /* 不调用 uptime、线程名、日志等可能获取内核锁的接口。 */
    if (esf != NULL) {
        FaultRecord.r0 = esf->basic.r0;
        FaultRecord.r1 = esf->basic.r1;
        FaultRecord.r2 = esf->basic.r2;
        FaultRecord.r3 = esf->basic.r3;
        FaultRecord.r12 = esf->basic.r12;
        FaultRecord.lr = esf->basic.lr;
        FaultRecord.pc = esf->basic.pc;
        FaultRecord.xpsr = esf->basic.xpsr;
#if defined(CONFIG_EXTRA_EXCEPTION_INFO)
        if (esf->extra_info.exc_return != 0u) {
            FaultRecord.excReturn = esf->extra_info.exc_return;
            FaultRecord.msp = esf->extra_info.msp;
        }
#endif
    }
    FaultPwmStop();
    BspResetEvidenceWriteFatal(&FaultRecord);
    /* 此函数只读事先生成并带校验的执行器路由，CAN 后端不再调用驱动锁。 */
    CanTxEmergencyStopNow();
    FaultRs485Stop();
    FaultReset();
}

void k_sys_fatal_error_handler(unsigned int reason, const struct arch_esf *esf)
{
    uint32_t mapped = ROBOT_FAULT_REASON_ERROR_HANDLER;
    switch (reason) {
        case K_ERR_CPU_EXCEPTION: mapped = ROBOT_FAULT_REASON_HARDFAULT; break;
        case K_ERR_STACK_CHK_FAIL: mapped = ROBOT_FAULT_REASON_STACK_OVERFLOW; break;
        case K_ERR_SPURIOUS_IRQ: mapped = ROBOT_FAULT_REASON_DEFAULT_INTERRUPT; break;
        case K_ERR_KERNEL_PANIC: mapped = ROBOT_FAULT_REASON_ASSERT; break;
        default:
            if (reason >= K_ERR_ARCH_START) { mapped = ROBOT_FAULT_REASON_HARDFAULT; }
            break;
    }
    /* arg0 保留原始 Zephyr reason；arg1 标识来源，避免与旧 HAL reason 混淆。 */
    FaultFinish(mapped, reason, 0x5a455048u, esf);
}

void __wrap_z_fatal_error(unsigned int reason, const struct arch_esf *esf)
{
    /* 绕过内核 fatal 的 LOG_ERR/线程名/转储，它们可能重入出错位置的锁。 */
    k_sys_fatal_error_handler(reason, esf);
}

#if defined(CONFIG_ASSERT_NO_FILE_INFO)
void assert_post_action(void)
{
    FaultFinish(ROBOT_FAULT_REASON_ASSERT, 0u, 0u, NULL);
}
#else
void assert_post_action(const char *file, unsigned int line)
{
    /* 只保存源码字符串地址与行号，不读取文件名；可用对应 ELF 还原。 */
    FaultFinish(ROBOT_FAULT_REASON_ASSERT, line, (uint32_t)(uintptr_t)file, NULL);
}
#endif

void RobotFaultZephyrReset(uint32_t reason, uint32_t arg0, uint32_t arg1)
{
    FaultFinish(reason, arg0, arg1, NULL);
}

static void FaultNmi(void)
{
    RobotFaultZephyrReset(ROBOT_FAULT_REASON_NMI, 0u, 0x5a455048u);
}

static int FaultNmiInit(void)
{
    z_arm_nmi_set_handler(FaultNmi);
    return 0;
}

SYS_INIT(FaultNmiInit, PRE_KERNEL_1, 1);

static int FaultBootCapture(void)
{
    BspResetEvidenceBoot boot;
    if (BspResetEvidenceGetBoot(&boot) != 0u && boot.evidenceValid != 0u) {
        FaultBootLocked = 1u;
    }
    return 0;
}

SYS_INIT(FaultBootCapture, PRE_KERNEL_2, 0);

uint8_t RobotFaultZephyrBootLocked(void)
{
    return FaultBootLocked;
}
