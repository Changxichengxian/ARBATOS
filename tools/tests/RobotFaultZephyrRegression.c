/* 真实 RobotFaultZephyr.c 的宿主回归：异常路径由 NVIC_SystemReset 桩跳出。 */

#include <setjmp.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "soc.h"
#include "BspResetEvidence.h"
#include <zephyr/fatal.h>

TestScb TestScbInstance;
TestRcc TestRccInstance;
TestTim TestTim1Instance, TestTim2Instance, TestTim3Instance, TestTim12Instance;
TestUsart TestUsart2Instance, TestUsart3Instance;
uint32_t TestIpsr, TestMsp, TestPsp, TestControl;
unsigned TestDmbCount, TestDsbCount, TestIrqDisableCount;

static jmp_buf TestResetJump;
static uint8_t TestExpectReset;
static unsigned TestEvidenceWrites, TestCanStops, TestResets;
static uint32_t TestBootStage;
static BspResetEvidenceRecord TestEvidence;
static BspResetEvidenceBoot TestBoot;
static uint8_t TestBootAvailable;
static void (*TestNmiHandler)(void);
static char TestOrder[32];
static unsigned TestOrderLength;
static uint8_t TestRealFaultClearCfsr;
static uint8_t TestRealFaultInvokeFatal;

static void TestLog(char value)
{
    if (TestOrderLength + 1u < sizeof(TestOrder)) {
        TestOrder[TestOrderLength++] = value;
        TestOrder[TestOrderLength] = '\0';
    }
}

uint32_t __get_IPSR(void) { return TestIpsr; }
uint32_t __get_MSP(void) { return TestMsp; }
uint32_t __get_PSP(void) { return TestPsp; }
uint32_t __get_CONTROL(void) { return TestControl; }
void __disable_irq(void) { TestIrqDisableCount++; TestLog('I'); }
void __DMB(void) { TestDmbCount++; }
void __DSB(void) { TestDsbCount++; TestLog('D'); }
void __NOP(void) { }
void NVIC_SystemReset(void)
{
    TestResets++;
    TestLog('R');
    if (TestExpectReset != 0u) longjmp(TestResetJump, 1);
}

void BspResetEvidenceWriteFatal(const BspResetEvidenceRecord *record)
{
    TestEvidence = *record;
    TestEvidenceWrites++;
    TestLog('E');
}

uint8_t BspResetEvidenceGetBoot(BspResetEvidenceBoot *out)
{
    if (TestBootAvailable == 0u) return 0u;
    *out = TestBoot;
    return 1u;
}

void BspResetEvidenceCaptureBoot(void) { }
void BspResetEvidenceAcknowledge(uint32_t sequence) { (void)sequence; }
void CanTxEmergencyStopNow(void) { TestCanStops++; TestLog('C'); }
uint32_t WatchDiagBootStageGet(void) { return TestBootStage; }
void z_arm_nmi_set_handler(void (*handler)(void)) { TestNmiHandler = handler; }

#include "../../shared/zephyr/port/platform/RobotFaultZephyr.c"

void __real_z_arm_fault(uint32_t msp, uint32_t psp, uint32_t excReturn,
                        _callee_saved_t *callee)
{
    (void)msp; (void)psp; (void)excReturn; (void)callee;
    if (TestRealFaultClearCfsr != 0u) SCB->CFSR = 0u;
    if (TestRealFaultInvokeFatal != 0u) {
        struct arch_esf esf = {0};
        __wrap_z_fatal_error(K_ERR_CPU_EXCEPTION, &esf);
    }
}

static int Check(int condition, const char *message)
{
    if (condition != 0) return 1;
    (void)fprintf(stderr, "FAIL: %s\n", message);
    return 0;
}

static void ResetFixture(void)
{
    (void)memset(&TestScbInstance, 0, sizeof(TestScbInstance));
    (void)memset(&TestRccInstance, 0, sizeof(TestRccInstance));
    (void)memset(&TestTim1Instance, 0, sizeof(TestTim1Instance));
    (void)memset(&TestTim2Instance, 0, sizeof(TestTim2Instance));
    (void)memset(&TestTim3Instance, 0, sizeof(TestTim3Instance));
    (void)memset(&TestTim12Instance, 0, sizeof(TestTim12Instance));
    (void)memset(&TestUsart2Instance, 0, sizeof(TestUsart2Instance));
    (void)memset(&TestUsart3Instance, 0, sizeof(TestUsart3Instance));
    (void)memset(&TestEvidence, 0, sizeof(TestEvidence));
    (void)memset(&TestBoot, 0, sizeof(TestBoot));
    (void)memset(TestOrder, 0, sizeof(TestOrder));
    TestIpsr = 0x51u; TestMsp = 0x20001000u; TestPsp = 0x20002000u; TestControl = 3u;
    TestDmbCount = TestDsbCount = TestIrqDisableCount = 0u;
    TestExpectReset = TestEvidenceWrites = TestCanStops = TestResets = 0u;
    TestBootStage = 0x2468u; TestBootAvailable = 0u; TestNmiHandler = NULL;
    TestOrderLength = 0u; TestRealFaultClearCfsr = TestRealFaultInvokeFatal = 0u;
    FaultEntered = 0u; FaultShadowValid = 0u; FaultBootLocked = 0u; FaultLastTickMs = 0u;
}

static int ResetNow(uint32_t reason, uint32_t arg0, uint32_t arg1)
{
    TestExpectReset = 1u;
    if (setjmp(TestResetJump) == 0) {
        RobotFaultZephyrReset(reason, arg0, arg1);
        return 0;
    }
    TestExpectReset = 0u;
    return 1;
}

static int ZephyrFatalNow(unsigned int reason, const struct arch_esf *esf)
{
    TestExpectReset = 1u;
    if (setjmp(TestResetJump) == 0) {
        __wrap_z_fatal_error(reason, esf);
        return 0;
    }
    TestExpectReset = 0u;
    return 1;
}

static int AssertPostActionNow(const char *file, unsigned int line)
{
    TestExpectReset = 1u;
    if (setjmp(TestResetJump) == 0) {
        assert_post_action(file, line);
        return 0;
    }
    TestExpectReset = 0u;
    return 1;
}

static int TestWrappedFaultKeepsOriginalCfsr(void)
{
    _callee_saved_t callee = {0};
    ResetFixture();
    SCB->CFSR = 0xa5a55a5au;
    TestRealFaultClearCfsr = 1u;
    TestRealFaultInvokeFatal = 1u;
    TestExpectReset = 1u;
    if (setjmp(TestResetJump) == 0) {
        __wrap_z_arm_fault(0x20003000u, 0x20004000u, 4u, &callee);
        return Check(0, "fatal must reset instead of returning");
    }
    TestExpectReset = 0u;
    return Check(TestEvidence.cfsr == 0xa5a55a5au, "real handler clears CFSR after wrapper but evidence must retain original") &&
           Check(TestEvidence.msp == 0x20003000u && TestEvidence.psp == 0x20004000u &&
                     TestEvidence.stackPtr == 0x20004000u, "wrapper exception stack fields must survive into fatal evidence");
}

static int TestRecoverableWrapperInvalidatesShadow(void)
{
    _callee_saved_t callee = {0};
    ResetFixture();
    SCB->CFSR = 0x11111111u;
    TestRealFaultClearCfsr = 1u;
    __wrap_z_arm_fault(1u, 2u, 0u, &callee);
    if (!Check(TestEvidenceWrites == 0u && TestCanStops == 0u && TestResets == 0u,
               "recoverable z_arm_fault return must not stop or reset")) return 0;
    if (!Check(FaultShadowValid == 0u, "recoverable z_arm_fault return must invalidate shadow")) return 0;
    return Check(ResetNow(77u, 1u, 2u) && TestEvidence.cfsr == 0u,
                 "later software panic must read current registers, not stale shadow");
}

static int TestFatalOrderAndPanicFields(void)
{
    ResetFixture();
    RCC->APB2ENR = RCC_APB2ENR_TIM1EN;
    RCC->APB1LENR = RCC_APB1LENR_TIM2EN | RCC_APB1LENR_TIM3EN | RCC_APB1LENR_TIM12EN |
                   RCC_APB1LENR_USART2EN | RCC_APB1LENR_USART3EN;
    TIM1->CCER = TIM2->CCER = TIM3->CCER = TIM12->CCER = 0xffffffffu;
    TIM1->BDTR = TIM_BDTR_MOE;
    USART2->CR1 = USART3->CR1 = USART_CR1_TE;
    RobotFaultZephyrObserveTime(1234u);
    if (!Check(ResetNow(ROBOT_FAULT_REASON_ASSERT, 0x12u, 0x34u), "software fatal must reset")) return 0;
    if (!Check(strstr(TestOrder, "IDECDR") != NULL, "fatal order must disable IRQ, stop PWM, write evidence, stop CAN, reset")) return 0;
    if (!Check(TIM1->CCER == 0u && (TIM1->BDTR & TIM_BDTR_MOE) == 0u && TIM2->CCER == 0u &&
               TIM3->CCER == 0u && TIM12->CCER == 0u && (USART2->CR1 & USART_CR1_TE) == 0u &&
               (USART3->CR1 & USART_CR1_TE) == 0u, "fatal must disable PWM outputs and RS485 transmitters")) return 0;
    return Check(TestEvidence.reason == ROBOT_FAULT_REASON_ASSERT && TestEvidence.arg0 == 0x12u &&
                     TestEvidence.arg1 == 0x34u && TestEvidence.tickMs == 1234u &&
                     TestEvidence.bootStage == TestBootStage,
                 "software panic fields, observed tick and pure-read boot stage must be recorded");
}

static int TestWrappedFatalAndAssertPostAction(void)
{
    static const char file[] = "fatal-entry-test.c";
    struct arch_esf esf = {0};

    ResetFixture();
    esf.basic.pc = 0x08001234u;
    if (!Check(ZephyrFatalNow(K_ERR_ARCH_START, &esf), "wrapped Zephyr fatal must reset")) return 0;
    if (!Check(TestEvidence.reason == ROBOT_FAULT_REASON_HARDFAULT &&
               TestEvidence.arg0 == K_ERR_ARCH_START && TestEvidence.arg1 == 0x5a455048u &&
               TestEvidence.pc == 0x08001234u,
               "CPU architecture fatal reasons must map to hardfault through wrap entry")) return 0;

    ResetFixture();
    if (!Check(AssertPostActionNow(file, 321u), "assert post action must reset")) return 0;
    return Check(TestEvidence.reason == ROBOT_FAULT_REASON_ASSERT && TestEvidence.arg0 == 321u &&
                     TestEvidence.arg1 == (uint32_t)(uintptr_t)file,
                 "assert post action must persist assert reason, source line and file address");
}

static int TestNestedFatalDoesNotWriteAgain(void)
{
    ResetFixture();
    if (!Check(ResetNow(1u, 2u, 3u), "first fatal must reset")) return 0;
    return Check(ResetNow(4u, 5u, 6u) && TestEvidenceWrites == 1u && TestCanStops == 1u && TestResets == 2u,
                 "second entry before reset completion must only reset, without recursive writes or CAN stop");
}

static int TestBootLockAndNmiRegistration(void)
{
    ResetFixture();
    TestBootAvailable = 1u;
    TestBoot.evidenceValid = 1u;
    if (!Check(FaultBootCapture() == 0 && RobotFaultZephyrBootLocked() != 0u,
               "valid boot evidence must lock this run")) return 0;
    TestBoot.evidenceValid = 0u;
    if (!Check(FaultBootCapture() == 0 && RobotFaultZephyrBootLocked() != 0u,
               "later acknowledgement must not unlock an already locked run")) return 0;
    if (!Check(FaultNmiInit() == 0 && TestNmiHandler != NULL, "NMI handler must register when Zephyr API is available")) return 0;
    return Check((TestExpectReset = 1u, setjmp(TestResetJump)) != 0 ?
                     (TestExpectReset = 0u, TestEvidence.reason == ROBOT_FAULT_REASON_NMI) :
                     (TestNmiHandler(), 0), "registered NMI handler must enter fatal reset path");
}

int main(void)
{
    if (!TestWrappedFaultKeepsOriginalCfsr()) return 1;
    if (!TestRecoverableWrapperInvalidatesShadow()) return 1;
    if (!TestFatalOrderAndPanicFields()) return 1;
    if (!TestWrappedFatalAndAssertPostAction()) return 1;
    if (!TestNestedFatalDoesNotWriteAgain()) return 1;
    if (!TestBootLockAndNmiRegistration()) return 1;
    (void)puts("PASS: RobotFaultZephyr fatal-entry host regression");
    return 0;
}
