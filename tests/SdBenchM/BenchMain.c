#include "main.h"
#include "cmsis_os2.h"
#include "FreeRTOS.h"
#include "task.h"
#include "SdBench.h"

extern TIM_HandleTypeDef htim23;
static TIM_HandleTypeDef SdBenchTimer;

/* 启动汇编沿用M板；早期入口不访问尚未初始化的全局变量。 */
void ExitRun0Mode(void) {}
void RobotFaultEarlyInit(void) {}
void RobotFaultDefaultHandler(void) { SdBenchStop(-119); }

/* 此入口没有机器人配置，也不初始化 CAN、串口、电机、IMU 或执行器定时器。 */
static void SdBenchClockInit(void)
{
    RCC_OscInitTypeDef osc = {0};
    RCC_ClkInitTypeDef clk = {0};

    HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE0);
    while (!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {
    }
    osc.OscillatorType = RCC_OSCILLATORTYPE_HSI48 | RCC_OSCILLATORTYPE_HSI | RCC_OSCILLATORTYPE_HSE;
    osc.HSEState = RCC_HSE_ON;
    osc.HSIState = RCC_HSI_DIV1;
    osc.HSICalibrationValue = 64;
    osc.HSI48State = RCC_HSI48_ON;
    osc.PLL.PLLState = RCC_PLL_ON;
    osc.PLL.PLLSource = RCC_PLLSOURCE_HSE;
    osc.PLL.PLLM = 2;
    osc.PLL.PLLN = 40;
    osc.PLL.PLLP = 1;
    /* 与上一轮副板测试一致：CPU 480 MHz、SPI123 内核 96 MHz。 */
    osc.PLL.PLLQ = 5;
    osc.PLL.PLLR = 2;
    osc.PLL.PLLRGE = RCC_PLL1VCIRANGE_3;
    osc.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
    if (HAL_RCC_OscConfig(&osc) != HAL_OK) {
        SdBenchStop(-101);
    }
    clk.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 |
                    RCC_CLOCKTYPE_PCLK2 | RCC_CLOCKTYPE_D3PCLK1 | RCC_CLOCKTYPE_D1PCLK1;
    clk.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    clk.SYSCLKDivider = RCC_SYSCLK_DIV1;
    clk.AHBCLKDivider = RCC_HCLK_DIV2;
    clk.APB3CLKDivider = RCC_APB3_DIV2;
    clk.APB1CLKDivider = RCC_APB1_DIV2;
    clk.APB2CLKDivider = RCC_APB2_DIV2;
    clk.APB4CLKDivider = RCC_APB4_DIV2;
    if (HAL_RCC_ClockConfig(&clk, FLASH_LATENCY_3) != HAL_OK) {
        SdBenchStop(-102);
    }
}

static void SdBenchTimerInit(void)
{
    __HAL_RCC_TIM2_CLK_ENABLE();
    SdBenchTimer.Instance = TIM2;
    SdBenchTimer.Init.Prescaler = (HAL_RCC_GetPCLK1Freq() * 2u / 1000000u) - 1u;
    SdBenchTimer.Init.CounterMode = TIM_COUNTERMODE_UP;
    SdBenchTimer.Init.Period = 0xFFFFFFFFu;
    SdBenchTimer.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    if (HAL_TIM_Base_Init(&SdBenchTimer) != HAL_OK || HAL_TIM_Base_Start(&SdBenchTimer) != HAL_OK) {
        SdBenchStop(-103);
    }
}

uint32_t SdBenchNowUs(void)
{
    return __HAL_TIM_GET_COUNTER(&SdBenchTimer);
}

void SdBenchStop(int32_t error)
{
    __disable_irq();
    SdBenchState.magic = SD_BENCH_MAGIC;
    SdBenchState.version = SD_BENCH_VERSION;
    SdBenchState.size = sizeof(SdBenchState);
    SdBenchState.error = error;
    SdBenchState.faultCfsr = SCB->CFSR;
    SdBenchState.faultHfsr = SCB->HFSR;
    SdBenchState.phase = SD_BENCH_PHASE_FAILED;
    /* 故障时保留现场，不自动复位后反复创建测试文件。 */
    while (1) {
        __NOP();
    }
}

int main(void)
{
    const osThreadAttr_t attr = {
        .name = "sd_bench",
        .stack_size = 8192u,
        .priority = osPriorityNormal,
    };
    SdBenchState.magic = SD_BENCH_MAGIC;
    SdBenchState.version = SD_BENCH_VERSION;
    SdBenchState.size = sizeof(SdBenchState);
    HAL_Init();
    SdBenchClockInit();
    SdBenchTimerInit();
    if (osKernelInitialize() != osOK || osThreadNew(SdBenchTask, NULL, &attr) == NULL) {
        SdBenchStop(-104);
    }
    if (osKernelStart() != osOK) {
        SdBenchStop(-105);
    }
    SdBenchStop(-106);
    return 0;
}

void HAL_MspInit(void)
{
    __HAL_RCC_SYSCFG_CLK_ENABLE();
    HAL_NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_4);
    HAL_NVIC_SetPriority(PendSV_IRQn, 15, 0);
}

void TIM23_IRQHandler(void)
{
    HAL_TIM_IRQHandler(&htim23);
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *timer)
{
    if (timer->Instance == TIM23) {
        HAL_IncTick();
    }
}

void Error_Handler(void) { SdBenchStop(-110); }
void NMI_Handler(void) { SdBenchStop(-111); }
void HardFault_Handler(void) { SdBenchStop(-112); }
void MemManage_Handler(void) { SdBenchStop(-113); }
void BusFault_Handler(void) { SdBenchStop(-114); }
void UsageFault_Handler(void) { SdBenchStop(-115); }

void RobotFaultAssert(const char *file, uint32_t line)
{
    (void)file;
    (void)line;
    SdBenchStop(-116);
}

void vApplicationMallocFailedHook(void) { SdBenchStop(-117); }
void vApplicationStackOverflowHook(TaskHandle_t task, char *name)
{
    (void)task;
    (void)name;
    SdBenchStop(-118);
}

void vApplicationGetIdleTaskMemory(StaticTask_t **task, StackType_t **stack, uint32_t *size)
{
    static StaticTask_t idleTask;
    static StackType_t idleStack[configMINIMAL_STACK_SIZE];
    *task = &idleTask;
    *stack = idleStack;
    *size = configMINIMAL_STACK_SIZE;
}

void vApplicationGetTimerTaskMemory(StaticTask_t **task, StackType_t **stack, uint32_t *size)
{
    static StaticTask_t timerTask;
    static StackType_t timerStack[configTIMER_TASK_STACK_DEPTH];
    *task = &timerTask;
    *stack = timerStack;
    *size = configTIMER_TASK_STACK_DEPTH;
}
