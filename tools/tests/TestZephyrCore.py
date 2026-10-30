import os
import shutil
import subprocess
import tempfile
import unittest
from pathlib import Path


REPO = Path(__file__).resolve().parents[2]


class ZephyrCoreTest(unittest.TestCase):
    def test_task_notify_preserves_same_bit_arriving_after_clear(self):
        zig = shutil.which("zig") or str(Path(os.environ["LOCALAPPDATA"]) /
                                        "Microsoft/WinGet/Links/zig.exe")
        compat = (REPO / "shared/zephyr/compat/freertos/ArbFreeRtosCompat.c").read_text(encoding="utf-8")
        start = compat.index("BaseType_t xTaskNotifyWait(")
        end = compat.index("void vTaskNotifyGiveFromISR(", start)
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            source = root / "notify.c"
            source.write_text("""
#include <assert.h>
#include <stdint.h>
#include <stddef.h>
typedef int BaseType_t;
typedef uint32_t TickType_t;
typedef int32_t atomic_val_t;
typedef struct { unsigned count; } Sem;
typedef struct { uint32_t notify_bits; Sem bits_notify; } ArbTaskCompat;
static ArbTaskCompat slot;
static int inject;
static int inject_before_clear;
#define pdFAIL 0
#define pdPASS 1
#define K_NO_WAIT 0
static ArbTaskCompat *ArbTaskForHandle(void *unused) { (void)unused; return &slot; }
static int ArbTimeoutFromTicks(TickType_t ticks) { return (int)ticks; }
static uint32_t atomic_get(uint32_t *p)
{
    uint32_t old = *p;
    if (inject_before_clear) {
        inject_before_clear = 0;
        *p |= 2u;
        slot.bits_notify.count = 1u;
    }
    return old;
}
static uint32_t atomic_and(uint32_t *p, atomic_val_t mask)
{
    uint32_t old = *p;
    *p &= (uint32_t)mask;
    if (inject && (uint32_t)mask == 0u) {
        inject = 0;
        *p |= 1u;
        slot.bits_notify.count = 1u;
    }
    return old;
}
static int k_sem_take(Sem *sem, int timeout)
{ (void)timeout; if (!sem->count) return -1; sem->count = 0; return 0; }
static void k_sem_give(Sem *sem) { sem->count = 1; }
""" + compat[start:end] + """
int main(void)
{
    uint32_t value = 99;
    assert(xTaskNotifyWait(0, UINT32_MAX, &value, 0) == pdFAIL);
    slot.notify_bits = 1u;
    slot.bits_notify.count = 1u;
    inject = 1;
    assert(xTaskNotifyWait(0, UINT32_MAX, &value, 0) == pdPASS && value == 1u);
    assert(xTaskNotifyWait(0, UINT32_MAX, &value, 0) == pdPASS && value == 1u);
    assert(xTaskNotifyWait(0, UINT32_MAX, &value, 0) == pdFAIL);
    slot.notify_bits = 1u;
    slot.bits_notify.count = 1u;
    inject_before_clear = 1;
    assert(xTaskNotifyWait(0, UINT32_MAX, &value, 0) == pdPASS && value == 3u);
    slot.notify_bits = 3u;
    slot.bits_notify.count = 1u;
    assert(xTaskNotifyWait(0, 1u, &value, 0) == pdPASS && value == 3u);
    assert(xTaskNotifyWait(0, UINT32_MAX, &value, 0) == pdPASS && value == 2u);
    return 0;
}
""", encoding="utf-8")
            output = root / "notify.exe"
            subprocess.run([zig, "cc", "-std=c99", "-Wall", "-Wextra", "-Werror", str(source),
                            "-o", str(output)], check=True)
            subprocess.run([str(output)], check=True)

    def test_uart_baud_is_available_before_receive_initialization(self):
        zig = shutil.which("zig") or str(Path(os.environ["LOCALAPPDATA"]) /
                                        "Microsoft/WinGet/Links/zig.exe")
        uart = (REPO / "shared/zephyr/port/uart/BspZephyrUart.c").read_text(encoding="utf-8")
        # 单独编译真实读取函数，设备查询由桩控制，不复制该函数的实现。
        start = uart.index("static uint32_t ArbUartGetBaudrate(")
        end = uart.index("static int ArbUartSetBaudrate(", start)
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            source = root / "main.c"
            source.write_text("""
#include <assert.h>
#include <stddef.h>
#include <stdint.h>
struct device { int ready; };
struct uart_config { uint32_t baudrate; };
typedef struct { const struct device *dev; uint32_t baudrate; } ArbUartPort;
static int result;
static unsigned queries;
static int device_is_ready(const struct device *dev) { return dev->ready; }
static int uart_config_get(const struct device *dev, struct uart_config *config)
{ (void)dev; queries++; config->baudrate = 230400; return result; }
""" + uart[start:end] + """
int main(void)
{
    struct device dev = {0};
    ArbUartPort port = {NULL, 0};
    assert(ArbUartGetBaudrate(&port) == 0);
    port.dev = &dev;
    assert(ArbUartGetBaudrate(&port) == 0);
    assert(queries == 0);
    dev.ready = 1;
    result = -1;
    assert(ArbUartGetBaudrate(&port) == 0);
    result = 0;
    assert(ArbUartGetBaudrate(&port) == 230400);
    assert(queries == 2);
    assert(ArbUartGetBaudrate(&port) == 230400);
    assert(queries == 2);
    return 0;
}
""", encoding="utf-8")
            output = root / "regression.exe"
            subprocess.run([zig, "cc", "-std=c99", "-Wall", "-Wextra", "-Werror", str(source),
                            "-o", str(output)], check=True)
            subprocess.run([str(output)], check=True)

    def test_pvd_preserves_power_registers_and_reads_live_status(self):
        zig = shutil.which("zig") or str(Path(os.environ["LOCALAPPDATA"]) /
                                        "Microsoft/WinGet/Links/zig.exe")
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            (root / "zephyr").mkdir()
            (root / "zephyr/kernel.h").write_text("""
#include <stdint.h>
#include <stddef.h>
#define ARG_UNUSED(x) (void)(x)
static inline int k_is_in_isr(void) { return 0; }
static inline void k_busy_wait(uint32_t us) { (void)us; }
static inline int k_msleep(int32_t ms) { return ms; }
static inline uint64_t k_cycle_get_64(void) { return 1; }
static inline uint64_t k_cyc_to_us_floor64(uint64_t n) { return n; }
static inline uint32_t k_uptime_get_32(void) { return 0; }
static inline unsigned irq_lock(void) { return 0; }
static inline void irq_unlock(unsigned key) { (void)key; }
""", encoding="utf-8")
            (root / "soc.h").write_text("""
typedef struct { unsigned APB1ENR; } TestRcc;
typedef struct { unsigned CR, CSR; } TestPwr;
static TestRcc rcc;
static TestPwr pwr;
#define RCC (&rcc)
#define PWR (&pwr)
#define RCC_APB1ENR_PWREN (1u << 28)
#define PWR_CR_PLS (7u << 5)
#define PWR_CR_PVDE (1u << 4)
#define PWR_CSR_PVDO (1u << 2)
""", encoding="utf-8")
            for name in ("BspDelay.h", "BspFric.h", "BspPwr.h", "BspTime.h"):
                (root / name).write_text("", encoding="utf-8")
            (root / "BspCrc32.h").write_text("typedef unsigned char bool_t;", encoding="utf-8")
            (root / "BspDwt.h").write_text("void BSP_DWT_Init(void);", encoding="utf-8")
            (root / "BspResetEvidence.h").write_text("""
typedef struct { unsigned resetFlags; } BspResetEvidenceBoot;
static unsigned captures;
static void BspResetEvidenceCaptureBoot(void) { captures++; }
static unsigned BspResetEvidenceGetBoot(BspResetEvidenceBoot *boot)
{ boot->resetFlags = 123; return 1; }
""", encoding="utf-8")
            (root / "RobotFaultZephyr.h").write_text(
                "static void RobotFaultZephyrObserveTime(unsigned t) { (void)t; }", encoding="utf-8")
            source = root / "main.c"
            source.write_text('#include <assert.h>\n#include "' +
                              (REPO / "shared/zephyr/port/common/BspCoreZephyr.c").as_posix() + '"\n' + """
int main(void)
{
#if defined(CONFIG_SOC_SERIES_STM32F4X)
    rcc.APB1ENR = 0x42u;
    pwr.CR = 0x8001u;
    BspPwrPvdInit();
    assert(rcc.APB1ENR == (0x42u | RCC_APB1ENR_PWREN));
    assert(pwr.CR == (0x8001u | PWR_CR_PLS | PWR_CR_PVDE));
    assert(BspPwrPvdVddLow() == 0);
    pwr.CSR = PWR_CSR_PVDO;
    assert(BspPwrPvdVddLow() == 1);
    pwr.CSR = 0;
    assert(BspPwrPvdVddLow() == 0);
    BspPwrPvdInit();
    assert(pwr.CR == (0x8001u | PWR_CR_PLS | PWR_CR_PVDE));
    assert(captures == 2);
#else
    BspPwrPvdInit();
    assert(BspPwrPvdVddLow() == 0);
    assert(captures == 1);
#endif
    assert(BspPwrRccCsr() == 123);
    return 0;
}
""", encoding="utf-8")
            for macros in ([], ["-DCONFIG_SOC_SERIES_STM32F4X"]):
                with self.subTest(macros=macros):
                    output = root / "regression.exe"
                    subprocess.run([zig, "cc", "-std=c99", "-Wall", "-Wextra", "-Werror", *macros,
                                    "-I" + str(root), str(source), "-o", str(output)], check=True)
                    subprocess.run([str(output)], check=True)


if __name__ == "__main__":
    unittest.main()
