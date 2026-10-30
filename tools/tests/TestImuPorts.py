import os
import subprocess
import tempfile
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
ZIG = Path(os.environ["LOCALAPPDATA"]) / "Microsoft/WinGet/Links/zig.exe"


def extract(text, start, end):
    return text[text.index(start):text.index(end, text.index(start))]


class ImuPortsTest(unittest.TestCase):
    def test_restored_bias_is_adjusted_incrementally(self):
        imu = (ROOT / "shared/zephyr/port/sensors/ImuFusionTask.c").read_text(encoding="utf-8")
        apply = extract(imu, "static void ImuApplyGyroOffset(", "__weak bool_t CalibrateGyroOffsetSave")
        callbacks = extract(imu, "static void ImuApplyGyroOffsetCallback(", "static void ImuCalibrationSampleDone")
        update = extract(imu, "static void ImuGyroCalibrationUpdate(",
                         "#if !defined(CONFIG_ARBATOS_PREFLIGHT_ONLY) && ROBOT_TASK_BUILD_SDLOG")
        source = """
#include <assert.h>
#include <math.h>
#include <string.h>
#include "GyroZeroCali.h"
#define ARG_UNUSED(x) (void)(x)
#define ROBOT_CALI_TARGET_IMU_GYRO 1
static fp32 InsGyroOffset[3] = {.04f, -.02f, .01f};
static fp32 InsGyro[3], InsTemp = 40;
static uint8_t InsGyroApplyDelta, InsHeaterStable = 1;
static GyroZeroCaliRuntimeState InsGyroCaliState;
static int robot_mode_is_calibration(int target) { (void)target; return 0; }
static int get_control_temperature(void) { return 40; }
static uint8_t ImuBootAdjustAllowed(void) { return 1; }
static void ImuRotateVector(fp32 *out, const fp32 *in) { memcpy(out, in, 3 * sizeof(fp32)); }
static void ImuCalibrationSampleDone(const GyroZeroCaliSampleState *s, void *ctx)
{ (void)s; (void)ctx; }
static bool_t ImuSaveGyroOffsetCallback(const fp32 *v, void *ctx)
{ (void)v; (void)ctx; return 1; }
""" + apply + callbacks + update + """
int main(void)
{
    const fp32 raw[3] = {-.03f, .015f, -.005f};
    const fp32 accel[3] = {0, 0, 9.80665f};
    GyroZeroCaliRuntimeReset(&InsGyroCaliState);
    for (unsigned i = 1; i < 6000; ++i) {
        for (unsigned j = 0; j < 3; ++j) InsGyro[j] = raw[j] + InsGyroOffset[j];
        ImuGyroCalibrationUpdate(raw, accel, i);
    }
    assert(GyroZeroCaliRuntimeIsCalibrated(&InsGyroCaliState));
    assert(fabsf(InsGyroOffset[0] - .03f) < .0001f);
    assert(fabsf(InsGyroOffset[1] + .015f) < .0001f);
    assert(fabsf(InsGyroOffset[2] - .005f) < .0001f);
    return 0;
}
"""
        with tempfile.TemporaryDirectory() as directory:
            c = Path(directory) / "bias.c"
            exe = Path(directory) / "bias.exe"
            c.write_text(source, encoding="utf-8")
            subprocess.run([str(ZIG), "cc", "-std=c99", "-Wall", "-Wextra", "-Werror",
                            "-I", str(ROOT / "shared/application/services/calibration"),
                            "-I", str(ROOT / "shared/components/support"),
                            str(c), "-lm", "-o", str(exe)], check=True)
            subprocess.run([str(exe)], check=True)

    def test_ready_configuration_failures(self):
        bmi = (ROOT / "shared/zephyr/port/sensors/BspBmi088Port.c").read_text(encoding="utf-8")
        callback = extract(bmi, "static void Bmi088OnReady(", "#endif")
        wait = extract(bmi, "int Bmi088PortWaitSample(", "uint32_t Bmi088PortErrorCount")
        init = extract(bmi, "void BMI088_GPIO_init(", "void BMI088_com_init")
        source = """
#include <assert.h>
#include <stdint.h>
#include <errno.h>
struct device { int unused; };
struct gpio_callback { int unused; };
struct gpio_dt_spec { struct device *port; int pin; };
typedef unsigned gpio_port_pins_t;
static struct device dev;
static struct device *Bmi088Spi = &dev;
static struct gpio_dt_spec Bmi088AccelCs = {&dev, 1}, Bmi088GyroCs = {&dev, 2};
static struct gpio_dt_spec Bmi088Ready = {&dev, 5};
static struct gpio_callback Bmi088ReadyCallback;
static uint8_t Bmi088ReadyConfigured;
static int Bmi088ReadySem, fail_input, fail_add, fail_interrupt, removed;
#define DT_NODE_HAS_PROP(a, b) 1
#define ARG_UNUSED(x) (void)(x)
#define K_MSEC(x) (x)
#define GPIO_OUTPUT_INACTIVE 1
#define GPIO_INPUT 2
#define GPIO_INT_EDGE_TO_ACTIVE 3
#define BIT(x) (1u << (x))
static int device_is_ready(const struct device *d) { (void)d; return 1; }
static int gpio_is_ready_dt(const struct gpio_dt_spec *g) { (void)g; return 1; }
static int gpio_pin_configure_dt(const struct gpio_dt_spec *g, int flags)
{ (void)g; return flags == GPIO_INPUT && fail_input ? -1 : 0; }
static void gpio_init_callback(struct gpio_callback *c,
    void (*fn)(const struct device *, struct gpio_callback *, gpio_port_pins_t), unsigned pins)
{ (void)c; (void)fn; (void)pins; }
static int gpio_add_callback(struct device *d, struct gpio_callback *c)
{ (void)d; (void)c; return fail_add ? -1 : 0; }
static int gpio_remove_callback(struct device *d, struct gpio_callback *c)
{ (void)d; (void)c; removed++; return 0; }
static int gpio_pin_interrupt_configure_dt(const struct gpio_dt_spec *g, int flags)
{ (void)g; (void)flags; return fail_interrupt ? -1 : 0; }
static void k_sem_give(int *sem) { *sem = 1; }
static int k_sem_take(int *sem, unsigned timeout)
{ (void)timeout; if (!*sem) return -EAGAIN; *sem = 0; return 0; }
""" + callback + wait + init + """
int main(void)
{
    fail_input = 1; BMI088_GPIO_init();
    assert(Bmi088PortWaitSample(20) == -ENODEV);
    fail_input = 0; fail_add = 1; BMI088_GPIO_init();
    assert(Bmi088PortWaitSample(20) == -ENODEV);
    fail_add = 0; fail_interrupt = 1; BMI088_GPIO_init();
    assert(removed == 1 && Bmi088PortWaitSample(20) == -ENODEV);
    fail_interrupt = 0; BMI088_GPIO_init();
    assert(Bmi088PortWaitSample(20) == -EAGAIN);
    Bmi088OnReady(&dev, &Bmi088ReadyCallback, BIT(5));
    assert(Bmi088PortWaitSample(20) == 0);
    assert(Bmi088PortWaitSample(20) == -EAGAIN);
    return 0;
}
"""
        with tempfile.TemporaryDirectory() as directory:
            c = Path(directory) / "ready.c"
            exe = Path(directory) / "ready.exe"
            c.write_text(source, encoding="utf-8")
            subprocess.run([str(ZIG), "cc", "-std=c99", "-Wall", "-Wextra", "-Werror",
                            str(c), "-o", str(exe)], check=True)
            subprocess.run([str(exe)], check=True)

    def test_spi_and_magnetometer(self):
        bmi = (ROOT / "shared/zephyr/port/sensors/BspBmi088Port.c").read_text(encoding="utf-8")
        mag = (ROOT / "shared/zephyr/port/sensors/BspIst8310Port.c").read_text(encoding="utf-8")
        source = """
#include <assert.h>
#include <stdint.h>
#include <stddef.h>
#include <string.h>
#include <errno.h>
#include <math.h>
struct spi_buf { void *buf; size_t len; };
struct spi_buf_set { const struct spi_buf *buffers; size_t count; };
static int Bmi088Spi, Bmi088SpiCfg, Ist8310I2c;
static volatile uint32_t Bmi088PortErrors;
static int spi_error, calls;
static unsigned expected_length;
static uint8_t expected_command;
static int spi_transceive(int dev, const int *cfg, const struct spi_buf_set *tx,
                          const struct spi_buf_set *rx)
{
    (void)dev; (void)cfg; calls++;
    assert(tx->buffers->len == expected_length && rx->buffers->len == expected_length);
    assert(((uint8_t *)tx->buffers->buf)[0] == expected_command);
    for (unsigned i = 1; i < expected_length; ++i)
        ((uint8_t *)rx->buffers->buf)[i] = (uint8_t)(0x20 + i);
    return spi_error;
}
static int ready = 1, i2c_error;
static uint8_t frame[7];
#define IST8310_IIC_ADDRESS 0x1c
static int device_is_ready(int dev) { (void)dev; return ready; }
static int i2c_burst_read(int dev, int address, int reg, uint8_t *data, size_t size)
{
    (void)dev; assert(address == 0x0e && reg == 2 && size == 7);
    memcpy(data, frame, size);
    return i2c_error;
}
""" + bmi[bmi.index("void Bmi088PortReadBytes("):] + "\n" + mag[mag.index("int BspIst8310Read("):] + """
int main(void)
{
    uint8_t out[6] = {0};
    expected_length = 7; expected_command = 0x82;
    Bmi088PortReadBytes(expected_command, out, 6, 0);
    assert(calls == 1 && out[0] == 0x21 && out[5] == 0x26);
    expected_length = 8;
    Bmi088PortReadBytes(expected_command, out, 6, 1);
    assert(calls == 2 && out[0] == 0x22 && out[5] == 0x27);
    spi_error = -EIO;
    Bmi088PortReadBytes(expected_command, out, 6, 1);
    assert(calls == 3 && Bmi088PortErrors == 1);
    for (unsigned i = 0; i < 6; ++i) assert(out[i] == 0);
    float xyz[3] = {9, 9, 9};
    assert(BspIst8310Read(NULL) == -EINVAL);
    ready = 0; assert(BspIst8310Read(xyz) == -ENODEV);
    ready = 1; i2c_error = -EIO; assert(BspIst8310Read(xyz) == -EIO);
    i2c_error = 0; assert(BspIst8310Read(xyz) == -EAGAIN);
    assert(xyz[0] == 9 && xyz[1] == 9 && xyz[2] == 9);
    frame[0] = 1; frame[1] = 10; frame[2] = 0;
    frame[3] = 0xf6; frame[4] = 0xff; frame[5] = 100; frame[6] = 0;
    assert(BspIst8310Read(xyz) == 0);
    assert(fabsf(xyz[0] - 3) < .001f && fabsf(xyz[1] + 3) < .001f &&
           fabsf(xyz[2] - 30) < .001f);
    return 0;
}
"""
        with tempfile.TemporaryDirectory() as directory:
            c = Path(directory) / "port.c"
            exe = Path(directory) / "port.exe"
            c.write_text(source, encoding="utf-8")
            subprocess.run([str(ZIG), "cc", "-std=c99", "-Wall", "-Wextra", "-Werror",
                            str(c), "-lm", "-o", str(exe)], check=True)
            subprocess.run([str(exe)], check=True)
