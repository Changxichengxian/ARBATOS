"""提取实际云台补偿函数，检查启停、衰减、测量转速和时钟回绕；不连接硬件。"""
import argparse
from pathlib import Path
import re
import shutil
import subprocess

from TestMotorRmProtocol import function

ROOT = Path(__file__).resolve().parents[2]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--zig", default=shutil.which("zig"))
    args = parser.parse_args()
    if not args.zig:
        parser.error("请用 --zig 指定 Zig 编译器")
    out = ROOT / "local/build/host-tests/gimbal-spin-ff"
    out.mkdir(parents=True, exist_ok=True)
    task = (ROOT / "shared/application/gimbal/GimbalControlTask.c").read_text(encoding="utf-8")
    definitions = "\n".join(re.findall(r"^#define GIMBAL_YAW_CHASSIS_\w+[^\n]*", task, re.M))
    body = function("shared/application/gimbal/GimbalCoreControl.inc", "GimbalYawChassisSpinFfCurrent")
    fixture = r'''
#include <assert.h>
#include <stdint.h>
#include <stddef.h>
#include <stdio.h>
#include <math.h>
typedef float fp32;
#define CHASSIS_STATE_MODE_NO_FOLLOW_YAW 2
typedef struct { uint8_t valid, mode; float wz_set, wz; } ChassisState;
typedef struct { float angle, angle_set; } GimbalMotor;
static ChassisState chassis;
static uint32_t tick;
static uint8_t read_ok = 1;
static struct { struct { float yaw_spin_ff_gain, yaw_spin_ff_limit; } gimbal; } g_config;
static uint8_t ChassisStateRead(ChassisState *out) { *out = chassis; return read_ok; }
static uint32_t BspTimeGetTickMs(void) { return tick; }
static float fp32_constrain(float x, float lo, float hi) { return fminf(hi, fmaxf(lo, x)); }
static float rad_format(float x) { return remainderf(x, 6.28318530718f); }
static void near(float actual, float expected) { assert(fabsf(actual - expected) < .01f); }
'''
    tests = r'''
int main(void)
{
    GimbalMotor yaw = {0};
    const float dir = YAW_TURN ? 1.0f : -1.0f;
    chassis = (ChassisState){.valid=1, .wz=.5f};
    near(GimbalYawChassisSpinFfCurrent(&yaw), 0);
    chassis.mode = 2; chassis.wz_set = 3.5f; chassis.wz = 0;
    near(GimbalYawChassisSpinFfCurrent(&yaw), 0);
    chassis.wz = 1;
    near(GimbalYawChassisSpinFfCurrent(&yaw), dir * 1500);
    chassis.wz = -9;
    near(GimbalYawChassisSpinFfCurrent(&yaw), -dir * 6000);
    g_config.gimbal.yaw_spin_ff_gain = 2200;
    g_config.gimbal.yaw_spin_ff_limit = 9000;
    chassis.wz = 3.5f;
    near(GimbalYawChassisSpinFfCurrent(&yaw), dir * 7700);
    chassis.wz = -9;
    near(GimbalYawChassisSpinFfCurrent(&yaw), -dir * 9000);
    chassis.mode = 0; chassis.wz = .5f;
    /* 退出发生在 tick=0 也必须启动窗口。 */
    near(GimbalYawChassisSpinFfCurrent(&yaw), dir * 600);
    tick = 350;
    near(GimbalYawChassisSpinFfCurrent(&yaw), dir * 300);
    tick = 700;
    near(GimbalYawChassisSpinFfCurrent(&yaw), 0);
    tick = 1200;
    near(GimbalYawChassisSpinFfCurrent(&yaw), 0);
    chassis.mode = 2;
    (void)GimbalYawChassisSpinFfCurrent(&yaw);
    chassis.mode = 0; chassis.wz = 0; yaw.angle_set = .1f;
    near(GimbalYawChassisSpinFfCurrent(&yaw), 1800);
    near(GimbalYawChassisSpinFfCurrent(NULL), 0);
    near(GimbalYawChassisSpinFfCurrent(&yaw), 0);
    /* 反馈失效后恢复普通模式，不能留下退出补偿。 */
    chassis.mode = 2;
    (void)GimbalYawChassisSpinFfCurrent(&yaw);
    read_ok = 0;
    near(GimbalYawChassisSpinFfCurrent(&yaw), 0);
    read_ok = 1; chassis.mode = 0;
    near(GimbalYawChassisSpinFfCurrent(&yaw), 0);
    chassis.mode = 2;
    (void)GimbalYawChassisSpinFfCurrent(&yaw);
    chassis.valid = 0;
    near(GimbalYawChassisSpinFfCurrent(&yaw), 0);
    chassis.valid = 1; chassis.mode = 0;
    near(GimbalYawChassisSpinFfCurrent(&yaw), 0);
    chassis.mode = 2; tick = UINT32_MAX - 99;
    (void)GimbalYawChassisSpinFfCurrent(&yaw);
    chassis.mode = 0; yaw.angle_set = 0; chassis.wz = .5f;
    near(GimbalYawChassisSpinFfCurrent(&yaw), dir * 600);
    tick += 350;
    near(GimbalYawChassisSpinFfCurrent(&yaw), dir * 300);
    tick += 350;
    near(GimbalYawChassisSpinFfCurrent(&yaw), 0);
    /* 负车型系数反转持续旋转及退出窗口的转速项，不反转角差纠偏。 */
    g_config.gimbal.yaw_spin_ff_gain = -2200;
    chassis.mode = 2; chassis.wz_set = 3.5f; chassis.wz = 3.5f;
    near(GimbalYawChassisSpinFfCurrent(&yaw), -dir * 7700);
    chassis.wz = -9;
    near(GimbalYawChassisSpinFfCurrent(&yaw), dir * 9000);
    chassis.mode = 0; chassis.wz = .5f;
    near(GimbalYawChassisSpinFfCurrent(&yaw), -dir * 600);
    tick += 350;
    near(GimbalYawChassisSpinFfCurrent(&yaw), -dir * 300);
    tick += 350;
    near(GimbalYawChassisSpinFfCurrent(&yaw), 0);
    chassis.mode = 2;
    (void)GimbalYawChassisSpinFfCurrent(&yaw);
    chassis.mode = 0; yaw.angle_set = .1f;
    near(GimbalYawChassisSpinFfCurrent(&yaw), 1800 - dir * 600);
    near(GimbalYawChassisSpinFfCurrent(NULL), 0);
    near(GimbalYawChassisSpinFfCurrent(&yaw), 0);
    puts("Gimbal spin feedforward regression passed");
    return 0;
}
'''
    source = out / "ExtractedSpin.c"
    source.write_text(fixture + definitions + "\n" + body + tests, encoding="utf-8")
    for turn in (0, 1):
        exe = out / f"spin-turn-{turn}.exe"
        subprocess.run([args.zig, "cc", "-std=c99", "-Wall", "-Wextra", "-Werror",
                        f"-DYAW_TURN={turn}", str(source), "-o", str(exe)], check=True)
        subprocess.run([str(exe)], check=True)


if __name__ == "__main__":
    main()
