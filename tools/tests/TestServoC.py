"""读取 C 板实际资源映射，编译真实 PWM 授权适配与故障关断函数。"""
import argparse
from pathlib import Path
import re
import shutil
import subprocess

from TestGimbalCalibration import function


ROOT = Path(__file__).resolve().parents[2]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--zig", default=shutil.which("zig"))
    args = parser.parse_args()
    if not args.zig:
        parser.error("请用 --zig 指定 Zig 编译器")
    dts = (ROOT / "boards/DjiCF407/zephyr/dji_c_f407.dts").read_text(encoding="utf-8")
    pinctrl = (ROOT / "boards/DjiCF407/zephyr/dji_c_f407-pinctrl.dtsi").read_text(encoding="utf-8")
    resources = dts.split("servo-pwms =", 1)[1].split(";", 1)[0]
    mapping = re.findall(r"<&pwm(\d+)\s+(\d+)\s+PWM_MSEC\(20\)\s+PWM_POLARITY_NORMAL>", resources)
    assert mapping == [("8", "2"), ("8", "1"), ("1", "4"), ("1", "3")]
    for timer, channel, port, pin, af in ((8, 2, "I", 6, 3), (8, 1, "C", 6, 3),
                                         (1, 4, "E", 14, 1), (1, 3, "E", 13, 1)):
        label = f"tim{timer}_ch{channel}_p{port.lower()}{pin}"
        node = re.search(rf"{label}:\s*{label}\s*\{{(.*?)\}}", pinctrl, re.S).group(1)
        assert f"STM32_PINMUX('{port}', {pin}, AF{af})" in node
        block = re.search(rf"&timers{timer}\s*\{{(.*?)\n\}};", dts, re.S).group(1)
        assert "st,prescaler = <167>" in block and label in block
    specifications = "\n".join(f"#define SERVO_SPEC_{i} {{.dev=&devices[{timer}], .channel={channel}, .period=20000000u}}"
                                for i, (timer, channel) in enumerate(mapping))
    fixture = r'''
#include <assert.h>
#include <stdint.h>
#include <stddef.h>
#include <errno.h>
#include <stdio.h>
#include "ServoOutputPolicy.h"
#define DT_PATH(x) 0
#define DT_NODE_HAS_PROP(...) 1
#define ARRAY_SIZE(a) (sizeof(a)/sizeof((a)[0]))
#define PWM_USEC(us) ((us)*1000u)
#define CONFIG_BOARD_DJI_C_F407 1
#define CONFIG_ARBATOS_PREFLIGHT_ONLY 0
#define IS_ENABLED(x) (x)
#define ROBOT_RUN_MODE_FULL 0
#define ROBOT_TASK_MODULE_SERVO 1
static unsigned irqDepth,authorized=1,bootLocked,safetyLocked,lifecycle=1,mode,singleTask,inIsr;
uint8_t LowCmdOutputSnapshotAuthorized(MotorId id,const MotorCmd *cmd,const ControlOutputStamp *owner)
{ (void)id; (void)cmd; (void)owner; assert(irqDepth); return authorized; }
static unsigned irq_lock(void) { return irqDepth++; }
static void irq_unlock(unsigned key) { assert(irqDepth==key+1); irqDepth=key; }
static unsigned k_is_in_isr(void) { return inIsr; }
static unsigned RobotFaultZephyrBootLocked(void) { return bootLocked; }
static unsigned RobotSafetyOutputLocked(void) { return safetyLocked; }
static unsigned RobotLifecycleOutputAllowed(void) { return lifecycle; }
static unsigned robot_mode_current(void) { return mode; }
static unsigned robot_mode_is_single_task(unsigned task) { assert(task==1); return singleTask; }
struct device { unsigned timer; };
static struct device devices[9];
struct pwm_dt_spec { const struct device *dev; unsigned channel,period; };
static unsigned pulses[9][5],writes;
static int pwm_is_ready_dt(const struct pwm_dt_spec *p) { (void)p; return 1; }
static int pwm_set_dt(const struct pwm_dt_spec *p,unsigned period,unsigned pulse)
{ assert(irqDepth && period==20000000u && pulse<=2500000u); pulses[p->dev->timer][p->channel]=pulse; writes++; return 0; }
#define ARBATOS_PWM_DT_SPEC_GET_BY_IDX(node,prop,idx) SERVO_SPEC_##idx
'''
    adapter = (ROOT / "shared/zephyr/port/platform/BspServoPwm.c").read_text(encoding="utf-8")
    adapter = re.sub(r"(?m)^#include[^\n]*\n", "", adapter)
    fault_fixture = r'''
static struct { uint32_t APB2ENR; } rcc;
static struct { uint32_t CCER,BDTR; } tim1,tim8;
#define RCC (&rcc)
#define TIM1 (&tim1)
#define TIM8 (&tim8)
#define RCC_APB2ENR_TIM1EN 1u
#define RCC_APB2ENR_TIM8EN 2u
#define TIM_BDTR_MOE (1u<<15)
static unsigned barriers;
static void __DSB(void) { barriers++; }
'''
    tests = r'''
int main(void)
{
    for(unsigned i=0;i<9;i++) devices[i].timer=i;
    ServoChannelConfig channel={.enabled=1,.minUs=500,.centerUs=1500,.maxUs=2500,.stepUs=10};
    MotorCmd cmd={.active=1,.mode=MotorModePosVel,.timeoutMs=30,.tick=100,.q=0};
    ControlOutputStamp owner={.valid=1};
    unsigned timers[4]={8,8,1,1},channels[4]={2,1,4,3};
    for(unsigned port=0;port<4;port++) {
        channel.port=port;
        assert(ServoPwmApply(&channel,Motor18,&cmd,&owner,110)==1500);
        assert(pulses[timers[port]][channels[port]]==1500000);
    }
    authorized=0;
    assert(ServoPwmApply(&channel,Motor18,&cmd,&owner,110)==0 && pulses[1][3]==0);
    authorized=1; bootLocked=1; assert(ServoPwmApply(&channel,Motor18,&cmd,&owner,110)==0);
    bootLocked=0; safetyLocked=1; assert(ServoPwmApply(&channel,Motor18,&cmd,&owner,110)==0);
    safetyLocked=0; lifecycle=0; assert(ServoPwmApply(&channel,Motor18,&cmd,&owner,110)==0);
    lifecycle=1; mode=2; assert(ServoPwmApply(&channel,Motor18,&cmd,&owner,110)==0);
    mode=0; assert(ServoPwmApply(&channel,Motor18,&cmd,&owner,131)==0);
    cmd.mode=MotorModeDisable; assert(ServoPwmApply(&channel,Motor18,&cmd,&owner,110)==0);
    cmd.mode=MotorModePosVel;
    unsigned before=writes; inIsr=1; assert(ServoPwmApply(&channel,Motor18,&cmd,&owner,110)==0 && writes==before);
    inIsr=0; ServoPwmDisable(0); assert(pulses[8][2]==0 && irqDepth==0);
    tim1.CCER=tim8.CCER=0xffff; tim1.BDTR=tim8.BDTR=TIM_BDTR_MOE|7;
    FaultPwmStop(); assert(tim1.CCER==0xffff && tim8.CCER==0xffff);
    rcc.APB2ENR=RCC_APB2ENR_TIM1EN;
    FaultPwmStop(); assert(tim1.CCER==0 && tim1.BDTR==7 && tim8.CCER==0xffff);
    rcc.APB2ENR|=RCC_APB2ENR_TIM8EN;
    FaultPwmStop(); assert(tim8.CCER==0 && tim8.BDTR==7 && barriers==3);
    puts("PASS: C board servo pins, order, 20ms period, LowCmd guards and fatal PWM stop");
    return 0;
}
'''
    out = ROOT / "local/build/host-tests/servo-c"
    out.mkdir(parents=True, exist_ok=True)
    source = out / "regression.c"
    source.write_text(fixture + specifications + "\n" + adapter + fault_fixture +
                      function("shared/zephyr/port/platform/RobotFaultZephyr.c", "FaultPwmStop") + tests,
                      encoding="utf-8")
    executable = out / "regression.exe"
    subprocess.run([args.zig, "cc", "-std=c11", "-Wall", "-Wextra", "-Werror", "-DROBOT_TASK_BUILD_SERVO=1",
                    "-I" + str(ROOT / "shared/application/robot"),
                    "-I" + str(ROOT / "shared/application/services/servo"),
                    "-I" + str(ROOT / "shared/components/support"), str(source), "-o", str(executable)], check=True)
    subprocess.run([str(executable)], check=True)


if __name__ == "__main__":
    main()
