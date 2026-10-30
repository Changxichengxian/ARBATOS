"""检查 C 板发射微动开关原始极性及 GPIO 错误返回。"""
import argparse
from pathlib import Path
import re
import shutil
import subprocess


ROOT = Path(__file__).resolve().parents[2]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--zig", default=shutil.which("zig"))
    args = parser.parse_args()
    if not args.zig:
        parser.error("请用 --zig 指定 Zig 编译器")
    code = (ROOT / "shared/zephyr/port/platform/BspShootTrig.c").read_text(encoding="utf-8")
    code = re.sub(r"(?m)^#include[^\n]*\n", "", code)
    fixture = r'''
#include <stdint.h>
#include <stddef.h>
#include <assert.h>
#include <stdio.h>
typedef unsigned char bool_t;
#define DT_PATH(x) 0
#define DT_NODE_EXISTS(x) 1
#define DT_NODE_HAS_PROP(...) 1
#define GPIO_DT_SPEC_GET(...) { .port=NULL, .pin=7 }
#define GPIO_INPUT 1
struct gpio_dt_spec { const void *port; unsigned pin; };
static int ready,configResult,readResult,configCalls;
static int gpio_is_ready_dt(const struct gpio_dt_spec *p) { (void)p; return ready; }
static int gpio_pin_configure_dt(const struct gpio_dt_spec *p,unsigned flags)
{ assert(p->pin==7 && flags==GPIO_INPUT); configCalls++; return configResult; }
static int gpio_pin_get_raw(const void *port,unsigned pin)
{ (void)port; assert(pin==7); return readResult; }
'''
    tests = r'''
int main(void)
{
    uint8_t level=0;
    assert(!BspShootTrigReadRaw(NULL));
    assert(!BspShootTrigReadRaw(&level) && level==1 && configCalls==0);
    ready=1; configResult=-1;
    assert(!BspShootTrigReadRaw(&level) && level==1 && configCalls==1);
    configResult=0; readResult=1;
    assert(BspShootTrigReadRaw(&level) && level==1 && configCalls==2);
    readResult=0;
    assert(BspShootTrigReadRaw(&level) && level==0 && configCalls==2);
    readResult=-5;
    assert(!BspShootTrigReadRaw(&level) && level==1 && configCalls==2);
    puts("PASS: shoot trigger GPIO init, active-low raw input and read errors");
    return 0;
}
'''
    out = ROOT / "local/build/host-tests/shoot-trig"
    out.mkdir(parents=True, exist_ok=True)
    source = out / "regression.c"
    source.write_text(fixture + code + tests, encoding="utf-8")
    executable = out / "regression.exe"
    subprocess.run([args.zig, "cc", "-std=c11", "-Wall", "-Wextra", "-Werror", str(source), "-o", str(executable)], check=True)
    subprocess.run([str(executable)], check=True)


if __name__ == "__main__":
    main()
