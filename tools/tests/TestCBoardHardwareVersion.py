import os
from pathlib import Path
import re
import subprocess
import tempfile

root = Path(__file__).resolve().parents[2]
source = (root / "shared/zephyr/port/platform/BspAdc.c").read_text(encoding="utf-8")


def function(name):
    match = re.search(r"(?:static\s+)?(?:int|uint8_t)\s+" + name + r"\(void\)\s*\{", source)
    if not match:
        raise ValueError(name)
    body = source.index("{", match.start())
    depth = 0
    for end in range(body, len(source)):
        depth += (source[end] == "{") - (source[end] == "}")
        if depth == 0:
            return source[match.start():end + 1]
    raise ValueError(name)


harness = r'''
#include <stdint.h>
#include <assert.h>
#define ARRAY_SIZE(a) (sizeof(a) / sizeof((a)[0]))
#define GPIO_INPUT 1
struct gpio_dt_spec { int index; };
static const struct gpio_dt_spec HardwareVersionPins[] = {{0}, {1}, {2}};
static uint8_t HardwareVersionReady;
static int values[3], ready = 1, config_error = -1, read_error = -1;
static int gpio_is_ready_dt(const struct gpio_dt_spec *pin) { (void)pin; return ready; }
static int gpio_pin_configure_dt(const struct gpio_dt_spec *pin, int flags) {
    assert(flags == GPIO_INPUT); return pin->index == config_error ? -1 : 0;
}
static int gpio_pin_get_dt(const struct gpio_dt_spec *pin) {
    return pin->index == read_error ? -1 : values[pin->index];
}
'''
harness += "\n".join(function(name) for name in ("BspHardwareVersionInit", "get_hardware_version"))
harness += r'''
int main(void) {
    assert(get_hardware_version() == 0xff);
    ready = 0; assert(BspHardwareVersionInit() != 0);
    ready = 1; config_error = 1; assert(BspHardwareVersionInit() != 0);
    config_error = -1; assert(BspHardwareVersionInit() == 0);
    for (unsigned int version = 0; version < 8; version++) {
        for (unsigned int i = 0; i < 3; i++) values[i] = (version >> i) & 1;
        assert(get_hardware_version() == version);
    }
    read_error = 2; assert(get_hardware_version() == 0xff);
    return 0;
}
'''
zig = Path(os.environ["LOCALAPPDATA"]) / "Microsoft/WinGet/Links/zig.exe"
with tempfile.TemporaryDirectory() as directory:
    test_source = Path(directory) / "test.c"
    executable = Path(directory) / "test.exe"
    test_source.write_text(harness, encoding="utf-8")
    subprocess.run([str(zig), "cc", "-std=c99", "-Wall", "-Wextra", "-Werror", str(test_source), "-o", str(executable)], check=True)
    subprocess.run([str(executable)], check=True)
