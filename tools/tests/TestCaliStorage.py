import os
from pathlib import Path
import re
import subprocess
import tempfile

root = Path(__file__).resolve().parents[2]
source = (root / "shared/application/services/calibration/CalibrateTask.c").read_text(encoding="utf-8")


def function(name):
    match = re.search(r"(?:static\s+)?(?:void|bool_t|int8_t|uint32_t|const char\s*\*)\s*" + name + r"\([^;{}]*\)\s*\{", source)
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
#include <stddef.h>
#include <string.h>
#include <math.h>
#include <assert.h>
#define __ZEPHYR__ 1
#define CALI_LIST_LENGTH 5
#define CALI_GIMBAL 1
#define CALI_GYRO 2
#define CALI_SENSOR_HEAD_LEGHT 1
#define CALIED_FLAG 0x55
#define CALI_FUNC_CMD_INIT 0
#define ROBOT_TASK_BUILD_SINGLE_GIMBAL 1
#define FLASH_WRITE_BUF_LENGTH 120
typedef int bool_t;
typedef struct { uint16_t yaw_offset, pitch_offset; float yaw_max_angle, yaw_min_angle, pitch_max_angle, pitch_min_angle; } GimbalCali;
typedef struct { uint8_t name[3], cali_done, flash_len, cali_cmd; uint32_t *flash_buf; bool_t (*cali_hook)(uint32_t *, bool_t); } Sensor;
typedef struct { uint32_t magic, version, sequence; uint8_t payload[120]; uint32_t checksum; } CaliFileRecord;
typedef struct { unsigned int slot; } FIL;
typedef unsigned int UINT;
typedef int FRESULT;
#define FR_OK 0
#define FR_NO_FILE 1
#define FR_DISK_ERR 2
#define FA_READ 1
#define FA_WRITE 2
#define FA_CREATE_ALWAYS 4
static CaliFileRecord slots[2];
static int present[2], read_error, short_read, short_write, sync_error, close_error;
static int ready = 1, mounted = 1;
static uint32_t CaliSequence;
static uint8_t CaliReadOk;
static uint8_t flash_write_buf[120];
static uint32_t data[5][6];
static GimbalCali s_gimbal_cal_data;
static Sensor cali_sensor[5];
static uint32_t *cali_sensor_buf[5] = {data[0], (uint32_t *)&s_gimbal_cal_data, data[2], data[3], data[4]};
static uint8_t cali_sensor_size[5] = {2, 5, 6, 6, 6};
static bool_t dummy(uint32_t *data, bool_t cmd) { (void)data; (void)cmd; return 1; }
static void *cali_hook_fun[5] = {dummy, dummy, dummy, 0, 0};
static int allow_restore;
static uint8_t cali_gimbal_profile_enabled(void) { return 1; }
static uint8_t GimbalCalibrationRestore(uint16_t a, uint16_t b, float c, float d, float e, float f) {
    (void)a; (void)b; (void)c; (void)d; (void)e; (void)f; return allow_restore;
}
static const uint8_t cali_name[5][3] = {{'H','D',0},{'G','M',0},{'G','Y','R'},{'A','C','C'},{'M','A','G'}};
static uint8_t CalibrateReady(void) { return ready; }
static int SdcardIsMounted(void) { return mounted; }
static FRESULT f_open(FIL *f, const char *path, int mode) {
    f->slot = path[7] == '1';
    if (mode == FA_READ && !present[f->slot]) return FR_NO_FILE;
    if (mode & FA_WRITE) { present[f->slot] = 1; memset(&slots[f->slot], 0, sizeof(slots[0])); }
    return FR_OK;
}
static FRESULT f_read(FIL *f, void *buf, unsigned int size, UINT *count) {
    memcpy(buf, &slots[f->slot], size); *count = size - (short_read != 0); return read_error ? FR_DISK_ERR : FR_OK;
}
static FRESULT f_write(FIL *f, const void *buf, unsigned int size, UINT *count) {
    memcpy(&slots[f->slot], buf, size); *count = size - (short_write != 0); return FR_OK;
}
static FRESULT f_sync(FIL *f) { (void)f; return sync_error ? FR_DISK_ERR : FR_OK; }
static FRESULT f_close(FIL *f) { (void)f; return close_error ? FR_DISK_ERR : FR_OK; }
'''
harness += "\n".join(function(name) for name in ("CaliChecksum", "CaliPath", "CaliFileRead", "cali_data_read", "cali_data_write", "cali_param_init"))
harness += r'''
int main(void) {
    const unsigned int words[] = {2, 5, 6, 6, 6};
    assert(CaliChecksum("123456789", 9) == 0xcbf43926u);
    for (unsigned int i=0; i<5; i++) { cali_sensor[i].flash_buf=data[i]; cali_sensor[i].flash_len=words[i]; memcpy(cali_sensor[i].name,cali_name[i],3); }
    cali_sensor[1].flash_buf=(uint32_t *)&s_gimbal_cal_data;
    cali_data_read(); assert(CaliReadOk && CaliSequence==0);
    s_gimbal_cal_data.yaw_min_angle=-1; s_gimbal_cal_data.yaw_max_angle=1;
    s_gimbal_cal_data.pitch_min_angle=-1; s_gimbal_cal_data.pitch_max_angle=1;
    cali_sensor[1].cali_done=CALIED_FLAG;
    assert(cali_data_write() && CaliSequence==1);
    allow_restore=0; cali_param_init(); assert(!CaliReadOk);
    allow_restore=1; cali_param_init(); assert(CaliReadOk);
    assert(cali_data_write() && CaliSequence==2);
    slots[0].payload[0] ^= 1;
    CaliSequence=0; cali_data_read(); assert(CaliReadOk && CaliSequence==1);
    slots[1].payload[0] ^= 1;
    cali_data_read(); assert(!CaliReadOk);
    present[0]=present[1]=0; CaliSequence=0;
    short_write=1; assert(!cali_data_write() && CaliSequence==0); short_write=0;
    sync_error=1; assert(!cali_data_write() && CaliSequence==0); sync_error=0;
    close_error=1; assert(!cali_data_write() && CaliSequence==0); close_error=0;
    assert(cali_data_write() && CaliSequence==1);
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
