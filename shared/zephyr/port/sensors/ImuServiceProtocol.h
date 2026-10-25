#ifndef IMU_SERVICE_PROTOCOL_H
#define IMU_SERVICE_PROTOCOL_H

#include <stdint.h>
#include <string.h>

typedef enum { ImuCmdNone, ImuCmdWatch, ImuCmdStop, ImuCmdCalibrate, ImuCmdCancel } ImuCommand;

/* 固定短命令，拒绝溢出、尾随字符与零编号；串口分包由接收端组行。 */
static inline ImuCommand ImuCommandParse(const char *line, uint32_t *id)
{
    if (strncmp(line, "!IMU2,", 6) != 0) return ImuCmdNone;
    const char *p = line + 6;
    ImuCommand command;
    if (strncmp(p, "watch,", 6) == 0) { command = ImuCmdWatch; p += 6; }
    else if (strncmp(p, "stop,", 5) == 0) { command = ImuCmdStop; p += 5; }
    else if (strncmp(p, "calibrate,", 10) == 0) { command = ImuCmdCalibrate; p += 10; }
    else if (strncmp(p, "cancel,", 7) == 0) { command = ImuCmdCancel; p += 7; }
    else return ImuCmdNone;
    uint32_t value = 0;
    if (*p < '0' || *p > '9') return ImuCmdNone;
    while (*p >= '0' && *p <= '9') {
        uint32_t digit = (uint32_t)(*p++ - '0');
        if (value > (UINT32_MAX - digit) / 10u) return ImuCmdNone;
        value = value * 10u + digit;
    }
    if (*p != '\0' || value == 0u) return ImuCmdNone;
    *id = value;
    return command;
}

#endif
