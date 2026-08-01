#ifndef ARB_SUB_BOARD_BRINGUP_ZEPHYR_H
#define ARB_SUB_BOARD_BRINGUP_ZEPHYR_H

#include <stdint.h>

typedef struct
{
    uint32_t probe_count;
    uint32_t read_error_count;
    int32_t last_error;
    uint8_t rtc_ready;
    uint8_t rtc_voltage_low;
} SubBoardBringupDiag;

void SubBoardBringupGetDiag(SubBoardBringupDiag *out);

#endif
