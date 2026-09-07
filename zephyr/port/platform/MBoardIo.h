#ifndef M_BOARD_IO_H
#define M_BOARD_IO_H

#include <stdbool.h>
#include "SdLog.h"

typedef enum { MBoardPowerOut1, MBoardPowerOut2, MBoardPower5V } MBoardPower;
/* 返回 GPIO 操作结果，不代表外接电源电压已测量正常。 */
int MBoardPowerSet(MBoardPower output, bool enabled);
int MBoardRtcRead(SdLogDateTime *out);
/* 本地时间，支持2000～2099年；只有显式校时才改写RTC。 */
int MBoardRtcSet(const SdLogDateTime *time);
void MBoardIoPoll(void);

#endif
