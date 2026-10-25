#ifndef TEST_IMU_SERVICE_LOWCMD_H
#define TEST_IMU_SERVICE_LOWCMD_H

#include <stdint.h>

#define LOWCMD_WRITER_MAINTENANCE 241u

uint8_t LowCmdEnterEmergencyStop(uint16_t writer);
uint8_t LowCmdEmergencyActive(void);

#endif
