#ifndef TEST_IMU_SERVICE_SD_LOG_TASK_H
#define TEST_IMU_SERVICE_SD_LOG_TASK_H

#include <stdint.h>

#define SDLOG_FLASH_PAUSE_REQUEST 0x53445051u
#define SDLOG_FLASH_PAUSE_READY 0x53445244u
#define SDLOG_FLASH_PAUSE_FAILED 0x53444641u

extern volatile uint32_t g_sdLogFlashPauseRequest;
extern volatile uint32_t g_sdLogFlashPauseReady;

#endif
