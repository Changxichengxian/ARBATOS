#ifndef SD_BENCH_H
#define SD_BENCH_H

#include <stdint.h>

#define SD_BENCH_MAGIC 0x5344424Du
#define SD_BENCH_VERSION 1u
#define SD_BENCH_CASE_COUNT 10u
#define SD_BENCH_PHASE_BOOT 0u
#define SD_BENCH_PHASE_READY 1u
#define SD_BENCH_PHASE_WRITE 2u
#define SD_BENCH_PHASE_READ 3u
#define SD_BENCH_PHASE_DONE 4u
#define SD_BENCH_PHASE_FAILED 5u

typedef struct
{
    uint32_t targetHz;
    uint32_t actualHz;
    uint32_t mode;
    uint32_t multiRead;
    uint32_t dataBytes;
    uint32_t writeUs;
    uint32_t syncUs;
    uint32_t readUs;
    uint32_t verifyUs;
    uint32_t wallMs;
    uint32_t writeBps;
    uint32_t readBps;
    uint32_t maxWriteUs;
    uint32_t maxReadUs;
    uint32_t mismatchOffset;
    uint32_t expected;
    uint32_t actual;
    int32_t result;
    uint32_t fsResult;
    uint32_t completedBytes;
    uint32_t crc16;
    uint32_t transferCalls;
    uint32_t transferErrors;
    uint32_t transferTimeouts;
    uint32_t lastHalStatus;
    uint32_t lastHalError;
} SdBenchCase;

typedef struct
{
    uint32_t magic;
    uint32_t version;
    uint32_t size;
    uint32_t phase;
    int32_t error;
    uint32_t currentCase;
    uint32_t completedCases;
    uint32_t fsType;
    uint32_t cardType;
    uint32_t sectorCount;
    int32_t highSpeedResult;
    uint32_t cpuHz;
    uint32_t spiKernelHz;
    uint32_t chunkBytes;
    uint32_t startedMs;
    uint32_t finishedMs;
    uint32_t faultCfsr;
    uint32_t faultHfsr;
    char activeFile[32];
    SdBenchCase cases[SD_BENCH_CASE_COUNT];
} SdBenchDiag;

extern volatile SdBenchDiag SdBenchState;
void SdBenchTask(void *argument);
void SdBenchStop(int32_t error);
uint32_t SdBenchNowUs(void);

#endif
