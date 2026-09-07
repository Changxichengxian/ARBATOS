#include "SdBench.h"
#include "main.h"
#include "cmsis_os2.h"
#include "SdSpi.h"
#include "BspSdSpiPort.h"
#include "fatfs/ff.h"
#include <stdio.h>
#include <string.h>

#define SD_BENCH_CHUNK_BYTES 32768u
#define SD_BENCH_COMPARE_BYTES (8u * 1024u * 1024u)
#define SD_BENCH_STRESS_BYTES (32u * 1024u * 1024u)

volatile SdBenchDiag SdBenchState;
static FATFS SdBenchFs;
static FIL SdBenchFile;
static uint32_t SdBenchIo[SD_BENCH_CHUNK_BYTES / sizeof(uint32_t)];
static uint32_t SdBenchExpected[SD_BENCH_CHUNK_BYTES / sizeof(uint32_t)];

static void SdBenchFill(uint32_t *buffer, uint32_t offset, uint32_t seed)
{
    /* 每个位置与每一轮使用不同数据，抓到旧缓存、错扇区和短读。 */
    uint32_t value = offset ^ (seed * 0x9E3779B9u) ^ 0xA56C37D1u;
    for (uint32_t i = 0u; i < SD_BENCH_CHUNK_BYTES / sizeof(uint32_t); i++) {
        value ^= value << 13u;
        value ^= value >> 17u;
        value ^= value << 5u;
        buffer[i] = value;
    }
}

static uint32_t SdBenchRate(uint32_t bytes, uint32_t us)
{
    return us != 0u ? (uint32_t)(((uint64_t)bytes * 1000000u) / us) : 0u;
}

static uint32_t SdBenchPortErrors(volatile SdBenchCase *row)
{
    SdSpiPortBenchStats stats;
    SdSpiPortBenchGetStats(&stats);
    row->transferCalls = stats.transferCalls;
    row->transferErrors = stats.transferErrors;
    row->transferTimeouts = stats.transferTimeouts;
    row->lastHalStatus = stats.lastHalStatus;
    row->lastHalError = stats.lastHalError;
    return stats.transferErrors;
}

static int SdBenchFail(volatile SdBenchCase *row, int error, FRESULT fr)
{
    row->result = error;
    row->fsResult = (uint32_t)fr;
    (void)SdBenchPortErrors(row);
    /* 出错后不再关闭/删除/重挂载，避免继续在不可靠链路上修改目录。 */
    return error;
}

static int SdBenchRun(uint32_t index, uint32_t hz, uint32_t mode, uint32_t multi, uint32_t bytes)
{
    volatile SdBenchCase *row = &SdBenchState.cases[index];
    char path[32];
    FRESULT fr = FR_EXIST;
    UINT count;
    uint32_t start;
    uint32_t elapsed;
    uint32_t begin = HAL_GetTick();
    uint32_t seed = index + 1u;
    uint32_t actualHz = 0u;

    memset((void *)row, 0, sizeof(*row));
    row->targetHz = hz;
    row->mode = mode;
    row->multiRead = multi;
    row->dataBytes = bytes;
    row->crc16 = index == 9u ? 1u : 0u;
    row->mismatchOffset = 0xFFFFFFFFu;
    row->result = -1;
    SdBenchState.currentCase = index;
    if (SdSpiPortBenchConfigure(hz, mode, &actualHz) != 0 || actualHz != hz) {
        return SdBenchFail(row, -201, FR_OK);
    }
    row->actualHz = actualHz;
    SdSpiBenchSetMultiRead((uint8_t)multi);
    (void)SdSpiBenchSetCrc16((uint8_t)row->crc16);
    /* 只创建新文件，不覆盖用户文件、不格式化、不直接写裸扇区。 */
    for (uint32_t attempt = 0u; attempt < 1000u && fr == FR_EXIST; attempt++) {
        (void)snprintf(path, sizeof(path), "0:/SB%06lu.BIN", (unsigned long)(index * 1000u + attempt));
        fr = f_open(&SdBenchFile, path, FA_CREATE_NEW | FA_WRITE);
        if (SdBenchPortErrors(row) != 0u) {
            return SdBenchFail(row, -211, fr);
        }
    }
    if (fr != FR_OK) {
        return SdBenchFail(row, -202, fr);
    }
    memcpy((void *)SdBenchState.activeFile, path, strlen(path) + 1u);
    SdBenchState.phase = SD_BENCH_PHASE_WRITE;
    for (uint32_t offset = 0u; offset < bytes; offset += SD_BENCH_CHUNK_BYTES) {
        SdBenchFill(SdBenchIo, offset, seed);
        start = SdBenchNowUs();
        fr = f_write(&SdBenchFile, SdBenchIo, SD_BENCH_CHUNK_BYTES, &count);
        elapsed = SdBenchNowUs() - start;
        row->writeUs += elapsed;
        if (elapsed > row->maxWriteUs) {
            row->maxWriteUs = elapsed;
        }
        if (fr != FR_OK || count != SD_BENCH_CHUNK_BYTES || SdBenchPortErrors(row) != 0u) {
            return SdBenchFail(row, -203, fr);
        }
        row->completedBytes = offset + count;
    }
    start = SdBenchNowUs();
    fr = f_sync(&SdBenchFile);
    row->syncUs = SdBenchNowUs() - start;
    if (fr != FR_OK || SdBenchPortErrors(row) != 0u) {
        return SdBenchFail(row, -204, fr);
    }
    start = SdBenchNowUs();
    fr = f_close(&SdBenchFile);
    row->syncUs += SdBenchNowUs() - start;
    if (fr != FR_OK || SdBenchPortErrors(row) != 0u) {
        return SdBenchFail(row, -205, fr);
    }
    row->writeBps = SdBenchRate(bytes, row->writeUs + row->syncUs);
    fr = f_open(&SdBenchFile, path, FA_READ);
    if (fr != FR_OK || f_size(&SdBenchFile) != bytes || SdBenchPortErrors(row) != 0u) {
        return SdBenchFail(row, -206, fr);
    }
    SdBenchState.phase = SD_BENCH_PHASE_READ;
    row->completedBytes = 0u;
    for (uint32_t offset = 0u; offset < bytes; offset += SD_BENCH_CHUNK_BYTES) {
        /* 接收前清空，避免短传输留下上一块恰好相同的数据。 */
        memset(SdBenchIo, 0xA5, sizeof(SdBenchIo));
        start = SdBenchNowUs();
        fr = f_read(&SdBenchFile, SdBenchIo, SD_BENCH_CHUNK_BYTES, &count);
        elapsed = SdBenchNowUs() - start;
        row->readUs += elapsed;
        if (elapsed > row->maxReadUs) {
            row->maxReadUs = elapsed;
        }
        if (fr != FR_OK || count != SD_BENCH_CHUNK_BYTES || SdBenchPortErrors(row) != 0u) {
            return SdBenchFail(row, -207, fr);
        }
        start = SdBenchNowUs();
        SdBenchFill(SdBenchExpected, offset, seed);
        for (uint32_t i = 0u; i < SD_BENCH_CHUNK_BYTES / sizeof(uint32_t); i++) {
            if (SdBenchIo[i] != SdBenchExpected[i]) {
                row->mismatchOffset = offset + i * sizeof(uint32_t);
                row->expected = SdBenchExpected[i];
                row->actual = SdBenchIo[i];
                return SdBenchFail(row, -208, FR_OK);
            }
        }
        row->verifyUs += SdBenchNowUs() - start;
        row->completedBytes = offset + count;
    }
    fr = f_close(&SdBenchFile);
    if (fr != FR_OK || SdBenchPortErrors(row) != 0u) {
        return SdBenchFail(row, -209, fr);
    }
    row->readBps = SdBenchRate(bytes, row->readUs);
    fr = f_unlink(path);
    if (fr != FR_OK || SdBenchPortErrors(row) != 0u) {
        return SdBenchFail(row, -210, fr);
    }
    SdBenchState.activeFile[0] = '\0';
    row->wallMs = HAL_GetTick() - begin;
    row->result = 0;
    SdBenchState.completedCases++;
    return 0;
}

void SdBenchTask(void *argument)
{
    static const uint32_t matrix[7][3] = {
        {6000000u, 0u, 0u},
        {12000000u, 0u, 0u},
        {24000000u, 0u, 0u},
        {24000000u, 1u, 0u},
        {24000000u, 1u, 1u},
        {24000000u, 2u, 1u},
        {48000000u, 2u, 1u},
    };
    uint32_t sectors = 0u;
    uint32_t best = 0u;
    uint32_t actualHz = 0u;
    DWORD freeClusters = 0u;
    FATFS *spaceFs = NULL;
    int result;
    FRESULT fr;
    SdSpiPortBenchStats portStats;
    (void)argument;
    SdBenchState.startedMs = HAL_GetTick();
    SdBenchState.cpuHz = SystemCoreClock;
    SdBenchState.chunkBytes = SD_BENCH_CHUNK_BYTES;
    SdBenchState.highSpeedResult = -999;
    fr = f_mount(&SdBenchFs, "0:", 1u);
    SdSpiPortBenchGetStats(&portStats);
    if (fr != FR_OK || portStats.transferErrors != 0u) {
        SdBenchStop(-300 - (int32_t)fr);
    }
    SdBenchState.fsType = SdBenchFs.fs_type;
    SdBenchState.cardType = (uint32_t)SdSpiGetCardType();
    SdBenchState.spiKernelHz = HAL_RCCEx_GetPeriphCLKFreq(RCC_PERIPHCLK_SPI123);
    if (SdBenchState.cpuHz != 480000000u || SdBenchState.spiKernelHz != 96000000u) {
        SdBenchStop(-326);
    }
    if (SdSpiGetSectorCount(&sectors) != 0 || sectors == 0u) {
        SdBenchStop(-320);
    }
    SdBenchState.sectorCount = sectors;
    fr = f_getfree("0:", &freeClusters, &spaceFs);
    if (fr != FR_OK || spaceFs == NULL ||
        (uint64_t)freeClusters * spaceFs->csize * 512u < 64u * 1024u * 1024u) {
        /* 最大临时文件32MiB，预留同等余量，低空间时不创建文件。 */
        SdBenchStop(-328);
    }
    SdSpiPortBenchGetStats(&portStats);
    if (portStats.transferErrors != 0u) {
        SdBenchStop(-324);
    }
    SdBenchState.phase = SD_BENCH_PHASE_READY;
    for (uint32_t i = 0u; i < 7u; i++) {
        if (i == 6u) {
            /* 先在24MHz完成CMD6并确认卡接受，再允许端口升到48MHz。 */
            if (SdSpiPortBenchConfigure(24000000u, 1u, &actualHz) != 0 || actualHz != 24000000u) {
                SdBenchStop(-321);
            }
            (void)SdSpiBenchSetCrc16(1u);
            SdBenchState.highSpeedResult = SdSpiBenchHighSpeed();
            (void)SdSpiBenchSetCrc16(0u);
            SdSpiPortBenchGetStats(&portStats);
            if (portStats.transferErrors != 0u) {
                SdBenchStop(-325);
            }
            if (SdBenchState.highSpeedResult != 0) {
                if (SdBenchState.highSpeedResult != SD_SPI_BENCH_HS_UNSUPPORTED &&
                    SdBenchState.highSpeedResult != SD_SPI_BENCH_HS_NOT_SD &&
                    SdBenchState.highSpeedResult != SD_SPI_BENCH_HS_CHECK_BUSY) {
                    /* 只跳过明确不支持/查询时忙的卡，通信或切换异常不能继续写。 */
                    SdBenchStop(-327);
                }
                SdBenchState.cases[i].targetHz = 48000000u;
                SdBenchState.cases[i].result = 1; /* 明确跳过，不伪装成48MHz通过。 */
                break;
            }
        }
        result = SdBenchRun(i, matrix[i][0], matrix[i][1], matrix[i][2], SD_BENCH_COMPARE_BYTES);
        if (result != 0) {
            SdBenchStop(result);
        }
    }
    for (uint32_t i = 1u; i < 7u; i++) {
        const volatile SdBenchCase *candidate = &SdBenchState.cases[i];
        const volatile SdBenchCase *current = &SdBenchState.cases[best];
        if (candidate->result == 0 && candidate->dataBytes == SD_BENCH_COMPARE_BYTES &&
            (uint64_t)candidate->writeUs + candidate->syncUs + candidate->readUs <
            (uint64_t)current->writeUs + current->syncUs + current->readUs) {
            best = i;
        }
    }
    for (uint32_t i = 7u; i < SD_BENCH_CASE_COUNT; i++) {
        result = SdBenchRun(i, matrix[best][0], matrix[best][1], matrix[best][2], SD_BENCH_STRESS_BYTES);
        if (result != 0) {
            SdBenchStop(result);
        }
    }
    /* 完成后降频并卸载，固件停留在结果页；重新上电才会再跑一轮。 */
    if (SdSpiPortBenchConfigure(6000000u, 0u, &actualHz) != 0) {
        SdBenchStop(-322);
    }
    if (f_mount(NULL, "0:", 0u) != FR_OK) {
        SdBenchStop(-323);
    }
    SdBenchState.finishedMs = HAL_GetTick();
    SdBenchState.phase = SD_BENCH_PHASE_DONE;
    for (;;) {
        osDelay(1000u);
    }
}
