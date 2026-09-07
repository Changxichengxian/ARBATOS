#ifndef SUB_BOARD_BRINGUP_H
#define SUB_BOARD_BRINGUP_H

void SubBoardBringupRunOnce(void);
void SubBoardBringupPoll(void);

#if defined(SUB_BOARD_FACTORY_TEST)
#include <stdint.h>

#define SUB_BOARD_FACTORY_TEST_MAGIC   0x53424654u
#define SUB_BOARD_FACTORY_TEST_VERSION 3u

typedef enum
{
    SUB_BOARD_FACTORY_PHASE_IDLE = 0u,
    SUB_BOARD_FACTORY_PHASE_RTC,
    SUB_BOARD_FACTORY_PHASE_SD,
    SUB_BOARD_FACTORY_PHASE_BUTTONS,
    SUB_BOARD_FACTORY_PHASE_DONE,
    SUB_BOARD_FACTORY_PHASE_FAILED,
} SubBoardFactoryTestPhase;

typedef struct
{
    uint32_t target_hz;
    uint32_t actual_hz;
    uint32_t data_bytes;
    uint32_t write_ms;
    uint32_t sync_ms;
    uint32_t read_ms;
    uint32_t write_bytes_per_s;
    uint32_t read_bytes_per_s;
    int32_t result;
} SubBoardFactorySdSpeedResult;

/* 调试器直接读取这个对象；所有字段只由工厂测试线程更新。 */
typedef struct
{
    uint32_t magic;
    uint16_t version;
    uint16_t size;
    volatile uint32_t phase;
    volatile int32_t error;
    volatile uint32_t started_ms;
    volatile uint32_t finished_ms;
    uint8_t rtc_raw_before[7];
    uint8_t rtc_raw_after[7];
    uint8_t rtc_initial_voltage_low;
    uint8_t rtc_initialized;
    uint8_t rtc_walk_ok;
    uint8_t sd_card_type;
    uint8_t sd_fs_type;
    uint8_t reserved0;
    uint16_t rtc_walk_seconds;
    uint32_t rtc_hal_error;
    uint16_t rtc_year;
    uint8_t rtc_month;
    uint8_t rtc_day;
    uint8_t rtc_hour;
    uint8_t rtc_minute;
    uint8_t rtc_second;
    uint8_t pd14_raw;
    uint8_t pd15_raw;
    uint8_t pd14_stable;
    uint8_t pd15_stable;
    uint8_t buzzer_prompted;
    int32_t buzzer_result;
    uint32_t fault_reason;
    uint32_t fault_arg0;
    uint32_t fault_arg1;
    int32_t sd_probe_result;
    uint32_t sd_sector_count;
    uint32_t pd14_pressed_count;
    uint32_t pd14_released_count;
    uint32_t pd15_pressed_count;
    uint32_t pd15_released_count;
    SubBoardFactorySdSpeedResult sd[3];
} SubBoardFactoryTestStatus;

extern volatile SubBoardFactoryTestStatus g_sub_board_factory_test;
void SubBoardFactoryTestTask(void *argument);
#endif

#endif
