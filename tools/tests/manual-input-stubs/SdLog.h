#ifndef MANUAL_INPUT_TEST_SD_LOG_H
#define MANUAL_INPUT_TEST_SD_LOG_H

#include <stdint.h>

#define SDLOG_TAG_MANUAL_INPUT_RAW              0x0046u
#define SDLOG_TAG_EVENT                         0x0042u
#define SDLOG_EVT_MANUAL_SOURCE_SWITCH          44u
#define SDLOG_MANUAL_INPUT_PROTO_DBUS           1u
#define SDLOG_MANUAL_INPUT_RANGE_RAW_11BIT      1u

typedef struct
{
    uint8_t source;
    uint8_t proto;
    uint8_t range_mode;
    uint8_t channel_count;
    int16_t ch_raw[16];
    uint8_t sw[2];
    int16_t mouse_x;
    int16_t mouse_y;
    int16_t mouse_z;
    uint16_t key_value;
    uint8_t mouse_btns;
} sdlog_manual_input_raw_t;

typedef struct
{
    uint16_t event_id;
    uint16_t arg0_u16;
    uint32_t arg1_u32;
    uint32_t arg2_u32;
} sdlog_event_t;

uint8_t SdLogIsActive(void);
void SdLogWrite(uint16_t tag, const void *payload, uint16_t payload_size);

#endif
