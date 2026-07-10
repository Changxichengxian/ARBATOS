#ifndef MANUAL_INPUT_TEST_SD_LOG_H
#define MANUAL_INPUT_TEST_SD_LOG_H

#include <stdint.h>

#include "ManualInputProtocol.h"

#define SDLOG_TAG_MANUAL_INPUT_RAW              0x0046u
#define SDLOG_TAG_EVENT                         0x0042u
#define SDLOG_EVT_MANUAL_SOURCE_SWITCH          44u
#define SDLOG_MANUAL_INPUT_PROTO_DBUS           MANUAL_INPUT_PROTOCOL_DBUS
#define SDLOG_MANUAL_INPUT_PROTO_CRSF           MANUAL_INPUT_PROTOCOL_CRSF
#define SDLOG_MANUAL_INPUT_PROTO_IMAGE_CUSTOM   MANUAL_INPUT_PROTOCOL_IMAGE_CUSTOM
#define SDLOG_MANUAL_INPUT_PROTO_IMAGE_VT13     MANUAL_INPUT_PROTOCOL_IMAGE_VT13
#define SDLOG_MANUAL_INPUT_RANGE_RAW_11BIT      1u
#define SDLOG_MANUAL_INPUT_RANGE_CENTERED_660   2u
#define SDLOG_MANUAL_INPUT_RANGE_CENTERED_1024  3u

typedef struct
{
    uint8_t source;
    uint8_t proto;
    uint8_t range_mode;
    uint8_t channel_count;
    int16_t ch_raw[16];
    uint8_t sw[2];
    uint8_t mouse_btns;
    uint8_t reserved0;
    int16_t mouse_x;
    int16_t mouse_y;
    int16_t mouse_z;
    uint16_t key_value;
} sdlog_manual_input_raw_t;

typedef struct
{
    uint16_t event_id;
    uint16_t arg0_u16;
    uint32_t arg1_u32;
    uint32_t arg2_u32;
} sdlog_event_t;

typedef struct
{
    uint32_t last_rx_tick_ms;
    uint32_t frame_count;
    uint32_t controller_frame_count;
    uint32_t client_frame_count;
    uint32_t vt13_frame_count;
    uint32_t crc_error_count;
    uint32_t parse_error_count;
    uint32_t restart_count;
    uint16_t last_cmd_id;
    uint8_t port_active;
    uint8_t last_range_mode;
} sdlog_image_link_stats_t;

uint8_t SdLogIsActive(void);
void SdLogWrite(uint16_t tag, const void *payload, uint16_t payload_size);

#endif
