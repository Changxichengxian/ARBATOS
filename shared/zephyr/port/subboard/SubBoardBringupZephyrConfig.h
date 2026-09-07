#ifndef ARB_SUB_BOARD_BRINGUP_ZEPHYR_CONFIG_H
#define ARB_SUB_BOARD_BRINGUP_ZEPHYR_CONFIG_H

#include <zephyr/devicetree.h>

/* `subboard-rtc` points to a PCF8563-compatible I2C child at address 0x51. */
#ifndef ARB_SUBBOARD_RTC_NODE
#if DT_HAS_ALIAS(subboard_rtc)
#define ARB_SUBBOARD_RTC_NODE DT_ALIAS(subboard_rtc)
#endif
#endif

#ifndef ARB_SUBBOARD_RETRY_MS
#define ARB_SUBBOARD_RETRY_MS 1000u
#endif

#endif
