#ifndef CAN_BENCH_TEST_CAN_H
#define CAN_BENCH_TEST_CAN_H
#include <stdint.h>
#include <stddef.h>
struct device { int unused; };
extern struct device TestDevice;
#define DEVICE_DT_GET(x) (&TestDevice)
#define DT_ALIAS(x) 0
#define CAN_FRAME_IDE 1u
#define CAN_FRAME_RTR 2u
#define CAN_FRAME_FDF 4u
#define CAN_MODE_NORMAL 0
struct can_frame { uint32_t id; uint8_t dlc, flags, data[8]; };
struct can_filter { uint32_t id, mask; };
enum can_state { CAN_STATE_ERROR_ACTIVE };
struct can_bus_err_cnt { uint8_t tx_err_cnt, rx_err_cnt; };
typedef void (*can_tx_callback_t)(const struct device *, int, void *);
typedef void (*can_rx_callback_t)(const struct device *, struct can_frame *, void *);
int can_send(const struct device *, const struct can_frame *, int, can_tx_callback_t, void *);
static inline int device_is_ready(const struct device *dev) { return dev != NULL; }
static inline int can_set_mode(const struct device *dev, int mode) { (void)dev; (void)mode; return 0; }
static inline int can_set_bitrate(const struct device *dev, uint32_t rate) { (void)dev; (void)rate; return 0; }
static inline int can_add_rx_filter(const struct device *dev, can_rx_callback_t cb, void *arg, const struct can_filter *filter)
{ (void)dev; (void)cb; (void)arg; (void)filter; return 0; }
static inline int can_start(const struct device *dev) { (void)dev; return 0; }
static inline uint32_t can_stats_get_rx_overruns(const struct device *dev) { (void)dev; return 0; }
static inline int can_get_state(const struct device *dev, enum can_state *state, struct can_bus_err_cnt *errors)
{ (void)dev; *state = CAN_STATE_ERROR_ACTIVE; errors->rx_err_cnt = 0; errors->tx_err_cnt = 0; return 0; }
#endif
