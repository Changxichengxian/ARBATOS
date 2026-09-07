#ifndef RS485_FAULT_PORT_TEST_CLOCK_H
#define RS485_FAULT_PORT_TEST_CLOCK_H
#include <stdint.h>
struct device;
typedef const void *clock_control_subsys_t;
extern int Rs485TestClockResult;
extern uint32_t Rs485TestClockRate;
int clock_control_get_rate(const struct device *dev, clock_control_subsys_t subsys, uint32_t *rate);
#endif
