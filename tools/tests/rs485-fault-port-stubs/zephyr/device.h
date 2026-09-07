#ifndef RS485_FAULT_PORT_TEST_DEVICE_H
#define RS485_FAULT_PORT_TEST_DEVICE_H
struct device { int unused; };
extern struct device Rs485TestClockDevice;
#define STM32_CLOCK_CONTROL_NODE 0
#define DEVICE_DT_GET(node) (&Rs485TestClockDevice)
#endif
