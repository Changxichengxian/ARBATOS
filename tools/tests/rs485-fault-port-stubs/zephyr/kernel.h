#ifndef RS485_FAULT_PORT_TEST_KERNEL_H
#define RS485_FAULT_PORT_TEST_KERNEL_H
extern int Rs485TestInIsr;
static inline int k_is_in_isr(void) { return Rs485TestInIsr; }
#endif
