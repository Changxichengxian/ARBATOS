#ifndef RESET_EVIDENCE_PORT_TEST_INIT_H
#define RESET_EVIDENCE_PORT_TEST_INIT_H

#define PRE_KERNEL_1 0
#define SYS_INIT(function, level, priority) \
    static void *const reset_evidence_sys_init_ref_##function __attribute__((unused)) = (void *)&function

#endif
