/*
 * SPDX-FileCopyrightText: 2026 Xie Yuhan <2811158416@qq.com>
 * SPDX-License-Identifier: PolyForm-Noncommercial-1.0.0
 */

#ifndef WHEELLEG_MIT_TASK_H
#define WHEELLEG_MIT_TASK_H

#include <stdint.h>

#include "struct_typedef.h"

#ifdef __cplusplus
extern "C" {
#endif

void wheelleg_mit_task(void const *pvParameters);
uint8_t wheelleg_mit_get_foot_test_phase(void);
uint8_t wheelleg_mit_get_foot_test_ik_ok(void);
void wheelleg_mit_get_foot_test_target(uint8_t side, fp32 *x_m, fp32 *y_m, fp32 *length_m);
void wheelleg_mit_get_foot_test_wheel(uint8_t side,
                                      uint8_t *zero_valid,
                                      fp32 *zero_rad,
                                      fp32 *dx_m,
                                      fp32 *comp_rad,
                                      fp32 *target_rad);

#ifdef __cplusplus
}
#endif

#endif
