/*
 * SPDX-FileCopyrightText: 2026 Xie Yuhan <2811158416@qq.com>
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef WHEELLEG_MIT_TASK_H
#define WHEELLEG_MIT_TASK_H

#include <stdint.h>

#include "Types.h"

#ifdef __cplusplus
extern "C" {
#endif

void WheelLegMitTask(void const *pvParameters);
uint8_t WheelLegMitGetFootTestPhase(void);
uint8_t WheelLegMitGetFootTestIkOk(void);
void WheelLegMitGetFootTestTarget(uint8_t side, fp32 *x_m, fp32 *y_m, fp32 *length_m);
void WheelLegMitGetFootTestWheel(uint8_t side,
                                      uint8_t *zero_valid,
                                      fp32 *zero_rad,
                                      fp32 *dx_m,
                                      fp32 *comp_rad,
                                      fp32 *target_rad);

#ifdef __cplusplus
}
#endif

#endif
