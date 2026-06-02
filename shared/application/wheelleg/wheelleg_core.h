/*
 * SPDX-FileCopyrightText: 2026 Xie Yuhan <2811158416@qq.com>
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef WHEELLEG_CORE_H
#define WHEELLEG_CORE_H

#include <stdint.h>

#include "control_core.h"
#include "wheelleg_msg.h"

#ifdef __cplusplus
extern "C" {
#endif

#define WHEELLEG_CORE_ACTUATOR_COUNT 6u

typedef enum
{
    WHEELLEG_CORE_ACT_RIGHT_FRONT = 0u,
    WHEELLEG_CORE_ACT_RIGHT_BACK,
    WHEELLEG_CORE_ACT_RIGHT_WHEEL,
    WHEELLEG_CORE_ACT_LEFT_FRONT,
    WHEELLEG_CORE_ACT_LEFT_BACK,
    WHEELLEG_CORE_ACT_LEFT_WHEEL,
} wheelleg_core_actuator_e;

typedef struct
{
    wheelleg_cmd_t cmd;
    wheelleg_state_t state;
    fp32 dt_s;
    uint32_t tick_ms;
    uint8_t manual_enabled;
    uint8_t profile_enabled;
    uint8_t test_mode;
} wheelleg_core_input_t;

typedef struct
{
    wheelleg_status_t status;
    wheelleg_debug_t debug;
    actuator_cmd_t actuator[WHEELLEG_CORE_ACTUATOR_COUNT];
    uint8_t fault_flags;
    uint8_t actuator_count;
} wheelleg_core_output_t;

static inline void wheelleg_core_set_joint_torque(wheelleg_core_output_t *out,
                                                  wheelleg_core_actuator_e actuator,
                                                  fp32 torque_nm)
{
    if (out == NULL || (uint8_t)actuator >= WHEELLEG_CORE_ACTUATOR_COUNT)
    {
        return;
    }

    control_core_cmd_set_state_torque(&out->actuator[(uint8_t)actuator],
                                      0.0f,
                                      0.0f,
                                      0.0f,
                                      0.0f,
                                      torque_nm);
}

static inline void wheelleg_core_set_state_torque(wheelleg_core_output_t *out,
                                                  wheelleg_core_actuator_e actuator,
                                                  fp32 position_rad,
                                                  fp32 velocity_radps,
                                                  fp32 kp,
                                                  fp32 kd,
                                                  fp32 torque_nm)
{
    if (out == NULL || (uint8_t)actuator >= WHEELLEG_CORE_ACTUATOR_COUNT)
    {
        return;
    }

    control_core_cmd_set_state_torque(&out->actuator[(uint8_t)actuator],
                                      position_rad,
                                      velocity_radps,
                                      kp,
                                      kd,
                                      torque_nm);
}

static inline void wheelleg_core_output_clear(wheelleg_core_output_t *out)
{
    if (out == NULL)
    {
        return;
    }

    out->fault_flags = 0u;
    out->actuator_count = WHEELLEG_CORE_ACTUATOR_COUNT;
    control_core_cmd_clear_many(out->actuator, WHEELLEG_CORE_ACTUATOR_COUNT);
}

#ifdef __cplusplus
}
#endif

#endif
