/*
 * SPDX-FileCopyrightText: 2026 Xie Yuhan <2811158416@qq.com>
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ARM_CORE_H
#define ARM_CORE_H

#include <stdint.h>
#include <string.h>

#include "ControlCore.h"
#include "Types.h"

#ifdef __cplusplus
extern "C" {
#endif

#ifndef ARM_CORE_JOINT_COUNT
#define ARM_CORE_JOINT_COUNT 6u
#endif

typedef enum
{
    ARM_CORE_JOINT_ROLE_NONE = 0u,
    ARM_CORE_JOINT_ROLE_J0,
    ARM_CORE_JOINT_ROLE_MIT_SPEED,
} ArmCoreJointRole;

typedef struct
{
    uint8_t enabled;
    uint8_t role; // ArmCoreJointRole
    int8_t direction;
    uint16_t key_mask;
    fp32 key_speed_rad_s;
    fp32 max_kd;
} ArmCoreJointParam;

typedef struct
{
    uint8_t deadman_hold_ctrl;
    fp32 key_speed_scale;
    fp32 key_kd;
    int16_t j0_current;
    fp32 j0_unitree_key_speed_rad_s;
    fp32 j0_unitree_hold_kd;
    fp32 j0_unitree_drive_kd;
} ArmCoreConfig;

typedef struct
{
    uint16_t key_mask;
    uint8_t ctrl_held;
    uint8_t reverse;
    uint8_t j0_unitree_enabled;
} ArmCoreInput;

typedef struct
{
    uint8_t joint_count;
    uint8_t any_key_active;
    uint8_t mit_deadman_active;
    uint8_t mit_move_key_active;
    uint8_t j0_move_key_active;
    MotorCmd cmd[ARM_CORE_JOINT_COUNT];
} ArmCoreOutput;

static inline uint8_t ArmCoreKeyActive(const ArmCoreJointParam *param, uint16_t key_mask)
{
    if (param == NULL || param->enabled == 0u || param->key_mask == 0u)
    {
        return 0u;
    }

    return ((key_mask & param->key_mask) != 0u) ? 1u : 0u;
}

static inline void ArmCoreStepManual(const ArmCoreConfig *cfg,
                                        const ArmCoreJointParam *joint,
                                        uint8_t joint_count,
                                        const ArmCoreInput *input,
                                        ArmCoreOutput *output)
{
    uint8_t i;
    uint8_t count;
    uint8_t any_key = 0u;
    uint8_t mit_key = 0u;
    fp32 key_speed_scale = 1.0f;
    fp32 key_kd = 0.0f;
    int16_t j0_current_abs = 0;

    if (output == NULL)
    {
        return;
    }

    (void)memset(output, 0, sizeof(*output));
    control_core_cmd_clear_many(output->cmd, ARM_CORE_JOINT_COUNT);

    if (cfg == NULL || joint == NULL || input == NULL)
    {
        return;
    }

    count = (joint_count > ARM_CORE_JOINT_COUNT) ? ARM_CORE_JOINT_COUNT : joint_count;
    output->joint_count = count;
    key_speed_scale = (cfg->key_speed_scale > 0.0f) ? cfg->key_speed_scale : 1.0f;
    key_kd = (cfg->key_kd > 0.0f) ? cfg->key_kd : 0.0f;
    j0_current_abs = control_core_abs_i16(cfg->j0_current);

    for (i = 0u; i < count; i++)
    {
        if (ArmCoreKeyActive(&joint[i], input->key_mask) == 0u)
        {
            continue;
        }

        any_key = 1u;
        if (joint[i].role == (uint8_t)ARM_CORE_JOINT_ROLE_MIT_SPEED)
        {
            mit_key = 1u;
        }
        else if (joint[i].role == (uint8_t)ARM_CORE_JOINT_ROLE_J0)
        {
            output->j0_move_key_active = 1u;
        }
    }

    output->any_key_active = any_key;
    output->mit_move_key_active = mit_key;
    output->mit_deadman_active = (cfg->deadman_hold_ctrl != 0u) ?
        ((input->ctrl_held != 0u && any_key != 0u) ? 1u : 0u) :
        any_key;

    for (i = 0u; i < count; i++)
    {
        const ArmCoreJointParam *param = &joint[i];
        const uint8_t move_key = ArmCoreKeyActive(param, input->key_mask);
        const fp32 reverse_sign = (input->reverse != 0u) ? -1.0f : 1.0f;
        const fp32 direction = (param->direction < 0) ? -1.0f : 1.0f;

        if (param->enabled == 0u)
        {
            continue;
        }

        if (param->role == (uint8_t)ARM_CORE_JOINT_ROLE_J0)
        {
            if (input->j0_unitree_enabled != 0u)
            {
                if (cfg->deadman_hold_ctrl != 0u && input->ctrl_held == 0u)
                {
                    control_core_cmd_set_speed(&output->cmd[i], 0.0f, 0.0f, 0.0f);
                }
                else if (move_key != 0u)
                {
                    control_core_cmd_set_speed(&output->cmd[i],
                                               reverse_sign * cfg->j0_unitree_key_speed_rad_s,
                                               cfg->j0_unitree_drive_kd,
                                               0.0f);
                }
                else
                {
                    control_core_cmd_set_speed(&output->cmd[i], 0.0f, cfg->j0_unitree_hold_kd, 0.0f);
                }
            }
            else if (cfg->deadman_hold_ctrl != 0u && input->ctrl_held == 0u)
            {
                control_core_cmd_set_current(&output->cmd[i], 0);
            }
            else if (move_key != 0u)
            {
                const int16_t current = (input->reverse != 0u) ? (int16_t)(-j0_current_abs) : j0_current_abs;
                control_core_cmd_set_current(&output->cmd[i], current);
            }
            else
            {
                control_core_cmd_set_current(&output->cmd[i], 0);
            }
        }
        else if (param->role == (uint8_t)ARM_CORE_JOINT_ROLE_MIT_SPEED && output->mit_deadman_active != 0u)
        {
            if (move_key != 0u)
            {
                const fp32 kd = control_core_clamp_fp32(key_kd, 0.0f, param->max_kd);
                control_core_cmd_set_speed(&output->cmd[i],
                                           reverse_sign * direction * param->key_speed_rad_s * key_speed_scale,
                                           kd,
                                           0.0f);
            }
            else
            {
                control_core_cmd_set_speed(&output->cmd[i], 0.0f, 0.0f, 0.0f);
            }
        }
    }
}

#ifdef __cplusplus
}
#endif

#endif
