/*
 * SPDX-FileCopyrightText: 2026 陈轩 <2811158416@qq.com>
 * SPDX-License-Identifier: PolyForm-Noncommercial-1.0.0
 * Required Notice: Copyright 2026 陈轩 <2811158416@qq.com>
 *
 * First published in this repository: 2026-04-06
 * Use of this file is governed by the LICENSE file in the repository root.
 */

#ifndef CAN_RECEIVE_H
#define CAN_RECEIVE_H

#include "struct_typedef.h"

/* CAN send and receive ID */
typedef enum
{
    CAN_RM_GROUP_0X200_ID = 0x200,
    CAN_RM_GROUP_0X1FF_ID = 0x1FF,
    CAN_RM_GROUP_0X2FF_ID = 0x2FF,
} can_msg_id_e;

// rm motor data
typedef struct
{
    uint16_t ecd;
    int16_t speed_rpm;
    int16_t given_current;
    uint8_t temperate;
    int16_t last_ecd;
} motor_measure_t;

extern void CAN_cmd_rm_group(uint8_t bus,
                             uint16_t group_id,
                             int16_t motor1,
                             int16_t motor2,
                             int16_t motor3,
                             int16_t motor4);
uint8_t CAN_get_last_1ff_status(void);
uint32_t CAN_get_last_can1_error(void);
uint32_t CAN_get_last_can2_error(void);
uint32_t CAN_get_can1_rx_drop_count(void);
uint32_t CAN_get_can2_rx_drop_count(void);
uint32_t CAN_get_can1_tx_count(void);
uint32_t CAN_get_can2_tx_count(void);
uint32_t CAN_get_can1_tx_fail_count(void);
uint32_t CAN_get_can2_tx_fail_count(void);

extern void CAN_cmd_chassis_reset_ID(void);

extern const motor_measure_t *get_yaw_gimbal_motor_measure_point(void);
extern const motor_measure_t *get_yaw_upper_gimbal_motor_measure_point(void);
extern const motor_measure_t *get_pitch_gimbal_motor_measure_point(void);
extern const motor_measure_t *get_trigger_motor_measure_point(void);
extern const motor_measure_t *get_chassis_motor_measure_point(uint8_t i);
extern const motor_measure_t *get_friction_motor_measure_point(uint8_t i);

// Called by can_feedback_rx_task: update motor measures / detect / logging.
void CAN_rx_process_frame(uint8_t bus, uint16_t std_id, uint8_t dlc, const uint8_t data[8]);

#endif
