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
    CAN_CHASSIS_ALL_ID = 0x200,
    CAN_3508_M1_ID = 0x201,
    CAN_3508_M2_ID = 0x202,
    CAN_3508_M3_ID = 0x203,
    CAN_3508_M4_ID = 0x204,

    // CAN1 gimbal group (0x1FF): 0x205/0x206/0x207/0x208
    CAN_YAW_MOTOR_ID = 0x205,
    CAN_PITCH_MOTOR_ID = 0x206,
    CAN_TRIGGER_MOTOR_ID = 0x207,
    CAN_GIMBAL_ALL_ID = 0x1FF,    // 0x1FF broadcast
    CAN_YAW_ALL_ID = 0x2FF,       // 0x2FF broadcast

    // CAN2 friction 3510
    CAN_FRICTION1_ID = 0x201,
    CAN_FRICTION2_ID = 0x202,
    CAN_FRICTION3_ID = 0x203,
    CAN_FRICTION4_ID = 0x204,

    // CAN1 pitch motor (3510 / 6623 position-only)
    CAN_PITCH_3510_ID = 0x206,

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

// CAN1: 0x200 group send (0x201~0x204)
extern void CAN_cmd_chassis(int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4);
// CAN1: 0x1FF send 0x205/0x206/0x207/0x208
extern void CAN_cmd_chassis_1ff(int16_t motor205, int16_t motor206, int16_t motor207, int16_t motor208);
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
// CAN1: pitch 3510 on 0x200 (motor3 slot)
extern void CAN_cmd_pitch_3510(int16_t pitch);
// NOTE: Shared actuator set currents (yaw/pitch/trigger) are in actuator_cmd.h.
// CAN2: four 3510 friction motors
extern void CAN_cmd_friction_3510(int16_t f1, int16_t f2, int16_t f3, int16_t f4);

extern void CAN_cmd_chassis_reset_ID(void);

extern const motor_measure_t *get_yaw_gimbal_motor_measure_point(void);
extern const motor_measure_t *get_yaw_upper_gimbal_motor_measure_point(void);
extern const motor_measure_t *get_pitch_gimbal_motor_measure_point(void);
extern const motor_measure_t *get_trigger_motor_measure_point(void);
extern const motor_measure_t *get_chassis_motor_measure_point(uint8_t i);
extern const motor_measure_t *get_friction_motor_measure_point(uint8_t i);

// Called by can_rx_task: update motor measures / detect / logging.
void CAN_rx_process_frame(uint8_t bus, uint16_t std_id, uint8_t dlc, const uint8_t data[8]);

#endif
