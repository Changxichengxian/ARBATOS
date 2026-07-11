/*
 * SPDX-FileCopyrightText: 2026 陈轩 <2811158416@qq.com>
 * SPDX-License-Identifier: Apache-2.0
 *
 * First published in this repository: 2026-04-06
 * Use of this file is governed by the LICENSE file in the repository root.
 */


  
#ifndef DETECT_TASK_H
#define DETECT_TASK_H
#include "Types.h"


#define DETECT_TASK_INIT_TIME 57
#define DETECT_CONTROL_TIME 10

//错误码以及对应设备顺序
typedef enum
{
    DBUS_TOE = 0,
    CHASSIS_MOTOR1_TOE,
    CHASSIS_MOTOR2_TOE,
    CHASSIS_MOTOR3_TOE,
    CHASSIS_MOTOR4_TOE,
    YAW_GIMBAL_MOTOR_TOE,
    PITCH_GIMBAL_MOTOR_TOE,
    TRIGGER_MOTOR_TOE,
    BOARD_GYRO_TOE,
    BOARD_ACCEL_TOE,
    BOARD_MAG_TOE,
    REFEREE_TOE,
    RM_IMU_TOE,
    OLED_TOE,
    DETECT_ERROR_COUNT,
} DetectErrorIndex;

typedef struct
{
    uint32_t new_time;
    uint32_t last_time;
    uint32_t lost_time;
    uint32_t work_time;
    uint16_t set_offline_time : 12;
    uint16_t set_online_time : 12;
    uint8_t enable : 1;
    uint8_t priority : 4;
    uint8_t error_exist : 1;
    uint8_t is_lost : 1;
    uint8_t data_is_error : 1;

    fp32 frequency;
    bool_t (*data_is_error_fun)(void);
    void (*solve_lost_fun)(void);
    void (*solve_data_error_fun)(void);
} DetectError;

/*
 * DetectTask 是健康状态的唯一计算者。接收中断只更新时间事实，任务按固定周期
 * 发布这一份带代次和时间的完整快照；普通查询不得顺便推进掉线状态。
 */
typedef struct
{
    uint32_t newTimeMs;
    uint32_t lastTimeMs;
    uint32_t lostTimeMs;
    uint32_t workTimeMs;
    uint16_t offlineTimeMs;
    uint16_t onlineTimeMs;
    fp32 frequencyHz;
    uint8_t enable;
    uint8_t priority;
    uint8_t errorExist;
    uint8_t isLost;
    uint8_t dataIsError;
    uint8_t reserved0;
    uint8_t reserved1;
    uint8_t reserved2;
} DetectState;

typedef struct
{
    uint32_t seq;
    uint32_t publishTimeMs;
    uint16_t errorMask;
    uint16_t lostMask;
    uint16_t dataErrorMask;
    uint8_t valid;
    uint8_t reserved0;
    DetectState state[DETECT_ERROR_COUNT];
} DetectSnapshot;

typedef struct
{
    uint32_t seq;
    uint32_t publishTimeMs;
    uint16_t errorMask;
    uint16_t lostMask;
    uint16_t dataErrorMask;
    uint8_t valid;
    uint8_t reserved0;
} DetectSummary;

/**
  * @brief          detect task
  * @param[in]      pvParameters: NULL
  * @retval         none
  */
/**
  * @brief          检测任务
  * @param[in]      pvParameters: NULL
  * @retval         none
  */
extern void DetectTask(void const *pvParameters);
extern void HealthMonitorTask(void const *pvParameters);

/**
  * @brief          get toe error status
  * @param[in]      toe: table of equipment
  * @retval         true (error) or false (no error)
  */
/**
  * @brief          获取设备对应的错误状态
  * @param[in]      toe:设备目录
  * @retval         true(错误) 或者false(没错误)
  */
extern bool_t DetectIsError(uint8_t err);
extern uint8_t DetectSnapshotRead(DetectSnapshot *out);
extern uint8_t DetectSummaryRead(DetectSummary *out);

/**
  * @brief          record the time
  * @param[in]      toe: table of equipment
  * @retval         none
  */
/**
  * @brief          记录时间
  * @param[in]      toe:设备目录
  * @retval         none
  */
extern void DetectHook(uint8_t toe);

#endif
