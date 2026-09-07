/* 宿主回归用的最小平台边界，避免把 Zephyr 调度器带入算法测试。 */
#ifndef CONTROL_RUNTIME_TEST_PLATFORM_H
#define CONTROL_RUNTIME_TEST_PLATFORM_H

#include <stdint.h>

#include "LowCmd.h"

#define BSP_TIME_H
#define INS_Task_H
#define MANUAL_INPUT_H
#define MANUAL_INPUT_SNAPSHOT_H
#define MOTOR_INST_H
#define ROBOT_LIFECYCLE_H
#define ROBOT_MODE_H

#define RC_CH_VALUE_ABS_MAX ((uint16_t)1024)

enum
{
    INPUT_AXIS_CHASSIS_X = 0,
    INPUT_AXIS_CHASSIS_Y,
    INPUT_AXIS_CHASSIS_WZ,
    INPUT_AXIS_GIMBAL_YAW,
    INPUT_AXIS_GIMBAL_PITCH,
    INPUT_AXIS_COUNT
};

enum
{
    INPUT_SW_GIMBAL_MODE = 0,
    INPUT_SW_CHASSIS_MODE,
    INPUT_SW_COUNT
};

enum
{
    ROBOT_TASK_MODULE_CONTROL_CHASSIS = 21,
    ROBOT_TASK_MODULE_CONTROL_GIMBAL = 22,
};

typedef enum
{
    ROBOT_RUN_MODE_FULL = 0,
    ROBOT_RUN_MODE_SINGLE_TASK,
    ROBOT_RUN_MODE_SINGLE_MOTOR,
    ROBOT_RUN_MODE_CALIBRATION,
    ROBOT_RUN_MODE_ENTERTAIN,
} robot_run_mode_e;

typedef enum
{
    ROBOT_RUN_VARIANT_NORMAL = 0,
    ROBOT_RUN_VARIANT_CHASSIS_ONLY,
    ROBOT_RUN_VARIANT_GIMBAL_YAW_ONLY,
    ROBOT_RUN_VARIANT_GIMBAL_YAW_EASY,
    ROBOT_RUN_VARIANT_GIMBAL_PITCH_ONLY,
    ROBOT_RUN_VARIANT_GIMBAL_DUAL,
} robot_run_variant_e;

typedef int RobotTaskModule;

typedef struct
{
    MotorId actuator_id;
    uint8_t enabled;
} MotorInst;

typedef struct
{
    uint8_t enabled;
} MotorRoute;

typedef struct
{
    uint32_t seq;
    uint32_t sample_tick_ms;
    uint32_t age_ms;
    uint32_t publish_drop_count;
    float quat[4];
    float angle[3];
    float gyro[3];
    float accel[3];
    float mag[3];
    float temperature_c;
} InsSnapshot;

#define INS_YAW_ADDRESS_OFFSET 0
#define INS_ROLL_ADDRESS_OFFSET 1
#define INS_PITCH_ADDRESS_OFFSET 2

typedef struct
{
    uint8_t GimbalSafePos;
    uint8_t ChassisSafePos;
} ManualInputSemanticsConfig;

typedef struct
{
    int16_t axis[INPUT_AXIS_COUNT];
    uint8_t sw[INPUT_SW_COUNT];
} ControlInputState;

typedef struct ManualInputSnapshot
{
    ControlInputState control;
    ManualInputSemanticsConfig semantics;
    uint32_t sourceAgeMs;
    uint32_t sourceTimeoutMs;
    uint32_t authoritySeq;
    uint8_t online;
    uint8_t dataValid;
} ManualInputSnapshot;

uint32_t BspTimeGetTickMs(void);
uint8_t ManualInputSnapshotRead(ManualInputSnapshot *out);
uint8_t ControlInputSwitchIsPos(uint16_t raw, uint8_t pos);
uint8_t RobotLifecycleOutputAllowed(void);
robot_run_mode_e robot_mode_current(void);
robot_run_variant_e robot_mode_variant(void);
uint8_t robot_mode_is_single_task(RobotTaskModule module);

uint8_t InsSnapshotRead(InsSnapshot *out);
const MotorInst *MotorInstFindByName(const char *name);
MotorId MotorInstId(const MotorInst *inst);
uint8_t MotorInstEnabled(const MotorInst *inst);
const MotorRoute *MotorRouteFindByMotor(MotorId id);
uint8_t MotorInstSetIdsWithPermit(const MotorId *ids,
                                  const MotorCmd *cmds,
                                  uint8_t count,
                                  const ControlOutputPermit *permit);

#endif
