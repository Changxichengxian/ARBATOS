/* 单任务选择不能借用发射变体给其他任务放开发射输出。 */
#include <stdio.h>
#include "RobotMode.h"

Config g_config;

#define CHECK(condition) do { if (!(condition)) { \
    fprintf(stderr, "FAIL: robot mode at line %d\n", __LINE__); return 1; } } while (0)

int main(void)
{
    g_config.operation.mode = ROBOT_RUN_MODE_SINGLE_TASK;
    g_config.operation.variant = ROBOT_RUN_VARIANT_SHOOT_COMBO;
    g_config.operation.target_task = ROBOT_TASK_MODULE_CLASSIC_CHASSIS;
    CHECK(robot_mode_allow_shoot_fric() == 0u && robot_mode_allow_shoot_trigger() == 0u);
    g_config.operation.target_task = ROBOT_TASK_MODULE_SINGLE_GIMBAL;
    CHECK(robot_mode_allow_shoot_fric() == 0u && robot_mode_allow_shoot_trigger() == 0u);
    g_config.operation.target_task = ROBOT_TASK_MODULE_SHOOT_RM;
    CHECK(robot_mode_allow_shoot_fric() != 0u && robot_mode_allow_shoot_trigger() != 0u);
    g_config.operation.variant = ROBOT_RUN_VARIANT_SHOOT_FRIC_ONLY;
    CHECK(robot_mode_allow_shoot_fric() != 0u && robot_mode_allow_shoot_trigger() == 0u);
    g_config.operation.variant = ROBOT_RUN_VARIANT_SHOOT_TRIGGER_ONLY;
    CHECK(robot_mode_allow_shoot_fric() == 0u && robot_mode_allow_shoot_trigger() != 0u);
    g_config.operation.mode = ROBOT_RUN_MODE_FULL;
    CHECK(robot_mode_allow_shoot_fric() != 0u && robot_mode_allow_shoot_trigger() != 0u);
    g_config.operation.mode = ROBOT_RUN_MODE_CALIBRATION;
    CHECK(robot_mode_allow_shoot_fric() == 0u && robot_mode_allow_shoot_trigger() == 0u);
    g_config.operation.cali_target = ROBOT_CALI_TARGET_GIMBAL;
    CHECK(robot_mode_gimbal_cali() != 0u && robot_mode_gimbal_pitch_cali() == 0u);
    CHECK(robot_mode_allow_chassis() == 0u);
    g_config.operation.cali_target = ROBOT_CALI_TARGET_PITCH;
    CHECK(robot_mode_gimbal_cali() == 0u && robot_mode_gimbal_pitch_cali() != 0u);
    g_config.operation.cali_target = ROBOT_CALI_TARGET_NONE;
    CHECK(robot_mode_gimbal_cali() == 0u);
    g_config.operation.mode = ROBOT_RUN_MODE_SINGLE_MOTOR;
    CHECK(robot_mode_allow_shoot_fric() == 0u && robot_mode_allow_shoot_trigger() == 0u);
    puts("PASS: independent shoot task mode regression");
    return 0;
}
