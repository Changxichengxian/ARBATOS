#ifndef MANUAL_INPUT_TEST_ROBOT_CONFIG_H
#define MANUAL_INPUT_TEST_ROBOT_CONFIG_H

#include <stdint.h>

#include "Types.h"

#define MANUAL_INPUT_SRC_AUTO  0u
#define MANUAL_INPUT_SRC_DBUS  1u
#define MANUAL_INPUT_SRC_ELRS  2u
#define MANUAL_INPUT_SRC_IMAGE 3u
#define MANUAL_INPUT_SRC_USB   4u
#define MANUAL_INPUT_SRC_MAX   MANUAL_INPUT_SRC_USB

#define MANUAL_INPUT_MIX_SELECT_LATEST 0u
#define MANUAL_INPUT_MIX_MERGE         1u

#define MANUAL_INPUT_SWITCH_POS_UP   0u
#define MANUAL_INPUT_SWITCH_POS_MID  1u
#define MANUAL_INPUT_SWITCH_POS_DOWN 2u
#define MANUAL_INPUT_SWITCH_POS_MAX  MANUAL_INPUT_SWITCH_POS_DOWN

typedef enum
{
    INPUT_AXIS_CHASSIS_X = 0,
    INPUT_AXIS_CHASSIS_Y,
    INPUT_AXIS_CHASSIS_WZ,
    INPUT_AXIS_GIMBAL_YAW,
    INPUT_AXIS_GIMBAL_PITCH,
    INPUT_AXIS_CALIB_0,
    INPUT_AXIS_CALIB_1,
    INPUT_AXIS_CALIB_2,
    INPUT_AXIS_CALIB_3,
    INPUT_AXIS_COUNT
} input_axis_e;

typedef enum
{
    INPUT_SW_GIMBAL_MODE = 0,
    INPUT_SW_CHASSIS_MODE,
    INPUT_SW_SHOOT_MODE,
    INPUT_SW_CALIB_L,
    INPUT_SW_CALIB_R,
    INPUT_SW_COUNT
} input_switch_e;

typedef struct
{
    uint8_t rc_ch;
    uint8_t invert;
} input_axis_map_t;

typedef struct
{
    uint8_t rc_sw;
    uint8_t invert;
} input_switch_map_t;

typedef struct
{
    uint8_t ElrsChMap[5];
    uint8_t ElrsSwMap[2];
    input_axis_map_t axis[INPUT_AXIS_COUNT];
    input_switch_map_t sw[INPUT_SW_COUNT];
} input_config_t;

typedef struct
{
    uint8_t GimbalSafePos;
    uint8_t ChassisSafePos;
    uint8_t ChassisFollowPos;
    uint8_t ChassisSpinPos;
    uint8_t ShootStopPos;
    uint8_t ShootReadyPos;
    uint8_t ShootFirePos;
    uint8_t image_vt13_shoot_switch_input;
} ManualInputSemanticsConfig;

typedef struct
{
    uint8_t switch1_safe_value;
    uint8_t switch1_normal_value;
    uint8_t switch1_spin_value;
    uint8_t switch2_pause_pos;
    uint8_t switch2_btn_l_pos;
    uint8_t switch2_btn_r_pos;
    uint8_t auto_aim_pause_enable;
    uint8_t auto_aim_mouse_r_enable;
    uint8_t AuxFireBtnLEnable;
    uint8_t AuxFireMouseLEnable;
} ManualInputVt13Config;

typedef struct
{
    uint8_t active_source;
    uint8_t mix_mode;
    uint16_t source_timeout_ms;
    ManualInputSemanticsConfig semantics;
    uint16_t BoardKeyKeyMask;
    ManualInputVt13Config vt13;
} ManualInputConfig;

typedef struct
{
    ManualInputConfig manual_input;
    input_config_t input;
} Config;

extern Config g_config;

#endif
