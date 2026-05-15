/*
 * SPDX-FileCopyrightText: 2026 Xie Yuhan <2811158416@qq.com>
 * SPDX-License-Identifier: PolyForm-Noncommercial-1.0.0
 * Required Notice: Copyright 2026 Xie Yuhan <2811158416@qq.com>
 *
 * First published in this repository: 2026-04-06
 * Use of this file is governed by the LICENSE file in the repository root.
 */

#ifndef CONTROL_MANAGER_H
#define CONTROL_MANAGER_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#ifndef CONTROL_MANAGER_MAX_CONTROLLERS
#define CONTROL_MANAGER_MAX_CONTROLLERS 16u
#endif

/*
 * If switch/stop requests are made from multiple tasks, the project can define
 * CONTROL_MANAGER_ENTER_CRITICAL() and CONTROL_MANAGER_EXIT_CRITICAL() before
 * compiling control_manager.c. The default keeps this module OS-independent.
 */

typedef enum
{
    CONTROL_DOMAIN_CHASSIS = 0,
    CONTROL_DOMAIN_GIMBAL,
    CONTROL_DOMAIN_SHOOT,
    CONTROL_DOMAIN_ARM,
    CONTROL_DOMAIN_WHEELLEG,
    CONTROL_DOMAIN_SYSTEM,
    CONTROL_DOMAIN__COUNT
} control_domain_e;

typedef enum
{
    CONTROL_CONTROLLER_NONE = 0,
    CONTROL_CONTROLLER_CLASSIC_CHASSIS,
    CONTROL_CONTROLLER_SINGLE_GIMBAL,
    CONTROL_CONTROLLER_SHOOT,
    CONTROL_CONTROLLER_ARM_MOTION,
    CONTROL_CONTROLLER_WHEELLEG_SERVO_CALIBRATION,
    CONTROL_CONTROLLER_WHEELLEG_SERVO_BALANCE,
    CONTROL_CONTROLLER_WHEELLEG_MIT_CALIBRATION,
    CONTROL_CONTROLLER_WHEELLEG_MIT_STANDUP,
    CONTROL_CONTROLLER_WHEELLEG_MIT_BALANCE,
    CONTROL_CONTROLLER_WHEELLEG_MIT_JUMP,
    CONTROL_CONTROLLER_WHEELLEG_MIT_RECOVERY,
    CONTROL_CONTROLLER_CUSTOM_BASE = 128
} control_controller_id_e;

typedef enum
{
    CONTROL_REASON_NONE = 0,
    CONTROL_REASON_STARTUP,
    CONTROL_REASON_PROFILE,
    CONTROL_REASON_MODE_SWITCH,
    CONTROL_REASON_CALIBRATION,
    CONTROL_REASON_TEST,
    CONTROL_REASON_DISABLE,
    CONTROL_REASON_OFFLINE,
    CONTROL_REASON_FAULT,
    CONTROL_REASON_EMERGENCY_STOP,
} control_transition_reason_e;

typedef enum
{
    CONTROL_STATE_EMPTY = 0,
    CONTROL_STATE_STOPPED,
    CONTROL_STATE_RUNNING,
    CONTROL_STATE_FAULT,
} control_state_e;

typedef enum
{
    CONTROL_RESULT_OK = 0,
    CONTROL_RESULT_BAD_ARGUMENT,
    CONTROL_RESULT_FULL,
    CONTROL_RESULT_DUPLICATE,
    CONTROL_RESULT_NOT_FOUND,
    CONTROL_RESULT_DOMAIN_MISMATCH,
    CONTROL_RESULT_RESOURCE_BUSY,
    CONTROL_RESULT_NOT_ACTIVE,
    CONTROL_RESULT_CALLBACK_FAILED,
} control_result_e;

typedef enum
{
    CONTROL_REQUEST_NONE = 0,
    CONTROL_REQUEST_SWITCH,
    CONTROL_REQUEST_STOP,
} control_request_e;

enum
{
    CONTROL_RESOURCE_CHASSIS_WHEELS = (1ul << 0),
    CONTROL_RESOURCE_GIMBAL_YAW = (1ul << 1),
    CONTROL_RESOURCE_GIMBAL_PITCH = (1ul << 2),
    CONTROL_RESOURCE_SHOOT_TRIGGER = (1ul << 3),
    CONTROL_RESOURCE_SHOOT_FRICTION = (1ul << 4),
    CONTROL_RESOURCE_ARM = (1ul << 5),
    CONTROL_RESOURCE_WHEELLEG_LEFT_LEG = (1ul << 6),
    CONTROL_RESOURCE_WHEELLEG_RIGHT_LEG = (1ul << 7),
    CONTROL_RESOURCE_WHEELLEG_LEFT_WHEEL = (1ul << 8),
    CONTROL_RESOURCE_WHEELLEG_RIGHT_WHEEL = (1ul << 9),
};

typedef struct
{
    float dt_s;
    uint32_t tick_ms;
    uint32_t flags;
    control_transition_reason_e reason;
    void *input;
    void *output;
} control_context_t;

struct control_controller;
typedef control_result_e (*control_controller_callback_t)(const struct control_controller *controller,
                                                          control_context_t *context);

typedef struct control_controller
{
    uint16_t id;
    control_domain_e domain;
    uint32_t claim_mask;
    const char *name;
    control_controller_callback_t enter;
    control_controller_callback_t update;
    control_controller_callback_t exit;
    control_controller_callback_t stop;
    void *user;
} control_controller_t;

typedef struct
{
    uint8_t active;
    uint16_t active_id;
    const char *active_name;
    control_domain_e domain;
    control_state_e state;
    control_request_e pending_request;
    uint16_t pending_id;
    uint32_t active_claim_mask;
    control_transition_reason_e last_reason;
    control_result_e last_result;
    uint32_t update_count;
    uint32_t transition_count;
    uint32_t reject_count;
} control_domain_status_t;

void control_manager_init(void);
void control_manager_reset(void);

control_result_e control_manager_register(const control_controller_t *controller);
uint8_t control_manager_registered_count(void);

control_result_e control_manager_request_switch(uint16_t controller_id, control_transition_reason_e reason);
control_result_e control_manager_request_stop(control_domain_e domain, control_transition_reason_e reason);
void control_manager_request_stop_all(control_transition_reason_e reason);
void control_manager_clear_pending(control_domain_e domain);

control_result_e control_manager_update_domain(control_domain_e domain, control_context_t *context);
control_result_e control_manager_update_all(control_context_t *context);

uint8_t control_manager_is_active(uint16_t controller_id);
uint16_t control_manager_active_id(control_domain_e domain);
control_result_e control_manager_get_domain_status(control_domain_e domain, control_domain_status_t *out);
uint32_t control_manager_active_claim_mask(void);

#ifdef __cplusplus
}
#endif

#endif
