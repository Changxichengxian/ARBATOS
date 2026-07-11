/*
 * SPDX-FileCopyrightText: 2026 Xie Yuhan <2811158416@qq.com>
 * SPDX-License-Identifier: Apache-2.0
 *
 * First published in this repository: 2026-04-06
 * Use of this file is governed by the LICENSE file in the repository root.
 */

#include "ControlMgr.h"

#include <stddef.h>
#include <string.h>

#if !defined(CONTROL_MANAGER_ENTER_CRITICAL) || !defined(CONTROL_MANAGER_EXIT_CRITICAL)
#include "FreeRTOS.h"
#include "task.h"
#endif

#ifndef CONTROL_MANAGER_ENTER_CRITICAL
#define CONTROL_MANAGER_ENTER_CRITICAL() taskENTER_CRITICAL()
#endif

#ifndef CONTROL_MANAGER_EXIT_CRITICAL
#define CONTROL_MANAGER_EXIT_CRITICAL() taskEXIT_CRITICAL()
#endif

typedef struct
{
    ControlController controller[CONTROL_MGR_MAX_CONTROLLERS];
    uint8_t count;
} control_registry_t;

typedef struct
{
    const ControlController *active;
    ControlState state;
    ControlRequest pending_request;
    uint16_t pending_id;
    ControlReason pending_reason;
    ControlReason last_reason;
    ControlResult last_result;
    uint32_t update_count;
    uint32_t transition_count;
    uint32_t reject_count;
    uint32_t reserved_claim_mask;
    uint32_t authority_epoch;
    uint32_t cycle_seq;
    uint8_t update_in_progress;
    uint8_t protected_stop_reason;
    uint8_t grant_active;
} control_domain_state_t;

static control_registry_t s_registry;
static control_domain_state_t s_domain[ControlDomainCount];
static ControlMgrDiag s_diag;
static ControlActuatorAudit s_actuator_audit;
static uint32_t s_registered_actuator_mask[ControlDomainCount];
static uint32_t s_cross_domain_actuator_overlap_mask;
static uint32_t s_active_claim_mask;
/* Reset 不能清零；否则复位前的许可可能在新会话里重新匹配。 */
static uint32_t s_output_epoch_seed;
static uint8_t s_inited;

static uint8_t control_domain_valid(ControlDomain domain)
{
    return ((uint32_t)domain < (uint32_t)ControlDomainCount) ? 1u : 0u;
}

static uint32_t control_output_epoch_next_locked(void)
{
    s_output_epoch_seed++;
    if (s_output_epoch_seed == 0u)
    {
        s_output_epoch_seed++;
    }
    return s_output_epoch_seed;
}

static void control_output_revoke_locked(control_domain_state_t *domain_state)
{
    if (domain_state == NULL)
    {
        return;
    }

    domain_state->grant_active = 0u;
    domain_state->authority_epoch = control_output_epoch_next_locked();
    domain_state->cycle_seq = 0u;
}

static void control_pending_clear_locked(control_domain_state_t *domain_state)
{
    if (domain_state == NULL)
    {
        return;
    }

    domain_state->pending_request = ControlRequestNone;
    domain_state->pending_id = ControlIdNone;
    domain_state->pending_reason = ControlReasonNone;
}

static uint8_t control_stop_priority(ControlReason reason)
{
    if (reason == ControlReasonEmergencyStop)
    {
        return 2u;
    }
    if (reason == ControlReasonFault)
    {
        return 1u;
    }
    return 0u;
}

static uint8_t control_request_priority(ControlRequest request, ControlReason reason)
{
    return (request == ControlRequestStop) ? control_stop_priority(reason) : 0u;
}

static uint32_t control_reserved_claim_mask_locked(void)
{
    uint32_t mask = 0u;

    for (uint8_t i = 0u; i < (uint8_t)ControlDomainCount; i++)
    {
        mask |= s_domain[i].reserved_claim_mask;
    }
    return mask;
}

static uint32_t control_registered_actuator_mask_locked(void)
{
    uint32_t mask = 0u;

    for (uint8_t i = 0u; i < (uint8_t)ControlDomainCount; i++)
    {
        mask |= s_registered_actuator_mask[i];
    }
    return mask;
}

static uint32_t control_registered_actuator_mask_without_domain_locked(ControlDomain domain)
{
    uint32_t mask = 0u;

    for (uint8_t i = 0u; i < (uint8_t)ControlDomainCount; i++)
    {
        if (i != (uint8_t)domain)
        {
            mask |= s_registered_actuator_mask[i];
        }
    }
    return mask;
}

static uint32_t control_active_actuator_mask_locked(void)
{
    uint32_t mask = 0u;

    for (uint8_t i = 0u; i < (uint8_t)ControlDomainCount; i++)
    {
        if (s_domain[i].active != NULL)
        {
            mask |= s_domain[i].active->actuator_mask;
        }
    }
    return mask;
}

static void control_diag_register_locked(uint16_t controller_id, ControlResult result)
{
    s_diag.registerAttemptCount++;
    if (result == ControlResultOk)
    {
        return;
    }

    s_diag.registerFailCount++;
    s_diag.lastRegisterErrorId = controller_id;
    s_diag.lastRegisterError = result;
}

static void control_diag_switch_locked(uint16_t controller_id, ControlResult result)
{
    s_diag.switchAttemptCount++;
    if (result == ControlResultOk)
    {
        return;
    }

    s_diag.switchFailCount++;
    s_diag.lastSwitchErrorId = controller_id;
    s_diag.lastSwitchError = result;
}

static ControlResult control_queue_request_locked(control_domain_state_t *domain_state,
                                                  ControlRequest request,
                                                  uint16_t controller_id,
                                                  ControlReason reason)
{
    const uint8_t pending_priority = control_request_priority(domain_state->pending_request,
                                                              domain_state->pending_reason);
    const uint8_t running_priority = control_stop_priority(
        (ControlReason)domain_state->protected_stop_reason);
    const uint8_t incoming_priority = control_request_priority(request, reason);

    if (domain_state->pending_request == request &&
        domain_state->pending_id == controller_id &&
        domain_state->pending_reason == reason)
    {
        return ControlResultOk;
    }

    if (running_priority != 0u)
    {
        if (incoming_priority > running_priority)
        {
            domain_state->protected_stop_reason = (uint8_t)reason;
            return ControlResultOk;
        }

        domain_state->last_result = ControlResultResourceBusy;
        domain_state->reject_count++;
        s_diag.protectedRequestRejectCount++;
        return ControlResultResourceBusy;
    }

    if (pending_priority != 0u && incoming_priority < pending_priority)
    {
        domain_state->last_result = ControlResultResourceBusy;
        domain_state->reject_count++;
        s_diag.protectedRequestRejectCount++;
        return ControlResultResourceBusy;
    }

    domain_state->pending_id = controller_id;
    domain_state->pending_reason = reason;
    domain_state->pending_request = request;
    control_output_revoke_locked(domain_state);
    return ControlResultOk;
}

static ControlReason control_protected_stop_begin(ControlDomain domain, ControlReason reason)
{
    control_domain_state_t *domain_state;
    uint8_t protected_priority;
    uint8_t pending_priority;
    const uint8_t incoming_priority = control_stop_priority(reason);

    if (incoming_priority == 0u || control_domain_valid(domain) == 0u)
    {
        return reason;
    }

    domain_state = &s_domain[domain];
    CONTROL_MANAGER_ENTER_CRITICAL();
    protected_priority = control_stop_priority((ControlReason)domain_state->protected_stop_reason);
    if (protected_priority > incoming_priority)
    {
        reason = (ControlReason)domain_state->protected_stop_reason;
    }
    else
    {
        protected_priority = incoming_priority;
    }

    pending_priority = control_request_priority(domain_state->pending_request,
                                                domain_state->pending_reason);
    if (pending_priority > protected_priority)
    {
        reason = domain_state->pending_reason;
        protected_priority = pending_priority;
    }

    if (domain_state->pending_request != ControlRequestNone)
    {
        control_pending_clear_locked(domain_state);
        if (pending_priority == 0u)
        {
            domain_state->reject_count++;
            s_diag.protectedRequestRejectCount++;
        }
    }
    domain_state->protected_stop_reason = (uint8_t)reason;
    CONTROL_MANAGER_EXIT_CRITICAL();
    return reason;
}

static ControlResult control_update_enter(ControlDomain domain)
{
    control_domain_state_t *domain_state = &s_domain[domain];
    ControlResult result = ControlResultOk;

    CONTROL_MANAGER_ENTER_CRITICAL();
    if (domain_state->update_in_progress != 0u)
    {
        domain_state->last_result = ControlResultResourceBusy;
        domain_state->reject_count++;
        s_diag.updateReentryCount++;
        result = ControlResultResourceBusy;
    }
    else
    {
        domain_state->update_in_progress = 1u;
    }
    CONTROL_MANAGER_EXIT_CRITICAL();
    return result;
}

static void control_update_leave(ControlDomain domain)
{
    CONTROL_MANAGER_ENTER_CRITICAL();
    s_domain[domain].update_in_progress = 0u;
    CONTROL_MANAGER_EXIT_CRITICAL();
}

static ControlCtx *control_context_or_local(ControlCtx *context, ControlCtx *local)
{
    if (context != NULL)
    {
        return context;
    }

    memset(local, 0, sizeof(*local));
    return local;
}

static void control_output_context_clear(ControlCtx *context)
{
    if (context != NULL)
    {
        ControlOutputPermitClear(&context->outputPermit);
    }
}

static void control_output_issue_locked(control_domain_state_t *domain_state,
                                        const ControlController *controller,
                                        ControlCtx *context)
{
    if (domain_state == NULL || controller == NULL || context == NULL ||
        domain_state->active != controller ||
        domain_state->pending_request != ControlRequestNone ||
        domain_state->protected_stop_reason != (uint8_t)ControlReasonNone)
    {
        return;
    }

    domain_state->grant_active = 1u;
    domain_state->cycle_seq++;
    if (domain_state->cycle_seq == 0u)
    {
        /* 1 kHz 连续运行约 49 天会回绕；换代后旧 cycle 仍不能重新匹配。 */
        domain_state->authority_epoch = control_output_epoch_next_locked();
        domain_state->cycle_seq++;
    }

    context->outputPermit.stamp.authorityEpoch = domain_state->authority_epoch;
    context->outputPermit.stamp.cycleSeq = domain_state->cycle_seq;
    context->outputPermit.stamp.controllerId = controller->id;
    context->outputPermit.stamp.domain = (uint8_t)controller->domain;
    context->outputPermit.stamp.valid = 1u;
    context->outputPermit.actuatorMask = controller->actuator_mask;
}

#if defined(CONTROL_MANAGER_TEST)
/* 仅供主机回归把 49 天后的回绕边界压缩到一次 update。 */
uint8_t ControlMgrTestOutputCycleSet(ControlDomain domain, uint32_t cycle_seq)
{
    if (control_domain_valid(domain) == 0u)
    {
        return 0u;
    }

    CONTROL_MANAGER_ENTER_CRITICAL();
    s_domain[domain].cycle_seq = cycle_seq;
    CONTROL_MANAGER_EXIT_CRITICAL();
    return 1u;
}
#endif

static const ControlController *control_find(uint16_t controller_id)
{
    for (uint8_t i = 0u; i < s_registry.count; i++)
    {
        if (s_registry.controller[i].id == controller_id)
        {
            return &s_registry.controller[i];
        }
    }
    return NULL;
}

static const ControlController *control_find_by_name(const char *name)
{
    if (name == NULL)
    {
        return NULL;
    }

    for (uint8_t i = 0u; i < s_registry.count; i++)
    {
        if (s_registry.controller[i].name != NULL &&
            strcmp(s_registry.controller[i].name, name) == 0)
        {
            return &s_registry.controller[i];
        }
    }
    return NULL;
}

static uint8_t control_reason_uses_stop_callback(ControlReason reason)
{
    return (uint8_t)(reason == ControlReasonDisable ||
                     reason == ControlReasonOffline ||
                     reason == ControlReasonFault ||
                     reason == ControlReasonEmergencyStop);
}

static ControlResult control_call_callback(ControlCallback callback,
                                              const ControlController *controller,
                                              ControlCtx *context,
                                              ControlReason reason)
{
    ControlResult result;
    ControlReason saved_reason;

    if (callback == NULL)
    {
        return ControlResultOk;
    }

    saved_reason = context->reason;
    context->reason = reason;
    result = callback(controller, context);
    context->reason = saved_reason;
    return result;
}

static ControlResult control_stop_active(ControlDomain domain,
                                         ControlReason reason,
                                         ControlCtx *context)
{
    control_domain_state_t *domain_state;
    const ControlController *controller;
    ControlCallback callback;
    ControlResult result;

    if (control_domain_valid(domain) == 0u)
    {
        return ControlResultBadArgument;
    }

    domain_state = &s_domain[domain];
    reason = control_protected_stop_begin(domain, reason);
    CONTROL_MANAGER_ENTER_CRITICAL();
    if (domain_state->grant_active != 0u)
    {
        control_output_revoke_locked(domain_state);
    }
    CONTROL_MANAGER_EXIT_CRITICAL();
    control_output_context_clear(context);
    controller = domain_state->active;
    if (controller == NULL)
    {
        CONTROL_MANAGER_ENTER_CRITICAL();
        if (domain_state->state != ControlStateFault)
        {
            domain_state->state = ControlStateStopped;
            domain_state->last_result = ControlResultNotActive;
        }
        domain_state->last_reason = reason;
        domain_state->protected_stop_reason = (uint8_t)ControlReasonNone;
        CONTROL_MANAGER_EXIT_CRITICAL();
        return ControlResultNotActive;
    }

    callback = (control_reason_uses_stop_callback(reason) != 0u) ? controller->stop : controller->exit;
    result = control_call_callback(callback, controller, context, reason);

    CONTROL_MANAGER_ENTER_CRITICAL();
    if (control_stop_priority((ControlReason)domain_state->protected_stop_reason) >
        control_stop_priority(reason))
    {
        reason = (ControlReason)domain_state->protected_stop_reason;
    }
    s_active_claim_mask &= ~controller->claim_mask;
    domain_state->active = NULL;
    domain_state->state = (result == ControlResultOk) ? ControlStateStopped : ControlStateFault;
    domain_state->last_reason = reason;
    domain_state->last_result = result;
    domain_state->transition_count++;
    domain_state->protected_stop_reason = (uint8_t)ControlReasonNone;
    CONTROL_MANAGER_EXIT_CRITICAL();

    return result;
}

static ControlResult control_start_controller(const ControlController *next,
                                              ControlReason reason,
                                              ControlCtx *context)
{
    control_domain_state_t *domain_state;
    uint32_t claims_without_domain;
    uint32_t reserved_claims;
    ControlResult result;

    if (next == NULL)
    {
        return ControlResultNotFound;
    }
    if (control_domain_valid(next->domain) == 0u)
    {
        return ControlResultBadArgument;
    }

    domain_state = &s_domain[next->domain];
    CONTROL_MANAGER_ENTER_CRITICAL();
    if (domain_state->active != NULL && domain_state->active->id == next->id)
    {
        domain_state->last_result = ControlResultOk;
        CONTROL_MANAGER_EXIT_CRITICAL();
        return ControlResultOk;
    }
    claims_without_domain = s_active_claim_mask;
    if (domain_state->active != NULL)
    {
        claims_without_domain &= ~domain_state->active->claim_mask;
    }
    reserved_claims = control_reserved_claim_mask_locked();
    if (domain_state->reserved_claim_mask != 0u ||
        ((claims_without_domain | reserved_claims) & next->claim_mask) != 0u)
    {
        domain_state->last_result = ControlResultResourceBusy;
        domain_state->reject_count++;
        s_diag.claimConflictCount++;
        CONTROL_MANAGER_EXIT_CRITICAL();
        return ControlResultResourceBusy;
    }
    domain_state->reserved_claim_mask = next->claim_mask;
    CONTROL_MANAGER_EXIT_CRITICAL();

    if (domain_state->active != NULL)
    {
        result = control_stop_active(next->domain, reason, context);
        if (result != ControlResultOk)
        {
            CONTROL_MANAGER_ENTER_CRITICAL();
            if (control_request_priority(domain_state->pending_request,
                                         domain_state->pending_reason) != 0u)
            {
                domain_state->last_reason = domain_state->pending_reason;
                control_pending_clear_locked(domain_state);
            }
            domain_state->reserved_claim_mask = 0u;
            domain_state->reject_count++;
            CONTROL_MANAGER_EXIT_CRITICAL();
            return result;
        }
    }

    CONTROL_MANAGER_ENTER_CRITICAL();
    if (control_request_priority(domain_state->pending_request,
                                 domain_state->pending_reason) != 0u)
    {
        domain_state->reserved_claim_mask = 0u;
        domain_state->last_result = ControlResultResourceBusy;
        domain_state->reject_count++;
        s_diag.protectedRequestRejectCount++;
        CONTROL_MANAGER_EXIT_CRITICAL();
        return ControlResultResourceBusy;
    }
    domain_state->reserved_claim_mask = 0u;
    domain_state->active = next;
    domain_state->grant_active = 0u;
    domain_state->state = ControlStateRunning;
    domain_state->last_reason = reason;
    domain_state->last_result = ControlResultOk;
    s_active_claim_mask |= next->claim_mask;
    CONTROL_MANAGER_EXIT_CRITICAL();

    control_output_context_clear(context);
    result = control_call_callback(next->enter, next, context, reason);
    if (result != ControlResultOk)
    {
        const ControlResult enter_result = result;

        /*
         * enter 可能已经初始化并写过输出，也可能在失败前排入急停。
         * 统一走 Fault stop 做残留清理，并保留 enter 的原始错误供诊断。
         */
        (void)control_stop_active(next->domain, ControlReasonFault, context);
        CONTROL_MANAGER_ENTER_CRITICAL();
        domain_state->state = ControlStateFault;
        domain_state->last_result = enter_result;
        domain_state->reject_count++;
        CONTROL_MANAGER_EXIT_CRITICAL();
        return enter_result;
    }

    CONTROL_MANAGER_ENTER_CRITICAL();
    domain_state->transition_count++;
    CONTROL_MANAGER_EXIT_CRITICAL();
    return ControlResultOk;
}

static ControlResult control_apply_pending(ControlDomain domain, ControlCtx *context)
{
    control_domain_state_t *domain_state;
    ControlRequest request;
    uint16_t pending_id;
    ControlReason reason;
    const ControlController *next;

    if (control_domain_valid(domain) == 0u)
    {
        return ControlResultBadArgument;
    }

    domain_state = &s_domain[domain];

    CONTROL_MANAGER_ENTER_CRITICAL();
    request = domain_state->pending_request;
    pending_id = domain_state->pending_id;
    reason = domain_state->pending_reason;
    next = (request == ControlRequestSwitch) ? control_find(pending_id) : NULL;
    if (request == ControlRequestStop && control_stop_priority(reason) != 0u)
    {
        domain_state->protected_stop_reason = (uint8_t)reason;
    }
    control_pending_clear_locked(domain_state);
    CONTROL_MANAGER_EXIT_CRITICAL();

    switch (request)
    {
    case ControlRequestNone:
        return ControlResultOk;
    case ControlRequestStop:
        return control_stop_active(domain, reason, context);
    case ControlRequestSwitch:
        if (next == NULL)
        {
            CONTROL_MANAGER_ENTER_CRITICAL();
            domain_state->last_result = ControlResultNotFound;
            domain_state->reject_count++;
            CONTROL_MANAGER_EXIT_CRITICAL();
            return ControlResultNotFound;
        }
        if (next->domain != domain)
        {
            CONTROL_MANAGER_ENTER_CRITICAL();
            domain_state->last_result = ControlResultDomainMismatch;
            domain_state->reject_count++;
            CONTROL_MANAGER_EXIT_CRITICAL();
            return ControlResultDomainMismatch;
        }
        return control_start_controller(next, reason, context);
    default:
        CONTROL_MANAGER_ENTER_CRITICAL();
        domain_state->last_result = ControlResultBadArgument;
        domain_state->reject_count++;
        CONTROL_MANAGER_EXIT_CRITICAL();
        return ControlResultBadArgument;
    }
}

static void control_reset_state_unlocked(void)
{
    memset(&s_registry, 0, sizeof(s_registry));
    memset(&s_domain, 0, sizeof(s_domain));
    memset(&s_diag, 0, sizeof(s_diag));
    memset(&s_actuator_audit, 0, sizeof(s_actuator_audit));
    memset(s_registered_actuator_mask, 0, sizeof(s_registered_actuator_mask));
    s_cross_domain_actuator_overlap_mask = 0u;
    s_active_claim_mask = 0u;
    for (uint8_t i = 0u; i < (uint8_t)ControlDomainCount; i++)
    {
        s_domain[i].authority_epoch = control_output_epoch_next_locked();
    }
    s_inited = 1u;
}

void ControlMgrReset(void)
{
    CONTROL_MANAGER_ENTER_CRITICAL();
    control_reset_state_unlocked();
    CONTROL_MANAGER_EXIT_CRITICAL();
}

void ControlMgrInit(void)
{
    CONTROL_MANAGER_ENTER_CRITICAL();
    if (s_inited == 0u)
    {
        control_reset_state_unlocked();
    }
    CONTROL_MANAGER_EXIT_CRITICAL();
}

const char *ControlDomainName(ControlDomain domain)
{
    switch (domain)
    {
    case ControlDomainChassis:
        return "domain.chassis";
    case ControlDomainGimbal:
        return "domain.gimbal";
    case ControlDomainShoot:
        return "domain.shoot";
    case ControlDomainArm:
        return "domain.arm";
    case ControlDomainWheelleg:
        return "domain.wheelleg";
    case ControlDomainSystem:
        return "domain.system";
    default:
        return NULL;
    }
}

ControlResult ControlMgrRegister(const ControlController *controller)
{
    ControlResult result = ControlResultOk;
    const uint16_t controller_id = (controller != NULL) ? controller->id : ControlIdNone;

    ControlMgrInit();

    if (controller == NULL ||
        controller->id == ControlIdNone ||
        control_domain_valid(controller->domain) == 0u)
    {
        CONTROL_MANAGER_ENTER_CRITICAL();
        control_diag_register_locked(controller_id, ControlResultBadArgument);
        CONTROL_MANAGER_EXIT_CRITICAL();
        return ControlResultBadArgument;
    }

    CONTROL_MANAGER_ENTER_CRITICAL();
    if (control_find(controller->id) != NULL)
    {
        result = ControlResultDuplicate;
    }
    else if (s_registry.count >= (uint8_t)CONTROL_MGR_MAX_CONTROLLERS)
    {
        result = ControlResultFull;
    }
    else
    {
        const uint32_t other_domain_actuators =
            control_registered_actuator_mask_without_domain_locked(controller->domain);

        s_registry.controller[s_registry.count] = *controller;
        s_cross_domain_actuator_overlap_mask |= controller->actuator_mask & other_domain_actuators;
        s_registered_actuator_mask[controller->domain] |= controller->actuator_mask;
        s_registry.count++;
    }
    control_diag_register_locked(controller_id, result);
    CONTROL_MANAGER_EXIT_CRITICAL();

    return result;
}

uint8_t ControlMgrCount(void)
{
    uint8_t count;

    ControlMgrInit();

    CONTROL_MANAGER_ENTER_CRITICAL();
    count = s_registry.count;
    CONTROL_MANAGER_EXIT_CRITICAL();
    return count;
}

const ControlController *ControlMgrGet(uint8_t index)
{
    const ControlController *controller = NULL;

    ControlMgrInit();

    CONTROL_MANAGER_ENTER_CRITICAL();
    if (index < s_registry.count)
    {
        controller = &s_registry.controller[index];
    }
    CONTROL_MANAGER_EXIT_CRITICAL();

    return controller;
}

const ControlController *ControlMgrFindByName(const char *name)
{
    const ControlController *controller = NULL;

    ControlMgrInit();

    CONTROL_MANAGER_ENTER_CRITICAL();
    controller = control_find_by_name(name);
    CONTROL_MANAGER_EXIT_CRITICAL();

    return controller;
}

uint16_t ControlMgrFindIdByName(const char *name)
{
    const ControlController *controller = ControlMgrFindByName(name);

    return (controller != NULL) ? controller->id : ControlIdNone;
}

const char *ControlInputName(const ControlController *controller, uint8_t index)
{
    if (controller == NULL ||
        controller->meta.inputs == NULL ||
        index >= controller->meta.input_count)
    {
        return NULL;
    }

    return controller->meta.inputs[index];
}

const char *ControlOutputName(const ControlController *controller, uint8_t index)
{
    if (controller == NULL ||
        controller->meta.outputs == NULL ||
        index >= controller->meta.output_count)
    {
        return NULL;
    }

    return controller->meta.outputs[index];
}

uint16_t ControlPeriodMs(const ControlController *controller)
{
    return (controller != NULL) ? controller->meta.period_ms : 0u;
}

uint8_t ControlDue(const ControlController *controller, uint32_t tick_ms)
{
    uint32_t period_ms;
    uint32_t phase_ms;

    if (controller == NULL)
    {
        return 0u;
    }

    period_ms = (uint32_t)controller->meta.period_ms;
    if (period_ms == 0u)
    {
        return 1u;
    }

    phase_ms = (uint32_t)controller->meta.phase_ms % period_ms;
    return (((tick_ms + period_ms - phase_ms) % period_ms) == 0u) ? 1u : 0u;
}

uint8_t ControlInputCount(const ControlController *controller)
{
    return (controller != NULL) ? controller->meta.input_count : 0u;
}

uint8_t ControlOutputCount(const ControlController *controller)
{
    return (controller != NULL) ? controller->meta.output_count : 0u;
}

ControlResult ControlMgrSwitch(uint16_t controller_id, ControlReason reason)
{
    const ControlController *controller;
    ControlResult result;

    ControlMgrInit();
    CONTROL_MANAGER_ENTER_CRITICAL();
    controller = control_find(controller_id);
    if (controller == NULL)
    {
        control_diag_switch_locked(controller_id, ControlResultNotFound);
        CONTROL_MANAGER_EXIT_CRITICAL();
        return ControlResultNotFound;
    }
    if (control_domain_valid(controller->domain) == 0u)
    {
        control_diag_switch_locked(controller_id, ControlResultBadArgument);
        CONTROL_MANAGER_EXIT_CRITICAL();
        return ControlResultBadArgument;
    }

    result = control_queue_request_locked(&s_domain[controller->domain],
                                          ControlRequestSwitch,
                                          controller_id,
                                          reason);
    control_diag_switch_locked(controller_id, result);
    CONTROL_MANAGER_EXIT_CRITICAL();

    return result;
}

ControlResult ControlMgrSwitchByName(const char *name, ControlReason reason)
{
    const uint16_t id = ControlMgrFindIdByName(name);

    if (id == ControlIdNone)
    {
        ControlMgrInit();
        CONTROL_MANAGER_ENTER_CRITICAL();
        control_diag_switch_locked(ControlIdNone, ControlResultNotFound);
        CONTROL_MANAGER_EXIT_CRITICAL();
        return ControlResultNotFound;
    }

    return ControlMgrSwitch(id, reason);
}

ControlResult ControlMgrStop(ControlDomain domain, ControlReason reason)
{
    control_domain_state_t *domain_state;
    ControlResult result;

    ControlMgrInit();
    if (control_domain_valid(domain) == 0u)
    {
        return ControlResultBadArgument;
    }

    domain_state = &s_domain[domain];
    CONTROL_MANAGER_ENTER_CRITICAL();
    result = control_queue_request_locked(domain_state,
                                          ControlRequestStop,
                                          ControlIdNone,
                                          reason);
    CONTROL_MANAGER_EXIT_CRITICAL();

    return result;
}

void ControlMgrStopAll(ControlReason reason)
{
    ControlMgrInit();
    for (uint8_t i = 0u; i < (uint8_t)ControlDomainCount; i++)
    {
        (void)ControlMgrStop((ControlDomain)i, reason);
    }
}

void ControlMgrClearPending(ControlDomain domain)
{
    control_domain_state_t *domain_state;

    if (control_domain_valid(domain) == 0u)
    {
        return;
    }

    ControlMgrInit();
    domain_state = &s_domain[domain];
    CONTROL_MANAGER_ENTER_CRITICAL();
    if (control_request_priority(domain_state->pending_request,
                                 domain_state->pending_reason) != 0u)
    {
        domain_state->last_result = ControlResultResourceBusy;
        domain_state->reject_count++;
        s_diag.protectedRequestRejectCount++;
    }
    else
    {
        control_pending_clear_locked(domain_state);
    }
    CONTROL_MANAGER_EXIT_CRITICAL();
}

static ControlResult control_update_domain(ControlDomain domain,
                                           uint32_t tick_ms,
                                           uint8_t check_due,
                                           ControlCtx *context)
{
    control_domain_state_t *domain_state;
    const ControlController *active;
    ControlCtx local_context;
    ControlResult result;
    ControlResult callback_result;
    uint8_t protected_pending;

    ControlMgrInit();
    control_output_context_clear(context);
    if (control_domain_valid(domain) == 0u)
    {
        return ControlResultBadArgument;
    }

    context = control_context_or_local(context, &local_context);
    if (check_due != 0u)
    {
        context->tick_ms = tick_ms;
    }
    domain_state = &s_domain[domain];

    result = control_update_enter(domain);
    if (result != ControlResultOk)
    {
        return result;
    }

    result = control_apply_pending(domain, context);
    if (result != ControlResultOk && result != ControlResultNotActive)
    {
        goto update_done;
    }

    CONTROL_MANAGER_ENTER_CRITICAL();
    protected_pending = control_request_priority(domain_state->pending_request,
                                                 domain_state->pending_reason);
    CONTROL_MANAGER_EXIT_CRITICAL();
    if (protected_pending != 0u)
    {
        result = control_apply_pending(domain, context);
        if (result == ControlResultOk)
        {
            result = ControlResultNotActive;
        }
        goto update_done;
    }

    active = domain_state->active;
    if (active == NULL)
    {
        result = ControlResultNotActive;
        goto update_done;
    }
    if (check_due != 0u && ControlDue(active, tick_ms) == 0u)
    {
        result = ControlResultNotDue;
        goto update_done;
    }

    CONTROL_MANAGER_ENTER_CRITICAL();
    control_output_issue_locked(domain_state, active, context);
    CONTROL_MANAGER_EXIT_CRITICAL();
    callback_result = control_call_callback(active->update, active, context, ControlReasonNone);
    CONTROL_MANAGER_ENTER_CRITICAL();
    domain_state->update_count++;
    domain_state->last_result = callback_result;
    CONTROL_MANAGER_EXIT_CRITICAL();
    if (callback_result != ControlResultOk)
    {
        (void)control_stop_active(domain, ControlReasonFault, context);
        CONTROL_MANAGER_ENTER_CRITICAL();
        domain_state->state = ControlStateFault;
        domain_state->last_result = callback_result;
        CONTROL_MANAGER_EXIT_CRITICAL();
        result = ControlResultCallbackFailed;
        goto update_done;
    }

    CONTROL_MANAGER_ENTER_CRITICAL();
    protected_pending = control_request_priority(domain_state->pending_request,
                                                 domain_state->pending_reason);
    CONTROL_MANAGER_EXIT_CRITICAL();
    if (protected_pending != 0u)
    {
        result = control_apply_pending(domain, context);
        if (result == ControlResultOk)
        {
            result = ControlResultNotActive;
        }
        goto update_done;
    }

    result = ControlResultOk;

update_done:
    control_update_leave(domain);
    return result;
}

ControlResult ControlMgrUpdateDomain(ControlDomain domain, ControlCtx *context)
{
    return control_update_domain(domain, 0u, 0u, context);
}

ControlResult ControlMgrUpdateAll(ControlCtx *context)
{
    ControlResult first_error = ControlResultOk;

    for (uint8_t i = 0u; i < (uint8_t)ControlDomainCount; i++)
    {
        ControlResult result = ControlMgrUpdateDomain((ControlDomain)i, context);
        if (first_error == ControlResultOk &&
            result != ControlResultOk &&
            result != ControlResultNotActive)
        {
            first_error = result;
        }
    }

    return first_error;
}

ControlResult ControlMgrUpdateDomainDue(ControlDomain domain, uint32_t tick_ms, ControlCtx *context)
{
    return control_update_domain(domain, tick_ms, 1u, context);
}

ControlResult ControlMgrUpdateDueAll(uint32_t tick_ms, ControlCtx *context)
{
    ControlResult first_error = ControlResultOk;

    for (uint8_t i = 0u; i < (uint8_t)ControlDomainCount; i++)
    {
        ControlResult result = ControlMgrUpdateDomainDue((ControlDomain)i, tick_ms, context);
        if (first_error == ControlResultOk &&
            result != ControlResultOk &&
            result != ControlResultNotActive &&
            result != ControlResultNotDue)
        {
            first_error = result;
        }
    }

    return first_error;
}

uint8_t ControlMgrIsActive(uint16_t controller_id)
{
    uint8_t active = 0u;

    ControlMgrInit();
    CONTROL_MANAGER_ENTER_CRITICAL();
    for (uint8_t i = 0u; i < (uint8_t)ControlDomainCount; i++)
    {
        if (s_domain[i].active != NULL && s_domain[i].active->id == controller_id)
        {
            active = 1u;
            break;
        }
    }
    CONTROL_MANAGER_EXIT_CRITICAL();
    return active;
}

uint8_t ControlMgrIsActiveByName(const char *name)
{
    const uint16_t id = ControlMgrFindIdByName(name);

    if (id == ControlIdNone)
    {
        return 0u;
    }

    return ControlMgrIsActive(id);
}

uint16_t ControlMgrActiveId(ControlDomain domain)
{
    uint16_t active_id = ControlIdNone;

    ControlMgrInit();
    if (control_domain_valid(domain) == 0u)
    {
        return ControlIdNone;
    }

    CONTROL_MANAGER_ENTER_CRITICAL();
    if (s_domain[domain].active != NULL)
    {
        active_id = s_domain[domain].active->id;
    }
    CONTROL_MANAGER_EXIT_CRITICAL();
    return active_id;
}

const char *ControlMgrActiveName(ControlDomain domain)
{
    const char *name = NULL;

    ControlMgrInit();
    if (control_domain_valid(domain) == 0u)
    {
        return NULL;
    }

    CONTROL_MANAGER_ENTER_CRITICAL();
    if (s_domain[domain].active != NULL)
    {
        name = s_domain[domain].active->name;
    }
    CONTROL_MANAGER_EXIT_CRITICAL();
    return name;
}

ControlResult ControlMgrGetStatus(ControlDomain domain, ControlStatus *out)
{
    const control_domain_state_t *domain_state;

    ControlMgrInit();
    if (control_domain_valid(domain) == 0u || out == NULL)
    {
        return ControlResultBadArgument;
    }

    CONTROL_MANAGER_ENTER_CRITICAL();
    domain_state = &s_domain[domain];
    memset(out, 0, sizeof(*out));
    out->domain = domain;
    out->state = domain_state->state;
    out->pending_request = domain_state->pending_request;
    out->pending_id = domain_state->pending_id;
    out->last_reason = domain_state->last_reason;
    out->last_result = domain_state->last_result;
    out->update_count = domain_state->update_count;
    out->transition_count = domain_state->transition_count;
    out->reject_count = domain_state->reject_count;
    out->active_claim_mask = s_active_claim_mask;
    if (domain_state->active != NULL)
    {
        out->active = 1u;
        out->active_id = domain_state->active->id;
        out->active_name = domain_state->active->name;
    }
    CONTROL_MANAGER_EXIT_CRITICAL();

    return ControlResultOk;
}

uint32_t ControlMgrActiveClaimMask(void)
{
    uint32_t claim_mask;

    ControlMgrInit();

    CONTROL_MANAGER_ENTER_CRITICAL();
    claim_mask = s_active_claim_mask;
    CONTROL_MANAGER_EXIT_CRITICAL();
    return claim_mask;
}

ControlResult ControlMgrGetDiag(ControlMgrDiag *out)
{
    ControlMgrInit();
    if (out == NULL)
    {
        return ControlResultBadArgument;
    }

    CONTROL_MANAGER_ENTER_CRITICAL();
    *out = s_diag;
    out->reservedClaimMask = control_reserved_claim_mask_locked();
    CONTROL_MANAGER_EXIT_CRITICAL();
    return ControlResultOk;
}

ControlResult ControlMgrSetActuatorAudit(const ControlActuatorAudit *audit)
{
    ControlMgrInit();
    if (audit == NULL)
    {
        return ControlResultBadArgument;
    }

    CONTROL_MANAGER_ENTER_CRITICAL();
    s_actuator_audit = *audit;
    CONTROL_MANAGER_EXIT_CRITICAL();
    return ControlResultOk;
}

uint32_t ControlMgrActiveActuatorMask(void)
{
    uint32_t mask;

    ControlMgrInit();
    CONTROL_MANAGER_ENTER_CRITICAL();
    mask = control_active_actuator_mask_locked();
    CONTROL_MANAGER_EXIT_CRITICAL();
    return mask;
}

ControlResult ControlMgrGetActuatorDiag(ControlActuatorDiag *out)
{
    uint32_t registered_mask;

    ControlMgrInit();
    if (out == NULL)
    {
        return ControlResultBadArgument;
    }

    CONTROL_MANAGER_ENTER_CRITICAL();
    registered_mask = control_registered_actuator_mask_locked();
    memset(out, 0, sizeof(*out));
    out->routableMask = s_actuator_audit.routableMask;
    out->registeredMask = registered_mask;
    out->activeMask = control_active_actuator_mask_locked();
    out->duplicateMask = s_actuator_audit.duplicateMask;
    out->crossDomainOverlapMask = s_cross_domain_actuator_overlap_mask;
    out->unownedMask = s_actuator_audit.routableMask & ~registered_mask;
    out->unroutableMask = registered_mask & ~s_actuator_audit.routableMask;
    out->unresolvedOutputCount = s_actuator_audit.unresolvedOutputCount;
    out->invalidIdCount = s_actuator_audit.invalidIdCount;
    CONTROL_MANAGER_EXIT_CRITICAL();
    return ControlResultOk;
}

uint8_t ControlMgrOutputStampValid(const ControlOutputStamp *stamp,
                                   uint32_t requiredMask)
{
    control_domain_state_t *domain_state;
    const ControlController *active;
    uint8_t valid = 0u;

    ControlMgrInit();
    if (stamp == NULL || stamp->valid != 1u ||
        stamp->domain >= (uint8_t)ControlDomainCount)
    {
        return 0u;
    }

    CONTROL_MANAGER_ENTER_CRITICAL();
    domain_state = &s_domain[stamp->domain];
    active = domain_state->active;
    if (domain_state->grant_active == 1u &&
        active != NULL &&
        active->id == stamp->controllerId &&
        active->domain == (ControlDomain)stamp->domain &&
        domain_state->authority_epoch == stamp->authorityEpoch &&
        domain_state->cycle_seq == stamp->cycleSeq &&
        (active->actuator_mask & requiredMask) == requiredMask)
    {
        valid = 1u;
    }
    CONTROL_MANAGER_EXIT_CRITICAL();
    return valid;
}

uint8_t ControlMgrOutputPermitValid(const ControlOutputPermit *permit,
                                    uint32_t requiredMask)
{
    if (ControlOutputPermitAllows(permit, requiredMask) == 0u)
    {
        return 0u;
    }

    return ControlMgrOutputStampValid(&permit->stamp, requiredMask);
}
