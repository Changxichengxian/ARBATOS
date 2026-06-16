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
} control_domain_state_t;

static control_registry_t s_registry;
static control_domain_state_t s_domain[ControlDomainCount];
static uint32_t s_active_claim_mask;
static uint8_t s_inited;

static uint8_t control_domain_valid(ControlDomain domain)
{
    return ((uint32_t)domain < (uint32_t)ControlDomainCount) ? 1u : 0u;
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
    controller = domain_state->active;
    if (controller == NULL)
    {
        domain_state->state = ControlStateStopped;
        domain_state->last_result = ControlResultNotActive;
        return ControlResultNotActive;
    }

    callback = (control_reason_uses_stop_callback(reason) != 0u) ? controller->stop : controller->exit;
    result = control_call_callback(callback, controller, context, reason);

    CONTROL_MANAGER_ENTER_CRITICAL();
    s_active_claim_mask &= ~controller->claim_mask;
    domain_state->active = NULL;
    domain_state->state = (result == ControlResultOk) ? ControlStateStopped : ControlStateFault;
    domain_state->last_reason = reason;
    domain_state->last_result = result;
    domain_state->transition_count++;
    CONTROL_MANAGER_EXIT_CRITICAL();

    return result;
}

static ControlResult control_start_controller(const ControlController *next,
                                                 ControlReason reason,
                                                 ControlCtx *context)
{
    control_domain_state_t *domain_state;
    uint32_t claims_without_domain;
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
    if (domain_state->active != NULL && domain_state->active->id == next->id)
    {
        domain_state->last_result = ControlResultOk;
        return ControlResultOk;
    }

    claims_without_domain = s_active_claim_mask;
    if (domain_state->active != NULL)
    {
        claims_without_domain &= ~domain_state->active->claim_mask;
    }

    if ((claims_without_domain & next->claim_mask) != 0u)
    {
        domain_state->last_result = ControlResultResourceBusy;
        domain_state->reject_count++;
        return ControlResultResourceBusy;
    }

    if (domain_state->active != NULL)
    {
        result = control_stop_active(next->domain, reason, context);
        if (result != ControlResultOk)
        {
            domain_state->reject_count++;
            return result;
        }
    }

    CONTROL_MANAGER_ENTER_CRITICAL();
    domain_state->active = next;
    domain_state->state = ControlStateRunning;
    domain_state->last_reason = reason;
    domain_state->last_result = ControlResultOk;
    s_active_claim_mask = claims_without_domain | next->claim_mask;
    CONTROL_MANAGER_EXIT_CRITICAL();

    result = control_call_callback(next->enter, next, context, reason);
    if (result != ControlResultOk)
    {
        CONTROL_MANAGER_ENTER_CRITICAL();
        s_active_claim_mask &= ~next->claim_mask;
        domain_state->active = NULL;
        domain_state->state = ControlStateFault;
        domain_state->last_result = result;
        domain_state->transition_count++;
        domain_state->reject_count++;
        CONTROL_MANAGER_EXIT_CRITICAL();
        return result;
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
    domain_state->pending_request = ControlRequestNone;
    domain_state->pending_id = ControlIdNone;
    domain_state->pending_reason = ControlReasonNone;
    CONTROL_MANAGER_EXIT_CRITICAL();

    switch (request)
    {
    case ControlRequestNone:
        return ControlResultOk;
    case ControlRequestStop:
        return control_stop_active(domain, reason, context);
    case ControlRequestSwitch:
        next = control_find(pending_id);
        if (next == NULL)
        {
            domain_state->last_result = ControlResultNotFound;
            domain_state->reject_count++;
            return ControlResultNotFound;
        }
        if (next->domain != domain)
        {
            domain_state->last_result = ControlResultDomainMismatch;
            domain_state->reject_count++;
            return ControlResultDomainMismatch;
        }
        return control_start_controller(next, reason, context);
    default:
        domain_state->last_result = ControlResultBadArgument;
        domain_state->reject_count++;
        return ControlResultBadArgument;
    }
}

static void control_reset_state_unlocked(void)
{
    memset(&s_registry, 0, sizeof(s_registry));
    memset(&s_domain, 0, sizeof(s_domain));
    s_active_claim_mask = 0u;
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

    ControlMgrInit();

    if (controller == NULL ||
        controller->id == ControlIdNone ||
        control_domain_valid(controller->domain) == 0u)
    {
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
        s_registry.controller[s_registry.count] = *controller;
        s_registry.count++;
    }
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
    control_domain_state_t *domain_state;

    ControlMgrInit();
    controller = control_find(controller_id);
    if (controller == NULL)
    {
        return ControlResultNotFound;
    }
    if (control_domain_valid(controller->domain) == 0u)
    {
        return ControlResultBadArgument;
    }

    domain_state = &s_domain[controller->domain];
    CONTROL_MANAGER_ENTER_CRITICAL();
    domain_state->pending_id = controller_id;
    domain_state->pending_reason = reason;
    domain_state->pending_request = ControlRequestSwitch;
    CONTROL_MANAGER_EXIT_CRITICAL();

    return ControlResultOk;
}

ControlResult ControlMgrSwitchByName(const char *name, ControlReason reason)
{
    const uint16_t id = ControlMgrFindIdByName(name);

    if (id == ControlIdNone)
    {
        return ControlResultNotFound;
    }

    return ControlMgrSwitch(id, reason);
}

ControlResult ControlMgrStop(ControlDomain domain, ControlReason reason)
{
    control_domain_state_t *domain_state;

    ControlMgrInit();
    if (control_domain_valid(domain) == 0u)
    {
        return ControlResultBadArgument;
    }

    domain_state = &s_domain[domain];
    CONTROL_MANAGER_ENTER_CRITICAL();
    domain_state->pending_id = ControlIdNone;
    domain_state->pending_reason = reason;
    domain_state->pending_request = ControlRequestStop;
    CONTROL_MANAGER_EXIT_CRITICAL();

    return ControlResultOk;
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
    if (control_domain_valid(domain) == 0u)
    {
        return;
    }

    ControlMgrInit();
    CONTROL_MANAGER_ENTER_CRITICAL();
    s_domain[domain].pending_request = ControlRequestNone;
    s_domain[domain].pending_id = ControlIdNone;
    s_domain[domain].pending_reason = ControlReasonNone;
    CONTROL_MANAGER_EXIT_CRITICAL();
}

ControlResult ControlMgrUpdateDomain(ControlDomain domain, ControlCtx *context)
{
    control_domain_state_t *domain_state;
    const ControlController *active;
    ControlCtx local_context;
    ControlResult result;

    ControlMgrInit();
    if (control_domain_valid(domain) == 0u)
    {
        return ControlResultBadArgument;
    }

    context = control_context_or_local(context, &local_context);
    domain_state = &s_domain[domain];

    result = control_apply_pending(domain, context);
    if (result != ControlResultOk && result != ControlResultNotActive)
    {
        return result;
    }

    active = domain_state->active;
    if (active == NULL)
    {
        return ControlResultNotActive;
    }

    result = control_call_callback(active->update, active, context, ControlReasonNone);
    domain_state->update_count++;
    domain_state->last_result = result;
    if (result != ControlResultOk)
    {
        (void)control_stop_active(domain, ControlReasonFault, context);
        domain_state->state = ControlStateFault;
        domain_state->last_result = result;
        return ControlResultCallbackFailed;
    }

    return ControlResultOk;
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
    control_domain_state_t *domain_state;
    const ControlController *active;
    ControlCtx local_context;
    ControlResult result;

    ControlMgrInit();
    if (control_domain_valid(domain) == 0u)
    {
        return ControlResultBadArgument;
    }

    context = control_context_or_local(context, &local_context);
    context->tick_ms = tick_ms;
    domain_state = &s_domain[domain];

    result = control_apply_pending(domain, context);
    if (result != ControlResultOk && result != ControlResultNotActive)
    {
        return result;
    }

    active = domain_state->active;
    if (active == NULL)
    {
        return ControlResultNotActive;
    }
    if (ControlDue(active, tick_ms) == 0u)
    {
        return ControlResultNotDue;
    }

    result = control_call_callback(active->update, active, context, ControlReasonNone);
    domain_state->update_count++;
    domain_state->last_result = result;
    if (result != ControlResultOk)
    {
        (void)control_stop_active(domain, ControlReasonFault, context);
        domain_state->state = ControlStateFault;
        domain_state->last_result = result;
        return ControlResultCallbackFailed;
    }

    return ControlResultOk;
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
