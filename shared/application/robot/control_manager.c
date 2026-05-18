/*
 * SPDX-FileCopyrightText: 2026 Xie Yuhan <2811158416@qq.com>
 * SPDX-License-Identifier: PolyForm-Noncommercial-1.0.0
 * Required Notice: Copyright 2026 Xie Yuhan <2811158416@qq.com>
 *
 * First published in this repository: 2026-04-06
 * Use of this file is governed by the LICENSE file in the repository root.
 */

#include "control_manager.h"

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
    control_controller_t controller[CONTROL_MANAGER_MAX_CONTROLLERS];
    uint8_t count;
} control_registry_t;

typedef struct
{
    const control_controller_t *active;
    control_state_e state;
    control_request_e pending_request;
    uint16_t pending_id;
    control_transition_reason_e pending_reason;
    control_transition_reason_e last_reason;
    control_result_e last_result;
    uint32_t update_count;
    uint32_t transition_count;
    uint32_t reject_count;
} control_domain_state_t;

static control_registry_t s_registry;
static control_domain_state_t s_domain[CONTROL_DOMAIN__COUNT];
static uint32_t s_active_claim_mask;
static uint8_t s_inited;

static uint8_t control_domain_valid(control_domain_e domain)
{
    return ((uint32_t)domain < (uint32_t)CONTROL_DOMAIN__COUNT) ? 1u : 0u;
}

static control_context_t *control_context_or_local(control_context_t *context, control_context_t *local)
{
    if (context != NULL)
    {
        return context;
    }

    memset(local, 0, sizeof(*local));
    return local;
}

static const control_controller_t *control_find(uint16_t controller_id)
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

static uint8_t control_reason_uses_stop_callback(control_transition_reason_e reason)
{
    return (uint8_t)(reason == CONTROL_REASON_DISABLE ||
                     reason == CONTROL_REASON_OFFLINE ||
                     reason == CONTROL_REASON_FAULT ||
                     reason == CONTROL_REASON_EMERGENCY_STOP);
}

static control_result_e control_call_callback(control_controller_callback_t callback,
                                              const control_controller_t *controller,
                                              control_context_t *context,
                                              control_transition_reason_e reason)
{
    control_result_e result;
    control_transition_reason_e saved_reason;

    if (callback == NULL)
    {
        return CONTROL_RESULT_OK;
    }

    saved_reason = context->reason;
    context->reason = reason;
    result = callback(controller, context);
    context->reason = saved_reason;
    return result;
}

static control_result_e control_stop_active(control_domain_e domain,
                                            control_transition_reason_e reason,
                                            control_context_t *context)
{
    control_domain_state_t *domain_state;
    const control_controller_t *controller;
    control_controller_callback_t callback;
    control_result_e result;

    if (control_domain_valid(domain) == 0u)
    {
        return CONTROL_RESULT_BAD_ARGUMENT;
    }

    domain_state = &s_domain[domain];
    controller = domain_state->active;
    if (controller == NULL)
    {
        domain_state->state = CONTROL_STATE_STOPPED;
        domain_state->last_result = CONTROL_RESULT_NOT_ACTIVE;
        return CONTROL_RESULT_NOT_ACTIVE;
    }

    callback = (control_reason_uses_stop_callback(reason) != 0u) ? controller->stop : controller->exit;
    result = control_call_callback(callback, controller, context, reason);

    CONTROL_MANAGER_ENTER_CRITICAL();
    s_active_claim_mask &= ~controller->claim_mask;
    domain_state->active = NULL;
    domain_state->state = (result == CONTROL_RESULT_OK) ? CONTROL_STATE_STOPPED : CONTROL_STATE_FAULT;
    domain_state->last_reason = reason;
    domain_state->last_result = result;
    domain_state->transition_count++;
    CONTROL_MANAGER_EXIT_CRITICAL();

    return result;
}

static control_result_e control_start_controller(const control_controller_t *next,
                                                 control_transition_reason_e reason,
                                                 control_context_t *context)
{
    control_domain_state_t *domain_state;
    uint32_t claims_without_domain;
    control_result_e result;

    if (next == NULL)
    {
        return CONTROL_RESULT_NOT_FOUND;
    }
    if (control_domain_valid(next->domain) == 0u)
    {
        return CONTROL_RESULT_BAD_ARGUMENT;
    }

    domain_state = &s_domain[next->domain];
    if (domain_state->active != NULL && domain_state->active->id == next->id)
    {
        domain_state->last_result = CONTROL_RESULT_OK;
        return CONTROL_RESULT_OK;
    }

    claims_without_domain = s_active_claim_mask;
    if (domain_state->active != NULL)
    {
        claims_without_domain &= ~domain_state->active->claim_mask;
    }

    if ((claims_without_domain & next->claim_mask) != 0u)
    {
        domain_state->last_result = CONTROL_RESULT_RESOURCE_BUSY;
        domain_state->reject_count++;
        return CONTROL_RESULT_RESOURCE_BUSY;
    }

    if (domain_state->active != NULL)
    {
        result = control_stop_active(next->domain, reason, context);
        if (result != CONTROL_RESULT_OK)
        {
            domain_state->reject_count++;
            return result;
        }
    }

    CONTROL_MANAGER_ENTER_CRITICAL();
    domain_state->active = next;
    domain_state->state = CONTROL_STATE_RUNNING;
    domain_state->last_reason = reason;
    domain_state->last_result = CONTROL_RESULT_OK;
    s_active_claim_mask = claims_without_domain | next->claim_mask;
    CONTROL_MANAGER_EXIT_CRITICAL();

    result = control_call_callback(next->enter, next, context, reason);
    if (result != CONTROL_RESULT_OK)
    {
        CONTROL_MANAGER_ENTER_CRITICAL();
        s_active_claim_mask &= ~next->claim_mask;
        domain_state->active = NULL;
        domain_state->state = CONTROL_STATE_FAULT;
        domain_state->last_result = result;
        domain_state->transition_count++;
        domain_state->reject_count++;
        CONTROL_MANAGER_EXIT_CRITICAL();
        return result;
    }

    CONTROL_MANAGER_ENTER_CRITICAL();
    domain_state->transition_count++;
    CONTROL_MANAGER_EXIT_CRITICAL();
    return CONTROL_RESULT_OK;
}

static control_result_e control_apply_pending(control_domain_e domain, control_context_t *context)
{
    control_domain_state_t *domain_state;
    control_request_e request;
    uint16_t pending_id;
    control_transition_reason_e reason;
    const control_controller_t *next;

    if (control_domain_valid(domain) == 0u)
    {
        return CONTROL_RESULT_BAD_ARGUMENT;
    }

    domain_state = &s_domain[domain];

    CONTROL_MANAGER_ENTER_CRITICAL();
    request = domain_state->pending_request;
    pending_id = domain_state->pending_id;
    reason = domain_state->pending_reason;
    domain_state->pending_request = CONTROL_REQUEST_NONE;
    domain_state->pending_id = CONTROL_CONTROLLER_NONE;
    domain_state->pending_reason = CONTROL_REASON_NONE;
    CONTROL_MANAGER_EXIT_CRITICAL();

    switch (request)
    {
    case CONTROL_REQUEST_NONE:
        return CONTROL_RESULT_OK;
    case CONTROL_REQUEST_STOP:
        return control_stop_active(domain, reason, context);
    case CONTROL_REQUEST_SWITCH:
        next = control_find(pending_id);
        if (next == NULL)
        {
            domain_state->last_result = CONTROL_RESULT_NOT_FOUND;
            domain_state->reject_count++;
            return CONTROL_RESULT_NOT_FOUND;
        }
        if (next->domain != domain)
        {
            domain_state->last_result = CONTROL_RESULT_DOMAIN_MISMATCH;
            domain_state->reject_count++;
            return CONTROL_RESULT_DOMAIN_MISMATCH;
        }
        return control_start_controller(next, reason, context);
    default:
        domain_state->last_result = CONTROL_RESULT_BAD_ARGUMENT;
        domain_state->reject_count++;
        return CONTROL_RESULT_BAD_ARGUMENT;
    }
}

static void control_reset_state_unlocked(void)
{
    memset(&s_registry, 0, sizeof(s_registry));
    memset(&s_domain, 0, sizeof(s_domain));
    s_active_claim_mask = 0u;
    s_inited = 1u;
}

void control_manager_reset(void)
{
    CONTROL_MANAGER_ENTER_CRITICAL();
    control_reset_state_unlocked();
    CONTROL_MANAGER_EXIT_CRITICAL();
}

void control_manager_init(void)
{
    CONTROL_MANAGER_ENTER_CRITICAL();
    if (s_inited == 0u)
    {
        control_reset_state_unlocked();
    }
    CONTROL_MANAGER_EXIT_CRITICAL();
}

control_result_e control_manager_register(const control_controller_t *controller)
{
    control_result_e result = CONTROL_RESULT_OK;

    control_manager_init();

    if (controller == NULL ||
        controller->id == CONTROL_CONTROLLER_NONE ||
        control_domain_valid(controller->domain) == 0u)
    {
        return CONTROL_RESULT_BAD_ARGUMENT;
    }

    CONTROL_MANAGER_ENTER_CRITICAL();
    if (control_find(controller->id) != NULL)
    {
        result = CONTROL_RESULT_DUPLICATE;
    }
    else if (s_registry.count >= (uint8_t)CONTROL_MANAGER_MAX_CONTROLLERS)
    {
        result = CONTROL_RESULT_FULL;
    }
    else
    {
        s_registry.controller[s_registry.count] = *controller;
        s_registry.count++;
    }
    CONTROL_MANAGER_EXIT_CRITICAL();

    return result;
}

uint8_t control_manager_registered_count(void)
{
    uint8_t count;

    control_manager_init();

    CONTROL_MANAGER_ENTER_CRITICAL();
    count = s_registry.count;
    CONTROL_MANAGER_EXIT_CRITICAL();
    return count;
}

control_result_e control_manager_request_switch(uint16_t controller_id, control_transition_reason_e reason)
{
    const control_controller_t *controller;
    control_domain_state_t *domain_state;

    control_manager_init();
    controller = control_find(controller_id);
    if (controller == NULL)
    {
        return CONTROL_RESULT_NOT_FOUND;
    }
    if (control_domain_valid(controller->domain) == 0u)
    {
        return CONTROL_RESULT_BAD_ARGUMENT;
    }

    domain_state = &s_domain[controller->domain];
    CONTROL_MANAGER_ENTER_CRITICAL();
    domain_state->pending_id = controller_id;
    domain_state->pending_reason = reason;
    domain_state->pending_request = CONTROL_REQUEST_SWITCH;
    CONTROL_MANAGER_EXIT_CRITICAL();

    return CONTROL_RESULT_OK;
}

control_result_e control_manager_request_stop(control_domain_e domain, control_transition_reason_e reason)
{
    control_domain_state_t *domain_state;

    control_manager_init();
    if (control_domain_valid(domain) == 0u)
    {
        return CONTROL_RESULT_BAD_ARGUMENT;
    }

    domain_state = &s_domain[domain];
    CONTROL_MANAGER_ENTER_CRITICAL();
    domain_state->pending_id = CONTROL_CONTROLLER_NONE;
    domain_state->pending_reason = reason;
    domain_state->pending_request = CONTROL_REQUEST_STOP;
    CONTROL_MANAGER_EXIT_CRITICAL();

    return CONTROL_RESULT_OK;
}

void control_manager_request_stop_all(control_transition_reason_e reason)
{
    control_manager_init();
    for (uint8_t i = 0u; i < (uint8_t)CONTROL_DOMAIN__COUNT; i++)
    {
        (void)control_manager_request_stop((control_domain_e)i, reason);
    }
}

void control_manager_clear_pending(control_domain_e domain)
{
    if (control_domain_valid(domain) == 0u)
    {
        return;
    }

    control_manager_init();
    CONTROL_MANAGER_ENTER_CRITICAL();
    s_domain[domain].pending_request = CONTROL_REQUEST_NONE;
    s_domain[domain].pending_id = CONTROL_CONTROLLER_NONE;
    s_domain[domain].pending_reason = CONTROL_REASON_NONE;
    CONTROL_MANAGER_EXIT_CRITICAL();
}

control_result_e control_manager_update_domain(control_domain_e domain, control_context_t *context)
{
    control_domain_state_t *domain_state;
    const control_controller_t *active;
    control_context_t local_context;
    control_result_e result;

    control_manager_init();
    if (control_domain_valid(domain) == 0u)
    {
        return CONTROL_RESULT_BAD_ARGUMENT;
    }

    context = control_context_or_local(context, &local_context);
    domain_state = &s_domain[domain];

    result = control_apply_pending(domain, context);
    if (result != CONTROL_RESULT_OK && result != CONTROL_RESULT_NOT_ACTIVE)
    {
        return result;
    }

    active = domain_state->active;
    if (active == NULL)
    {
        return CONTROL_RESULT_NOT_ACTIVE;
    }

    result = control_call_callback(active->update, active, context, CONTROL_REASON_NONE);
    domain_state->update_count++;
    domain_state->last_result = result;
    if (result != CONTROL_RESULT_OK)
    {
        (void)control_stop_active(domain, CONTROL_REASON_FAULT, context);
        domain_state->state = CONTROL_STATE_FAULT;
        domain_state->last_result = result;
        return CONTROL_RESULT_CALLBACK_FAILED;
    }

    return CONTROL_RESULT_OK;
}

control_result_e control_manager_update_all(control_context_t *context)
{
    control_result_e first_error = CONTROL_RESULT_OK;

    for (uint8_t i = 0u; i < (uint8_t)CONTROL_DOMAIN__COUNT; i++)
    {
        control_result_e result = control_manager_update_domain((control_domain_e)i, context);
        if (first_error == CONTROL_RESULT_OK &&
            result != CONTROL_RESULT_OK &&
            result != CONTROL_RESULT_NOT_ACTIVE)
        {
            first_error = result;
        }
    }

    return first_error;
}

uint8_t control_manager_is_active(uint16_t controller_id)
{
    uint8_t active = 0u;

    control_manager_init();
    CONTROL_MANAGER_ENTER_CRITICAL();
    for (uint8_t i = 0u; i < (uint8_t)CONTROL_DOMAIN__COUNT; i++)
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

uint16_t control_manager_active_id(control_domain_e domain)
{
    uint16_t active_id = CONTROL_CONTROLLER_NONE;

    control_manager_init();
    if (control_domain_valid(domain) == 0u)
    {
        return CONTROL_CONTROLLER_NONE;
    }

    CONTROL_MANAGER_ENTER_CRITICAL();
    if (s_domain[domain].active != NULL)
    {
        active_id = s_domain[domain].active->id;
    }
    CONTROL_MANAGER_EXIT_CRITICAL();
    return active_id;
}

control_result_e control_manager_get_domain_status(control_domain_e domain, control_domain_status_t *out)
{
    const control_domain_state_t *domain_state;

    control_manager_init();
    if (control_domain_valid(domain) == 0u || out == NULL)
    {
        return CONTROL_RESULT_BAD_ARGUMENT;
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

    return CONTROL_RESULT_OK;
}

uint32_t control_manager_active_claim_mask(void)
{
    uint32_t claim_mask;

    control_manager_init();

    CONTROL_MANAGER_ENTER_CRITICAL();
    claim_mask = s_active_claim_mask;
    CONTROL_MANAGER_EXIT_CRITICAL();
    return claim_mask;
}
