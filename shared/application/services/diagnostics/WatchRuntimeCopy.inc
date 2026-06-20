/*
 * SPDX-FileCopyrightText: 2026 ?? <2811158416@qq.com>
 * SPDX-License-Identifier: Apache-2.0
 *
 * Private implementation chunk included by Watch.c.
 */

#if WATCH_ENABLE_RUNTIME_COPY
static void WatchRuntimeAddEntry(const char *name,
                                    RuntimeInstanceKind kind,
                                    RuntimeInstanceState state,
                                    uint16_t source_id,
                                    uint16_t source_index,
                                    uint16_t parent_index)
{
    const uint8_t index = g_watch.runtime.entry_visible_count;

    if (g_watch.runtime.entry_count < 0xFFu)
    {
        g_watch.runtime.entry_count++;
    }

    if (index >= (uint8_t)(sizeof(g_watch.runtime.entry) / sizeof(g_watch.runtime.entry[0])))
    {
        return;
    }

    g_watch.runtime.entry[index] = RuntimeInstanceRefMake(name,
                                                            kind,
                                                            state,
                                                            source_id,
                                                            source_index,
                                                            parent_index);
    g_watch.runtime.entry_visible_count++;
}

static const WatchTaskModuleCreateSlot *WatchTaskModuleCreateFind(uint8_t module)
{
    for (uint8_t i = 0u; i < (uint8_t)(sizeof(s_task_module_create) / sizeof(s_task_module_create[0])); i++)
    {
        if (s_task_module_create[i].used != 0u && s_task_module_create[i].module == module)
        {
            return &s_task_module_create[i];
        }
    }

    return NULL;
}

static RuntimeInstanceState WatchRuntimeDeviceState(const RobotConfigDevice *device)
{
    if (device == NULL)
    {
        return RUNTIME_INSTANCE_STATE_UNKNOWN;
    }

    switch ((RobotConfigDeviceKind)device->kind)
    {
    case ROBOT_CONFIG_DEVICE_KIND_MOTOR:
    {
        const MotorInst *inst = MotorInstFindByMotor((MotorId)device->source_id);
        return (inst != NULL && MotorInstEnabled(inst) != 0u) ?
                   RUNTIME_INSTANCE_STATE_ENABLED :
                   RUNTIME_INSTANCE_STATE_DISABLED;
    }
    default:
        return RUNTIME_INSTANCE_STATE_ENABLED;
    }
}
#endif

static void WatchDiagPushStage(WatchBootStage stage)
{
    if (stage == WATCH_BOOT_STAGE_NONE)
    {
        g_watch.fault.boot_stage = stage;
        return;
    }

    if (g_watch.fault.boot_stage == stage)
    {
        return;
    }

    for (uint32_t i = (uint32_t)(sizeof(g_watch.fault.boot_trace) / sizeof(g_watch.fault.boot_trace[0])) - 1u; i > 0u; i--)
    {
        g_watch.fault.boot_trace[i] = g_watch.fault.boot_trace[i - 1u];
    }
    g_watch.fault.boot_trace[0] = stage;
    g_watch.fault.boot_stage = stage;
}

static void WatchCopyRc(void)
{
    if (ManualInputGetCurrentCopy(&rc_snapshot) != 0u)
    {
        rc_src = &rc_snapshot;
    }
    else
    {
        rc_src = NULL;
    }

    if (rc_src == NULL)
    {
        memset(&g_watch.rc, 0, sizeof(g_watch.rc));
        return;
    }

    for (uint8_t i = 0; i < 5; i++)
    {
        g_watch.rc.ch[i] = rc_src->rc.ch[i];
    }
    g_watch.rc.s[0] = rc_src->rc.s[0];
    g_watch.rc.s[1] = rc_src->rc.s[1];

    g_watch.rc.mouse_x = rc_src->mouse.x;
    g_watch.rc.mouse_y = rc_src->mouse.y;
    g_watch.rc.mouse_z = rc_src->mouse.z;
    g_watch.rc.mouse_l = rc_src->mouse.press_l;
    g_watch.rc.mouse_r = rc_src->mouse.press_r;
    g_watch.rc.key = rc_src->key.v;
}

static void WatchCopyNewrc(void)
{
    ImageRemoteState state = {0};

    if (!ImageRemoteGetState(&state))
    {
        memset(&g_watch.newrc, 0, sizeof(g_watch.newrc));
        return;
    }

    g_watch.newrc.valid = state.valid;
    g_watch.newrc.proto = state.proto;
    g_watch.newrc.range_mode = state.range_mode;
    for (uint8_t i = 0; i < 5; i++)
    {
        g_watch.newrc.raw_ch[i] = state.raw_ch[i];
    }
    for (uint8_t i = 0; i < 5; i++)
    {
        g_watch.newrc.ch[i] = state.ch[i];
    }
    g_watch.newrc.s[0] = state.s[0];
    g_watch.newrc.s[1] = state.s[1];
    g_watch.newrc.mouse_x = state.mouse_x;
    g_watch.newrc.mouse_y = state.mouse_y;
    g_watch.newrc.mouse_z = state.mouse_z;
    g_watch.newrc.mouse_l = state.mouse_l;
    g_watch.newrc.mouse_r = state.mouse_r;
    g_watch.newrc.mouse_mid = state.mouse_mid;
    g_watch.newrc.pause = state.pause;
    g_watch.newrc.btn_l = state.btn_l;
    g_watch.newrc.btn_r = state.btn_r;
    g_watch.newrc.trigger = state.trigger;
    g_watch.newrc.dial = state.dial;
    g_watch.newrc.key_value = state.key_value;
    g_watch.newrc.key_w = state.key_w;
    g_watch.newrc.key_s = state.key_s;
    g_watch.newrc.key_a = state.key_a;
    g_watch.newrc.key_d = state.key_d;
    g_watch.newrc.key_shift = state.key_shift;
    g_watch.newrc.key_ctrl = state.key_ctrl;
    g_watch.newrc.key_q = state.key_q;
    g_watch.newrc.key_e = state.key_e;
    g_watch.newrc.key_r = state.key_r;
    g_watch.newrc.key_f = state.key_f;
    g_watch.newrc.key_g = state.key_g;
    g_watch.newrc.key_z = state.key_z;
    g_watch.newrc.key_x = state.key_x;
    g_watch.newrc.key_c = state.key_c;
    g_watch.newrc.key_v = state.key_v;
    g_watch.newrc.key_b = state.key_b;
    g_watch.newrc.last_rx_tick_ms = state.last_rx_tick_ms;
}

#if WATCH_ENABLE_COMM_COPY
static void WatchCopyComm(void)
{
    BspRcDiag rc_diag;

    memset(&rc_diag, 0, sizeof(rc_diag));
    BspRcGetDiag(&rc_diag);

    g_watch.comm.rc_uart_rx_event_count = rc_diag.rx_event_cnt;
    g_watch.comm.rc_uart_bad_size_count = rc_diag.rx_bad_size_cnt;
    g_watch.comm.rc_uart_error_count = rc_diag.uart_error_cnt;
    g_watch.comm.rc_uart_last_error = rc_diag.uart_last_error;
    g_watch.comm.rc_uart_restart_count = rc_diag.restart_cnt;
    g_watch.comm.rc_uart_drop_count = rc_diag.drop_cnt;
    g_watch.comm.RcSbusFrameCount = ManualInputGetSbusFrameCount();
    g_watch.comm.rc_set_source_count = ManualInputGetSetSourceCount();
    g_watch.comm.rc_uart_last_size = rc_diag.rx_last_size;
    g_watch.comm.rc_uart_last_event = rc_diag.rx_last_event;

    g_watch.comm.CanRxCount[0] = CAN_get_can1_rx_count();
    g_watch.comm.CanRxCount[1] = CAN_get_can2_rx_count();
    g_watch.comm.CanRxCount[2] = CAN_get_can3_rx_count();
    g_watch.comm.CanRxDropCount[0] = CAN_get_can1_rx_drop_count();
    g_watch.comm.CanRxDropCount[1] = CAN_get_can2_rx_drop_count();
    g_watch.comm.CanRxDropCount[2] = CAN_get_can3_rx_drop_count();
    g_watch.comm.CanTxCount[0] = CAN_get_can1_tx_count();
    g_watch.comm.CanTxCount[1] = CAN_get_can2_tx_count();
    g_watch.comm.CanTxCount[2] = CAN_get_can3_tx_count();
    g_watch.comm.CanTxFailCount[0] = CAN_get_can1_tx_fail_count();
    g_watch.comm.CanTxFailCount[1] = CAN_get_can2_tx_fail_count();
    g_watch.comm.CanTxFailCount[2] = CAN_get_can3_tx_fail_count();

    g_watch.comm.can_last_rx_id[0] = CAN_get_can1_last_rx_id();
    g_watch.comm.can_last_rx_id[1] = CAN_get_can2_last_rx_id();
    g_watch.comm.can_last_rx_id[2] = CAN_get_can3_last_rx_id();
    g_watch.comm.can_last_tx_id[0] = CAN_get_can1_last_tx_id();
    g_watch.comm.can_last_tx_id[1] = CAN_get_can2_last_tx_id();
    g_watch.comm.can_last_tx_id[2] = CAN_get_can3_last_tx_id();
    g_watch.comm.can_last_rx_dlc[0] = CAN_get_can1_last_rx_dlc();
    g_watch.comm.can_last_rx_dlc[1] = CAN_get_can2_last_rx_dlc();
    g_watch.comm.can_last_rx_dlc[2] = CAN_get_can3_last_rx_dlc();
    g_watch.comm.can_last_tx_dlc[0] = CAN_get_can1_last_tx_dlc();
    g_watch.comm.can_last_tx_dlc[1] = CAN_get_can2_last_tx_dlc();
    g_watch.comm.can_last_tx_dlc[2] = CAN_get_can3_last_tx_dlc();

    g_watch.comm.can_protocol_lec[0] = CAN_get_can1_protocol_lec();
    g_watch.comm.can_protocol_lec[1] = CAN_get_can2_protocol_lec();
    g_watch.comm.can_protocol_lec[2] = CAN_get_can3_protocol_lec();
    g_watch.comm.can_protocol_dlec[0] = CAN_get_can1_protocol_dlec();
    g_watch.comm.can_protocol_dlec[1] = CAN_get_can2_protocol_dlec();
    g_watch.comm.can_protocol_dlec[2] = CAN_get_can3_protocol_dlec();
    g_watch.comm.can_bus_off[0] = CAN_get_can1_bus_off();
    g_watch.comm.can_bus_off[1] = CAN_get_can2_bus_off();
    g_watch.comm.can_bus_off[2] = CAN_get_can3_bus_off();
    g_watch.comm.CanTxErrorCount[0] = CAN_get_can1_tx_error_count();
    g_watch.comm.CanTxErrorCount[1] = CAN_get_can2_tx_error_count();
    g_watch.comm.CanTxErrorCount[2] = CAN_get_can3_tx_error_count();
    g_watch.comm.CanRxErrorCount[0] = CAN_get_can1_rx_error_count();
    g_watch.comm.CanRxErrorCount[1] = CAN_get_can2_rx_error_count();
    g_watch.comm.CanRxErrorCount[2] = CAN_get_can3_rx_error_count();
}
#endif

#if WATCH_ENABLE_RUNTIME_COPY
static void WatchCopyRuntime(void)
{
    uint8_t motor_count;
    uint8_t motor_visible_count;
    uint8_t controller_count;
    uint8_t controller_visible_count;
    uint8_t task_module_count;
    uint8_t task_module_visible_count;
    uint8_t device_count;
    uint8_t module_desc_count;
    RtProfSummary profiler_summary;
    LowCmdDiag lowcmd_diag = {0};
    MotorInstDiag motor_inst_diag = {0};

    memset(&g_watch.runtime, 0, sizeof(g_watch.runtime));
    RtProfGetSummary(&profiler_summary);
    (void)LowCmdGetDiag(&lowcmd_diag);
    (void)MotorInstGetDiag(&motor_inst_diag);
    g_watch.runtime.profile_kind = (uint8_t)RobotProfileGetKind();
    g_watch.runtime.BoardKind = (uint8_t)RobotBoardGetKind();
    g_watch.runtime.BoardCanBusCount = RobotBoardCanBusCount();
    g_watch.runtime.BoardHasFpu = RobotBoardHasFpu();
    g_watch.runtime.BoardCpuHz = RobotBoardCpuHz();
    g_watch.runtime.rtProfCount = profiler_summary.total_count;
    g_watch.runtime.RtProfActive_count = profiler_summary.active_count;
    g_watch.runtime.RtProfOverBudget_count = profiler_summary.over_budget_count;
    g_watch.runtime.rtProfTotalOverrun = profiler_summary.total_overrun_count;
    g_watch.runtime.rtProfMaxLastUs = profiler_summary.max_last_us;
    g_watch.runtime.rtProfMaxBudgetUs = profiler_summary.max_budget_us;
    g_watch.runtime.rtProfMaxOverBudgetUs = profiler_summary.max_over_budget_us;
    g_watch.runtime.lowcmd_seq = lowcmd_diag.seq;
    g_watch.runtime.lowcmd_rejected_count = lowcmd_diag.rejected_count;
    g_watch.runtime.lowcmd_emergency_stop_count = lowcmd_diag.emergency_stop_count;
    g_watch.runtime.lowcmd_last_reject_tick_ms = lowcmd_diag.last_reject_tick;
    g_watch.runtime.lowcmd_last_reject_writer = lowcmd_diag.last_reject_writer;
    g_watch.runtime.lowcmd_last_reject_owner = lowcmd_diag.last_reject_owner;
    g_watch.runtime.lowcmd_emergency_writer = lowcmd_diag.emergency_writer;
    g_watch.runtime.lowcmd_emergency_active = lowcmd_diag.emergency_active;
    g_watch.runtime.motor_feedback_conflict_count = motor_inst_diag.feedback_conflict_count;
    g_watch.runtime.motor_feedback_table_full_count = motor_inst_diag.feedback_table_full_count;
    g_watch.runtime.motor_feedback_conflict_id = motor_inst_diag.last_feedback_conflict_id;
    g_watch.runtime.motor_feedback_conflict_bus = motor_inst_diag.last_feedback_conflict_bus;
    g_watch.runtime.motor_feedback_conflict_kept = motor_inst_diag.last_feedback_conflict_kept;
    g_watch.runtime.motor_feedback_conflict_dropped = motor_inst_diag.last_feedback_conflict_dropped;

    (void)RobotModuleKnownModules(&module_desc_count);
    g_watch.runtime.module_desc_count = module_desc_count;
    task_module_count = RobotProfileModuleCount();
    task_module_visible_count = task_module_count;
    if (task_module_visible_count > (uint8_t)(sizeof(g_watch.runtime.task_module) / sizeof(g_watch.runtime.task_module[0])))
    {
        task_module_visible_count = (uint8_t)(sizeof(g_watch.runtime.task_module) / sizeof(g_watch.runtime.task_module[0]));
    }

    g_watch.runtime.task_module_count = task_module_count;
    g_watch.runtime.task_module_visible_count = task_module_visible_count;
    for (uint8_t i = 0u; i < task_module_visible_count; i++)
    {
        const RobotTaskModuleId module = RobotProfileModuleIdAt(i);
        WatchRuntimeTaskModule *dst = &g_watch.runtime.task_module[i];
        const WatchTaskModuleCreateSlot *create = WatchTaskModuleCreateFind((uint8_t)module);
        const RobotModuleDesc *module_desc = RobotModuleFindByTaskModule(module);

        dst->module = (uint8_t)module;
        dst->name = RobotProfileModuleName(module);
        dst->module_name = dst->name;
        if (module_desc != NULL)
        {
            dst->module_name = module_desc->name;
            dst->kind = (uint8_t)module_desc->kind;
            dst->priority = module_desc->defaultPriority;
            dst->period_ms = RobotModulePeriodMs(module_desc);
            dst->budget_us = RobotModuleBudgetUs(module_desc);
            dst->stack_words = module_desc->defaultStackWords;
            dst->flags = module_desc->flags;
            dst->require_count = module_desc->requireCount;
            dst->provide_count = module_desc->provideCount;
            if ((module_desc->flags & ROBOT_MODULE_FLAG_REQUIRED_BY_PROFILE) != 0u)
            {
                g_watch.runtime.module_required_count++;
            }
            if ((module_desc->flags & ROBOT_MODULE_FLAG_FAST_PATH) != 0u)
            {
                g_watch.runtime.module_fast_count++;
            }
        }
        else
        {
            g_watch.runtime.module_missing_desc_count++;
        }
        if (create != NULL)
        {
            if (create->name != NULL)
            {
                dst->name = create->name;
            }
            dst->create_state = create->create_state;
            dst->create_fail_count = create->create_fail_count;
            dst->thread_handle = create->thread_handle;
            dst->create_attempt_count = create->create_attempt_count;
        }
        WatchRuntimeAddEntry(dst->name,
                                RUNTIME_INSTANCE_KIND_TASK,
                                (dst->create_state == 2u) ?
                                    RUNTIME_INSTANCE_STATE_ACTIVE :
                                    ((dst->create_state >= 3u) ?
                                         RUNTIME_INSTANCE_STATE_FAULT :
                                         RUNTIME_INSTANCE_STATE_ENABLED),
                                (uint16_t)module,
                                i,
                                RUNTIME_INSTANCE_INDEX_NONE);
    }

    device_count = RobotConfigDeviceCount();
    g_watch.runtime.device_count = device_count;
    for (uint8_t i = 0u; i < device_count; i++)
    {
        RobotConfigDevice device;

        if (RobotConfigDeviceGet(i, &device) == 0u)
        {
            continue;
        }

        WatchRuntimeAddEntry(device.name,
                                RUNTIME_INSTANCE_KIND_DEVICE,
                                WatchRuntimeDeviceState(&device),
                                device.source_id,
                                i,
                                RUNTIME_INSTANCE_INDEX_NONE);
    }

    motor_count = MotorRouteCount();
    motor_visible_count = motor_count;
    if (motor_visible_count > (uint8_t)(sizeof(g_watch.runtime.motor) / sizeof(g_watch.runtime.motor[0])))
    {
        motor_visible_count = (uint8_t)(sizeof(g_watch.runtime.motor) / sizeof(g_watch.runtime.motor[0]));
    }

    g_watch.runtime.motor_count = motor_count;
    g_watch.runtime.motor_visible_count = motor_visible_count;
    for (uint8_t i = 0u; i < motor_visible_count; i++)
    {
        const MotorRoute *route = MotorRouteGet(i);
        WatchRuntimeMotor *dst = &g_watch.runtime.motor[i];
        MotorState fb;
        MotorApplied applied;
        MotorId actuator_id;

        if (route == NULL)
        {
            continue;
        }

        actuator_id = route->motorId;
        dst->name = route->name;
        dst->actuator_id = (uint16_t)actuator_id;
        dst->role = (uint8_t)route->role;
        dst->role_index = route->roleIndex;
        dst->enabled = route->enabled;
        dst->bus = route->bus;
        dst->transport = route->transport;
        dst->protocol = route->protocol;
        dst->control_mode = route->controlMode;
        dst->cmd_caps = route->cmdCaps;
        dst->model = route->model;
        if (LowStateGetMotor(actuator_id, &fb) != 0u)
        {
            dst->drive_state = fb.driveState;
        }
        if (LowStateGetApplied(actuator_id, &applied) != 0u)
        {
            dst->applied_mode = applied.mode;
            dst->applied_flags = applied.flags;
            dst->applied_current = applied.current;
            dst->applied_tick_ms = applied.tick;
            dst->applied_torque_nm = applied.tau;
        }
        if (dst->enabled != 0u)
        {
            g_watch.runtime.motor_enabled_count++;
        }
    }

    controller_count = ControlMgrCount();
    controller_visible_count = controller_count;
    if (controller_visible_count > (uint8_t)(sizeof(g_watch.runtime.controller) / sizeof(g_watch.runtime.controller[0])))
    {
        controller_visible_count = (uint8_t)(sizeof(g_watch.runtime.controller) / sizeof(g_watch.runtime.controller[0]));
    }

    g_watch.runtime.controller_count = controller_count;
    g_watch.runtime.controller_visible_count = controller_visible_count;
    g_watch.runtime.active_claim_mask = ControlMgrActiveClaimMask();
    for (uint8_t i = 0u; i < controller_visible_count; i++)
    {
        const ControlController *controller = ControlMgrGet(i);
        WatchRuntimeController *dst = &g_watch.runtime.controller[i];

        if (controller == NULL)
        {
            continue;
        }

        dst->name = controller->name;
        dst->id = controller->id;
        dst->period_ms = controller->meta.period_ms;
        dst->phase_ms = controller->meta.phase_ms;
        dst->domain = (uint8_t)controller->domain;
        dst->active = ControlMgrIsActive(controller->id);
        dst->priority = controller->meta.priority;
        dst->input_count = controller->meta.input_count;
        dst->output_count = controller->meta.output_count;
        WatchRuntimeAddEntry(dst->name,
                                RUNTIME_INSTANCE_KIND_CONTROLLER,
                                (dst->active != 0u) ?
                                    RUNTIME_INSTANCE_STATE_ACTIVE :
                                    RUNTIME_INSTANCE_STATE_ENABLED,
                                dst->id,
                                i,
                                (uint16_t)controller->domain);
    }

    g_watch.runtime.domain_count = (uint8_t)ControlDomainCount;
    for (uint8_t i = 0u; i < (uint8_t)ControlDomainCount; i++)
    {
        ControlStatus status = {0};
        WatchRuntimeDomain *dst = &g_watch.runtime.domain[i];

        if (ControlMgrGetStatus((ControlDomain)i, &status) != ControlResultOk)
        {
            continue;
        }

        dst->active_name = status.active_name;
        dst->active_id = status.active_id;
        dst->pending_id = status.pending_id;
        dst->domain = (uint8_t)status.domain;
        dst->active = status.active;
        dst->state = (uint8_t)status.state;
        dst->pending_request = (uint8_t)status.pending_request;
        dst->last_reason = (uint8_t)status.last_reason;
        dst->last_result = (uint8_t)status.last_result;
        dst->update_count = status.update_count;
        dst->transition_count = status.transition_count;
        dst->reject_count = status.reject_count;
        if (dst->active != 0u)
        {
            g_watch.runtime.active_controller_count++;
        }
        WatchRuntimeAddEntry(ControlDomainName(status.domain),
                                RUNTIME_INSTANCE_KIND_GROUP,
                                (status.state == ControlStateFault) ?
                                    RUNTIME_INSTANCE_STATE_FAULT :
                                    ((dst->active != 0u) ?
                                         RUNTIME_INSTANCE_STATE_ACTIVE :
                                         RUNTIME_INSTANCE_STATE_DISABLED),
                                (uint16_t)status.domain,
                                i,
                                RUNTIME_INSTANCE_INDEX_NONE);
    }
}
#endif
