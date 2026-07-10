/*
 * SPDX-FileCopyrightText: 2026 ?? <2811158416@qq.com>
 * SPDX-License-Identifier: Apache-2.0
 *
 * Private implementation chunk included by Watch.c.
 */

static uint8_t WatchBlockActiveAlways(void)
{
    return 1u;
}

static void WatchUpdateSetStage(WatchUpdateStage stage)
{
    g_watch.rtos.WatchUpdateStage = (uint8_t)stage;
    g_watch.rtos.WatchUpdateStageTickMs = HAL_GetTick();
}

#if WATCH_ENABLE_ARM_J0_UNITREE || WATCH_ENABLE_LOCOMOTION_WHEELLEG_MIT || WATCH_ENABLE_GIMBAL_SINGLE || WATCH_ENABLE_GIMBAL_DUAL
static fp32 WatchRadToDeg(fp32 rad)
{
    return rad * 57.29577951308232f;
}
#endif

#if WATCH_ENABLE_GIMBAL_SINGLE
static fp32 WatchEcdToDeg(MotorModel model, uint16_t ecd)
{
    const fp32 full_range = (fp32)MotorCfgEncoderRange(model);

    return (fp32)ecd * 360.0f / full_range;
}
#endif

#if WATCH_ENABLE_LOCOMOTION_CLASSIC
static uint8_t WatchBlockActiveLocomotionClassic(void)
{
    return RobotProfileModuleEnabled(ROBOT_TASK_MODULE_CLASSIC_CHASSIS);
}
#endif

#if WATCH_ENABLE_LOCOMOTION_WHEELLEG_SERVO
static uint8_t WatchBlockActiveWheelLegServo(void)
{
    return RobotProfileModuleEnabled(ROBOT_TASK_MODULE_WHEELLEG_SERVO);
}
#endif

#if WATCH_ENABLE_LOCOMOTION_WHEELLEG_MIT
static uint8_t WatchBlockActiveWheelLegMit(void)
{
    return RobotProfileModuleEnabled(ROBOT_TASK_MODULE_WHEELLEG_MIT);
}
#endif

#if WATCH_ENABLE_GIMBAL_SINGLE
static uint8_t WatchBlockActiveGimbalSingle(void)
{
    uint8_t active = RobotProfileModuleEnabled(ROBOT_TASK_MODULE_SINGLE_GIMBAL);
#if WATCH_ENABLE_GIMBAL_DUAL
    if (active == 0u)
    {
        active = RobotProfileModuleEnabled(ROBOT_TASK_MODULE_DUAL_YAW_GIMBAL);
    }
#endif
    return active;
}
#endif

#if WATCH_ENABLE_GIMBAL_DUAL
static uint8_t WatchBlockActiveGimbalDual(void)
{
    return RobotProfileModuleEnabled(ROBOT_TASK_MODULE_DUAL_YAW_GIMBAL);
}
#endif

#if WATCH_ENABLE_SHOOT_RM
static uint8_t WatchBlockActiveShootRm(void)
{
    if (g_config.motor.trigger.can_id != 0u)
    {
        return 1u;
    }

    for (uint8_t i = 0u; i < 4u; i++)
    {
        if (g_config.motor.friction[i].can_id != 0u)
        {
            return 1u;
        }
    }

    return 0u;
}
#endif

#if WATCH_ENABLE_ARM_J0_UNITREE
static uint8_t WatchBlockActiveArm(void)
{
    return RobotProfileModuleEnabled(ROBOT_TASK_MODULE_ARM);
}
#endif

static WatchTaskDiagEntry *WatchTaskDiagGet(WatchTaskId task_id)
{
    switch (task_id)
    {
    case WATCH_TASK_DEFAULT:
        return &g_watch.rtos.task.default_task;
    case WATCH_TASK_DETECT:
        return &g_watch.rtos.task.DetectTask;
    case WATCH_TASK_IMU:
        return &g_watch.rtos.task.imu_task;
    case WATCH_TASK_GIMBAL_CONTROL:
#if WATCH_ENABLE_GIMBAL_SINGLE || WATCH_ENABLE_GIMBAL_DUAL
        return &g_watch.rtos.task.GimbalControlTask;
#else
        return NULL;
#endif
    case WATCH_TASK_CHASSIS_CONTROL:
#if WATCH_ENABLE_LOCOMOTION_CLASSIC
        return &g_watch.rtos.task.ChassisControlTask;
#else
        return NULL;
#endif
    case WATCH_TASK_CAN_FEEDBACK_RX:
        return &g_watch.rtos.task.CanRxTask;
    case WATCH_TASK_CAN_COMMAND_TX:
        return &g_watch.rtos.task.CanTxTask;
    case WATCH_TASK_RC_SBUS:
        return &g_watch.rtos.task.RcSbusTask;
    case WATCH_TASK_HOST_LINK:
        return &g_watch.rtos.task.HostLinkTask;
    case WATCH_TASK_ELRS:
        return &g_watch.rtos.task.ElrsTask;
    case WATCH_TASK_ARM:
#if WATCH_ENABLE_ARM_J0_UNITREE
        return &g_watch.rtos.task.ArmTask;
#else
        return NULL;
#endif
    case WATCH_TASK_WHEELLEG_MIT:
#if WATCH_ENABLE_LOCOMOTION_WHEELLEG_MIT
        return &g_watch.rtos.task.WheelLegMitTask;
#else
        return NULL;
#endif
    default:
        return NULL;
    }
}

static WatchIrqDiagEntry *WatchIrqDiagGet(WatchIrqId irq_id)
{
    switch (irq_id)
    {
    case WATCH_IRQ_IST8310_EXTI:
        return &g_watch.rtos.irq.ist8310_exti;
    case WATCH_IRQ_IMU_EXTI:
        return &g_watch.rtos.irq.imu_exti;
    case WATCH_IRQ_SD_EXTI:
        return &g_watch.rtos.irq.sd_exti;
    case WATCH_IRQ_CAN1_RX0:
        return &g_watch.rtos.irq.can1_rx0;
    case WATCH_IRQ_CAN2_RX0:
        return &g_watch.rtos.irq.can2_rx0;
    case WATCH_IRQ_USART1:
        return &g_watch.rtos.irq.usart1;
    case WATCH_IRQ_UART7:
        return &g_watch.rtos.irq.uart7;
    case WATCH_IRQ_UART8:
        return &g_watch.rtos.irq.uart8;
    case WATCH_IRQ_OTG_FS:
        return &g_watch.rtos.irq.otg_fs;
    case WATCH_IRQ_TIM6_DAC:
        return &g_watch.rtos.irq.tim6_dac;
    case WATCH_IRQ_DMA_USART1_RX:
        return &g_watch.rtos.irq.dma_usart1_rx;
    case WATCH_IRQ_DMA_SPI5_TX:
        return &g_watch.rtos.irq.dma_spi5_tx;
    case WATCH_IRQ_DMA_SPI5_RX:
        return &g_watch.rtos.irq.dma_spi5_rx;
    case WATCH_IRQ_DMA_SDIO_TX:
        return &g_watch.rtos.irq.dma_sdio_tx;
    default:
        return NULL;
    }
}

void WatchDiagSetBootStage(WatchBootStage stage)
{
    WatchDiagPushStage(stage);
}

void WatchDiagMarkErrorHandler(uint32_t tick_ms, uint32_t ipsr)
{
    g_watch.fault.error_handler_count++;
    g_watch.fault.error_stage = g_watch.fault.boot_stage;
    g_watch.fault.error_tick_ms = tick_ms;
    g_watch.fault.error_ipsr = ipsr;
}

void WatchDiagSetErrorArgs(uint32_t arg0, uint32_t arg1)
{
    g_watch.fault.error_arg0 = arg0;
    g_watch.fault.error_arg1 = arg1;
}

void WatchDiagMarkFatal(uint32_t reason, uint32_t task_handle, const char *task_name)
{
    g_watch.rtos.fatal_reason = reason;
    g_watch.rtos.fatal_task_handle = task_handle;
    g_watch.rtos.current_task_handle = task_handle;
    memset(g_watch.rtos.fatal_task_name, 0, sizeof(g_watch.rtos.fatal_task_name));
    memset(g_watch.rtos.current_task_name, 0, sizeof(g_watch.rtos.current_task_name));
    if (task_name != NULL)
    {
        (void)strncpy(g_watch.rtos.fatal_task_name, task_name, sizeof(g_watch.rtos.fatal_task_name) - 1u);
        (void)strncpy(g_watch.rtos.current_task_name, task_name, sizeof(g_watch.rtos.current_task_name) - 1u);
    }
}

void WatchTaskModuleCreateReset(void)
{
    memset(s_task_module_create, 0, sizeof(s_task_module_create));
}

void WatchTaskModuleCreateResult(uint8_t module, const char *name, uint32_t thread_handle, uint8_t state)
{
    WatchTaskModuleCreateSlot *slot = NULL;

    for (uint8_t i = 0u; i < (uint8_t)(sizeof(s_task_module_create) / sizeof(s_task_module_create[0])); i++)
    {
        if (s_task_module_create[i].used != 0u && s_task_module_create[i].module == module)
        {
            slot = &s_task_module_create[i];
            break;
        }
        if (slot == NULL && s_task_module_create[i].used == 0u)
        {
            slot = &s_task_module_create[i];
        }
    }

    if (slot == NULL)
    {
        return;
    }

    slot->used = 1u;
    slot->module = module;
    slot->name = name;
    slot->thread_handle = thread_handle;
    slot->create_state = state;
    slot->create_attempt_count++;
    if (state >= 3u && slot->create_fail_count < 0xFFFFu)
    {
        slot->create_fail_count++;
    }
}

void WatchTaskBeat(WatchTaskId task_id)
{
    const uint32_t now_ms = HAL_GetTick();
    WatchTaskDiagEntry *entry = WatchTaskDiagGet(task_id);
    if (entry == NULL)
    {
        return;
    }

    if (entry->last_tick_ms != 0u &&
        (uint32_t)(now_ms - entry->last_tick_ms) < (uint32_t)ROBOT_PROFILE_WATCH_TASK_BEAT_MIN_PERIOD_MS)
    {
        return;
    }

    const uint64_t beat_start_us = RtProfBegin();
    taskENTER_CRITICAL();
    if (entry->last_tick_ms != 0u)
    {
        const uint32_t gap_ms = now_ms - entry->last_tick_ms;
        if (gap_ms > entry->max_gap_ms)
        {
            entry->max_gap_ms = gap_ms;
        }
    }
    entry->beat_count++;
    entry->last_tick_ms = now_ms;
    taskEXIT_CRITICAL();
    RtProfEnd(RtProfWatchBeat, beat_start_us);
}

void WatchTaskWait(WatchTaskId task_id)
{
    WatchTaskDiagEntry *entry = WatchTaskDiagGet(task_id);
    if (entry == NULL)
    {
        return;
    }

    taskENTER_CRITICAL();
    entry->wait_count++;
    taskEXIT_CRITICAL();
}

void WatchTaskTimeout(WatchTaskId task_id)
{
    WatchTaskDiagEntry *entry = WatchTaskDiagGet(task_id);
    if (entry == NULL)
    {
        return;
    }

    taskENTER_CRITICAL();
    entry->timeout_count++;
    taskEXIT_CRITICAL();
}

void WatchTaskError(WatchTaskId task_id)
{
    WatchTaskDiagEntry *entry = WatchTaskDiagGet(task_id);
    if (entry == NULL)
    {
        return;
    }

    taskENTER_CRITICAL();
    entry->error_count++;
    taskEXIT_CRITICAL();
}

void WatchImuSetStage(WatchImuStage stage)
{
    taskENTER_CRITICAL();
    g_watch.rtos.imu_task_stage = (uint8_t)stage;
    g_watch.rtos.imu_task_stage_tick_ms = HAL_GetTick();
    if (stage == WATCH_IMU_STAGE_ENTER)
    {
        g_watch.rtos.imu_task_enter_count++;
    }
    taskEXIT_CRITICAL();
}

void WatchIrqHit(WatchIrqId irq_id)
{
    const uint32_t now_ms = HAL_GetTick();
    WatchIrqDiagEntry *entry = WatchIrqDiagGet(irq_id);
    if (entry == NULL)
    {
        return;
    }

    {
        UBaseType_t saved = taskENTER_CRITICAL_FROM_ISR();
        entry->hit_count++;
        entry->last_tick_ms = now_ms;
        taskEXIT_CRITICAL_FROM_ISR(saved);
    }
}

void WatchInit(void)
{
    memset(&g_watch, 0, sizeof(g_watch));
#if WATCH_ENABLE_SHOOT_RM
    WatchPrepareMotorIds();
#endif

    memset(&s_watch_manual_input, 0, sizeof(s_watch_manual_input));
    s_watch_manual_input_valid = 0u;
    ins_quat_src = get_INS_quat_point();
    ins_angle_src = get_INS_angle_point();
    ins_gyro_src = get_gyro_data_point();
    ins_accel_src = get_accel_data_point();

    WatchUpdate();
}

void WatchUpdate(void)
{
    g_watch.rtos.WatchUpdateEnterCount++;
    s_watch_manual_input_valid = ManualInputSnapshotRead(&s_watch_manual_input);
    WatchUpdateSetStage(WATCH_UPDATE_STAGE_RC);
    WatchCopyRc();
    WatchUpdateSetStage(WATCH_UPDATE_STAGE_NEWRC);
    WatchCopyNewrc();
#if WATCH_ENABLE_COMM_COPY
    WatchUpdateSetStage(WATCH_UPDATE_STAGE_COMM);
    WatchCopyComm();
#endif
#if WATCH_ENABLE_RUNTIME_COPY
    WatchUpdateSetStage(WATCH_UPDATE_STAGE_RUNTIME);
    WatchCopyRuntime();
#endif
    WatchUpdateSetStage(WATCH_UPDATE_STAGE_IMU);
    WatchCopyImu();
#if WATCH_ENABLE_LOCOMOTION_CLASSIC
    WatchUpdateSetStage(WATCH_UPDATE_STAGE_CHASSIS);
    WatchCopyChassis();
#endif
#if WATCH_ENABLE_GIMBAL_SINGLE
    WatchUpdateSetStage(WATCH_UPDATE_STAGE_GIMBAL);
    WatchCopyGimbal();
#endif
#if WATCH_ENABLE_GIMBAL_DUAL
    WatchUpdateSetStage(WATCH_UPDATE_STAGE_GIMBAL);
    WatchCopyDualGimbal();
#endif
#if WATCH_ENABLE_SHOOT_RM
    WatchUpdateSetStage(WATCH_UPDATE_STAGE_SHOOT);
    WatchCopyShoot();
#endif
#if WATCH_ENABLE_ARM_J0_UNITREE
    WatchUpdateSetStage(WATCH_UPDATE_STAGE_ARM);
    WatchCopyArmJ0Unitree();
#endif
#if WATCH_ENABLE_LOCOMOTION_WHEELLEG_MIT
    WatchUpdateSetStage(WATCH_UPDATE_STAGE_WHEELLEG);
    WatchCopyWheelLegMit();
#endif
    WatchUpdateSetStage(WATCH_UPDATE_STAGE_DIAG);
    WatchCopyDiag();
    WatchUpdateSetStage(WATCH_UPDATE_STAGE_RTOS);
    WatchCopyRtos();
    WatchUpdateSetStage(WATCH_UPDATE_STAGE_DONE);
}

const WatchBlockDesc *WatchGetBlockTable(uint32_t *count)
{
    uint32_t active_count = 0u;
    for (uint32_t i = 0u; i < (uint32_t)(sizeof(g_watch_blocks) / sizeof(g_watch_blocks[0])); i++)
    {
        const WatchBlockDesc *block = &g_watch_blocks[i];
        if (block->is_active == NULL || block->is_active())
        {
            g_watch_active_blocks[active_count++] = *block;
        }
    }

    if (count != NULL)
    {
        *count = active_count;
    }
    return g_watch_active_blocks;
}

const WatchBlockDesc *WatchFindBlock(WatchBlockId id)
{
    for (uint32_t i = 0u; i < (uint32_t)(sizeof(g_watch_blocks) / sizeof(g_watch_blocks[0])); i++)
    {
        const WatchBlockDesc *block = &g_watch_blocks[i];
        if (block->id == id &&
            (block->is_active == NULL || block->is_active()))
        {
            return block;
        }
    }
    return NULL;
}

uint8_t WatchBlockIsActive(WatchBlockId id)
{
    const WatchBlockDesc *block = WatchFindBlock(id);
    if (block == NULL)
    {
        return 0u;
    }
    if (block->is_active == NULL)
    {
        return 1u;
    }
    return block->is_active();
}
