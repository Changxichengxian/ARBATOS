/*
 * SPDX-FileCopyrightText: 2026 ?? <2811158416@qq.com>
 * SPDX-License-Identifier: Apache-2.0
 *
 * Private implementation chunk included by watch.c.
 */

static uint8_t watch_block_active_always(void)
{
    return 1u;
}

static void watch_update_set_stage(watch_update_stage_e stage)
{
    g_watch.rtos.watch_update_stage = (uint8_t)stage;
    g_watch.rtos.watch_update_stage_tick_ms = HAL_GetTick();
}

#if WATCH_ENABLE_ARM_J0_UNITREE || WATCH_ENABLE_LOCOMOTION_WHEELLEG_MIT || WATCH_ENABLE_GIMBAL_SINGLE
static fp32 watch_rad_to_deg(fp32 rad)
{
    return rad * 57.29577951308232f;
}
#endif

#if WATCH_ENABLE_GIMBAL_SINGLE
static fp32 watch_ecd_to_deg(uint16_t ecd)
{
    const fp32 full_range = (g_config.gimbal.full_ecd_range != 0u) ?
                                (fp32)g_config.gimbal.full_ecd_range :
                                8192.0f;

    return (fp32)ecd * 360.0f / full_range;
}
#endif

#if WATCH_ENABLE_LOCOMOTION_WHEELLEG_MIT
static fp32 watch_abs_fp32(fp32 value)
{
    return (value >= 0.0f) ? value : -value;
}
#endif

#if WATCH_ENABLE_LOCOMOTION_CLASSIC
static uint8_t watch_block_active_locomotion_classic(void)
{
    return robot_profile_module_enabled(ROBOT_TASK_MODULE_CLASSIC_CHASSIS);
}
#endif

#if WATCH_ENABLE_LOCOMOTION_WHEELLEG_SERVO
static uint8_t watch_block_active_wheelleg_servo(void)
{
    return robot_profile_module_enabled(ROBOT_TASK_MODULE_WHEELLEG_SERVO);
}
#endif

#if WATCH_ENABLE_LOCOMOTION_WHEELLEG_MIT
static uint8_t watch_block_active_wheelleg_mit(void)
{
    return robot_profile_module_enabled(ROBOT_TASK_MODULE_WHEELLEG_MIT);
}
#endif

#if WATCH_ENABLE_GIMBAL_SINGLE
static uint8_t watch_block_active_gimbal_single(void)
{
    uint8_t active = robot_profile_module_enabled(ROBOT_TASK_MODULE_SINGLE_GIMBAL);
#if WATCH_ENABLE_GIMBAL_DUAL
    if (active == 0u)
    {
        active = robot_profile_module_enabled(ROBOT_TASK_MODULE_DUAL_YAW_GIMBAL);
    }
#endif
    return active;
}
#endif

#if WATCH_ENABLE_GIMBAL_DUAL
static uint8_t watch_block_active_gimbal_dual(void)
{
    return robot_profile_module_enabled(ROBOT_TASK_MODULE_DUAL_YAW_GIMBAL);
}
#endif

#if WATCH_ENABLE_SHOOT_RM
static uint8_t watch_block_active_shoot_rm(void)
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
static uint8_t watch_block_active_arm(void)
{
    return robot_profile_module_enabled(ROBOT_TASK_MODULE_ARM);
}
#endif

static watch_task_diag_entry_t *watch_task_diag_get(watch_task_id_e task_id)
{
    switch (task_id)
    {
    case WATCH_TASK_DEFAULT:
        return &g_watch.rtos.task.default_task;
    case WATCH_TASK_DETECT:
        return &g_watch.rtos.task.detect_task;
    case WATCH_TASK_IMU:
        return &g_watch.rtos.task.imu_task;
    case WATCH_TASK_GIMBAL_CONTROL:
#if WATCH_ENABLE_GIMBAL_SINGLE || WATCH_ENABLE_GIMBAL_DUAL
        return &g_watch.rtos.task.gimbal_control_task;
#else
        return NULL;
#endif
    case WATCH_TASK_CHASSIS_CONTROL:
#if WATCH_ENABLE_LOCOMOTION_CLASSIC
        return &g_watch.rtos.task.chassis_control_task;
#else
        return NULL;
#endif
    case WATCH_TASK_CAN_FEEDBACK_RX:
        return &g_watch.rtos.task.can_feedback_rx_task;
    case WATCH_TASK_CAN_COMMAND_TX:
        return &g_watch.rtos.task.can_command_tx_task;
    case WATCH_TASK_RC_SBUS:
        return &g_watch.rtos.task.rc_sbus_task;
    case WATCH_TASK_HOST_LINK:
        return &g_watch.rtos.task.host_link_task;
    case WATCH_TASK_ELRS:
        return &g_watch.rtos.task.elrs_task;
    case WATCH_TASK_ARM:
#if WATCH_ENABLE_ARM_J0_UNITREE
        return &g_watch.rtos.task.arm_task;
#else
        return NULL;
#endif
    case WATCH_TASK_WHEELLEG_MIT:
#if WATCH_ENABLE_LOCOMOTION_WHEELLEG_MIT
        return &g_watch.rtos.task.wheelleg_mit_task;
#else
        return NULL;
#endif
    default:
        return NULL;
    }
}

static watch_irq_diag_entry_t *watch_irq_diag_get(watch_irq_id_e irq_id)
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

void watch_diag_set_boot_stage(watch_boot_stage_e stage)
{
    watch_diag_push_stage(stage);
}

void watch_diag_mark_error_handler(uint32_t tick_ms, uint32_t ipsr)
{
    g_watch.fault.error_handler_count++;
    g_watch.fault.error_stage = g_watch.fault.boot_stage;
    g_watch.fault.error_tick_ms = tick_ms;
    g_watch.fault.error_ipsr = ipsr;
}

void watch_diag_set_error_args(uint32_t arg0, uint32_t arg1)
{
    g_watch.fault.error_arg0 = arg0;
    g_watch.fault.error_arg1 = arg1;
}

void watch_diag_mark_fatal(uint32_t reason, uint32_t task_handle, const char *task_name)
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

void watch_task_module_create_reset(void)
{
    memset(s_task_module_create, 0, sizeof(s_task_module_create));
}

void watch_task_module_create_result(uint8_t module, const char *name, uint32_t thread_handle, uint8_t state)
{
    watch_task_module_create_slot_t *slot = NULL;

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

void watch_task_beat(watch_task_id_e task_id)
{
    const uint32_t now_ms = HAL_GetTick();
    watch_task_diag_entry_t *entry = watch_task_diag_get(task_id);
    if (entry == NULL)
    {
        return;
    }

    if (entry->last_tick_ms != 0u &&
        (uint32_t)(now_ms - entry->last_tick_ms) < (uint32_t)ROBOT_PROFILE_WATCH_TASK_BEAT_MIN_PERIOD_MS)
    {
        return;
    }

    const uint64_t beat_start_us = rt_profiler_begin();
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
    rt_profiler_end(RT_PROFILER_WATCH_TASK_BEAT, beat_start_us);
}

void watch_task_wait(watch_task_id_e task_id)
{
    watch_task_diag_entry_t *entry = watch_task_diag_get(task_id);
    if (entry == NULL)
    {
        return;
    }

    taskENTER_CRITICAL();
    entry->wait_count++;
    taskEXIT_CRITICAL();
}

void watch_task_timeout(watch_task_id_e task_id)
{
    watch_task_diag_entry_t *entry = watch_task_diag_get(task_id);
    if (entry == NULL)
    {
        return;
    }

    taskENTER_CRITICAL();
    entry->timeout_count++;
    taskEXIT_CRITICAL();
}

void watch_task_error(watch_task_id_e task_id)
{
    watch_task_diag_entry_t *entry = watch_task_diag_get(task_id);
    if (entry == NULL)
    {
        return;
    }

    taskENTER_CRITICAL();
    entry->error_count++;
    taskEXIT_CRITICAL();
}

void watch_imu_set_stage(watch_imu_stage_e stage)
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

void watch_irq_hit(watch_irq_id_e irq_id)
{
    const uint32_t now_ms = HAL_GetTick();
    watch_irq_diag_entry_t *entry = watch_irq_diag_get(irq_id);
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

void watch_init(void)
{
    memset(&g_watch, 0, sizeof(g_watch));
#if WATCH_ENABLE_SHOOT_RM
    watch_prepare_motor_ids();
#endif

    if (manual_input_get_current_copy(&rc_snapshot) != 0u)
    {
        rc_src = &rc_snapshot;
    }
    else
    {
        rc_src = NULL;
    }
    ins_quat_src = get_INS_quat_point();
    ins_angle_src = get_INS_angle_point();
    ins_gyro_src = get_gyro_data_point();
    ins_accel_src = get_accel_data_point();

    watch_update();
}

void watch_update(void)
{
    g_watch.rtos.watch_update_enter_count++;
    watch_update_set_stage(WATCH_UPDATE_STAGE_RC);
    watch_copy_rc();
    watch_update_set_stage(WATCH_UPDATE_STAGE_NEWRC);
    watch_copy_newrc();
#if WATCH_ENABLE_COMM_COPY
    watch_update_set_stage(WATCH_UPDATE_STAGE_COMM);
    watch_copy_comm();
#endif
#if WATCH_ENABLE_RUNTIME_COPY
    watch_update_set_stage(WATCH_UPDATE_STAGE_RUNTIME);
    watch_copy_runtime();
#endif
    watch_update_set_stage(WATCH_UPDATE_STAGE_IMU);
    watch_copy_imu();
#if WATCH_ENABLE_LOCOMOTION_CLASSIC
    watch_update_set_stage(WATCH_UPDATE_STAGE_CHASSIS);
    watch_copy_chassis();
#endif
#if WATCH_ENABLE_GIMBAL_SINGLE
    watch_update_set_stage(WATCH_UPDATE_STAGE_GIMBAL);
    watch_copy_gimbal();
#endif
#if WATCH_ENABLE_SHOOT_RM
    watch_update_set_stage(WATCH_UPDATE_STAGE_SHOOT);
    watch_copy_shoot();
#endif
#if WATCH_ENABLE_ARM_J0_UNITREE
    watch_update_set_stage(WATCH_UPDATE_STAGE_ARM);
    watch_copy_arm_j0_unitree();
#endif
#if WATCH_ENABLE_LOCOMOTION_WHEELLEG_MIT
    watch_update_set_stage(WATCH_UPDATE_STAGE_WHEELLEG);
    watch_copy_wheelleg_mit();
#endif
    watch_update_set_stage(WATCH_UPDATE_STAGE_DIAG);
    watch_copy_diag();
    watch_update_set_stage(WATCH_UPDATE_STAGE_RTOS);
    watch_copy_rtos();
    watch_update_set_stage(WATCH_UPDATE_STAGE_DONE);
}

const watch_block_desc_t *watch_get_block_table(uint32_t *count)
{
    uint32_t active_count = 0u;
    for (uint32_t i = 0u; i < (uint32_t)(sizeof(g_watch_blocks) / sizeof(g_watch_blocks[0])); i++)
    {
        const watch_block_desc_t *block = &g_watch_blocks[i];
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

const watch_block_desc_t *watch_find_block(watch_block_id_e id)
{
    for (uint32_t i = 0u; i < (uint32_t)(sizeof(g_watch_blocks) / sizeof(g_watch_blocks[0])); i++)
    {
        const watch_block_desc_t *block = &g_watch_blocks[i];
        if (block->id == id &&
            (block->is_active == NULL || block->is_active()))
        {
            return block;
        }
    }
    return NULL;
}

uint8_t watch_block_is_active(watch_block_id_e id)
{
    const watch_block_desc_t *block = watch_find_block(id);
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
