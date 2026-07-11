/*
 * SPDX-FileCopyrightText: 2026 ?? <2811158416@qq.com>
 * SPDX-License-Identifier: Apache-2.0
 *
 * Private implementation chunk included by Watch.c.
 */

static void WatchCopyDiag(void)
{
    VisionLinkDebug vision = {0};
    ExternalMotionIntent motion = {0};
    ChassisAlgorithmDebug algorithm = {0};
    RobotLifecycleSnapshot lifecycle;
    uint32_t now_ms;
    uint32_t motion_age_ms = 0u;
    uint8_t dbus_error;
    uint8_t manual_online;
    uint8_t chassis_safe;
    uint8_t output_allowed;
    uint8_t robot_ok;
    uint8_t ctrl_ok;
    uint8_t motion_on;

    memset(&g_watch.diag, 0, sizeof(g_watch.diag));
    memset(&lifecycle, 0, sizeof(lifecycle));

    now_ms = HAL_GetTick();
    (void)RobotLifecycleGetSnapshot(&lifecycle);
#if ROBOT_TASK_BUILD_HOST_LINK
    VisionLinkGetDebug(&vision);
#endif
#if ROBOT_TASK_BUILD_CLASSIC_CHASSIS
    ChassisAlgorithmDebugRead(&algorithm);
#endif
    if (vision.lc_last_tick_ms != 0u)
    {
        g_watch.diag.lc_age_ms = now_ms - vision.lc_last_tick_ms;
    }
    if (algorithm.last_ms != 0u)
    {
        g_watch.diag.alg_age_ms = now_ms - algorithm.last_ms;
    }

    dbus_error = DetectIsError(DBUS_TOE) ? 1u : 0u;
    manual_online = (uint8_t)(s_watch_manual_input_valid != 0u &&
                              s_watch_manual_input.online != 0u);
    output_allowed = lifecycle.output_allowed;
    robot_ok = robot_mode_allow_chassis();
    ctrl_ok = (ControlMgrActiveId(ControlDomainChassis) == ControlIdClassicChassis) ? 1u : 0u;
    chassis_safe = (manual_online != 0u) ?
        ControlInputSwitchIsPos(
            s_watch_manual_input.control.sw[INPUT_SW_CHASSIS_MODE],
            s_watch_manual_input.semantics.ChassisSafePos) :
        1u;

    if (dbus_error != 0u) g_watch.diag.gate_bits |= (1u << 0);
    if (manual_online == 0u) g_watch.diag.gate_bits |= (1u << 1);
    if (chassis_safe != 0u || lifecycle.manual_safe != 0u) g_watch.diag.gate_bits |= (1u << 2);
    if (output_allowed == 0u) g_watch.diag.gate_bits |= (1u << 3);
    if (robot_ok == 0u) g_watch.diag.gate_bits |= (1u << 4);
    if (ctrl_ok == 0u) g_watch.diag.gate_bits |= (1u << 5);

    motion_on = ExternalMotionIntentReadLatest(&motion, &motion_age_ms) ? 1u : 0u;
    if (motion_on != 0u)
    {
        g_watch.diag.motion_on = 1u;
        g_watch.diag.motion_age_ms = motion_age_ms;
    }

    g_watch.diag.usb_rx = vision.rx_callback_count;
    g_watch.diag.lc_ok = vision.lc_crc_ok_count;
    g_watch.diag.lc_fail = vision.lc_crc_fail_count;
    g_watch.diag.crc_rx = vision.lc_last_crc_rx;
    g_watch.diag.crc_calc = vision.lc_last_crc_calc;
    g_watch.diag.alg = algorithm.count;
    g_watch.diag.mode = vision.lc_last_mode;
    g_watch.diag.frame = vision.lc_last_frame;
    g_watch.diag.flags = vision.lc_last_flags;
    g_watch.diag.t10ms = vision.lc_last_timeout_10ms;
    g_watch.diag.vx_cmps = vision.lc_last_vx_cmps;
    g_watch.diag.vy_cmps = vision.lc_last_vy_cmps;
    g_watch.diag.wz_mradps = vision.lc_last_wz_mradps;
    g_watch.diag.alg_vx_cmps = algorithm.vx_cmps;
    g_watch.diag.alg_vy_cmps = algorithm.vy_cmps;
    g_watch.diag.alg_wz_mradps = algorithm.wz_mradps;

#if WATCH_ENABLE_LOCOMOTION_CLASSIC
    g_watch.diag.vx_set = g_watch.chassis.vx_set;
    g_watch.diag.vy_set = g_watch.chassis.vy_set;
    g_watch.diag.wz_set = g_watch.chassis.wz_set;
    for (uint8_t i = 0u; i < 4u; i++)
    {
        g_watch.diag.cur[i] = g_watch.chassis.motor_current[i];
        if (g_watch.chassis.motor_current[i] != 0)
        {
            g_watch.diag.cur_on = 1u;
        }
    }
#endif

    if (vision.rx_callback_count == 0u)
    {
        g_watch.diag.reason = 1u; // USB CDC 没进接收回调
    }
    else if (vision.lc_head_count == 0u)
    {
        g_watch.diag.reason = 2u; // 没识别到 LC 包头
    }
    else if (vision.lc_crc_ok_count == 0u)
    {
        g_watch.diag.reason = 3u; // LC CRC 没通过
    }
    else if (vision.lc_valid_count == 0u)
    {
        g_watch.diag.reason = 4u; // LC 包内容非法
    }
    else if (motion_on == 0u)
    {
        g_watch.diag.reason = 5u; // 导航命令没有保持活跃，通常是超时
    }
    else if (manual_online == 0u)
    {
        g_watch.diag.reason = 6u; // 手动输入离线，底盘电流会被清零
    }
    else if (chassis_safe != 0u || lifecycle.manual_safe != 0u)
    {
        g_watch.diag.reason = 7u; // 安全档仍然有效
    }
    else if (output_allowed == 0u)
    {
        g_watch.diag.reason = 8u; // 生命周期不允许输出
    }
    else if (robot_ok == 0u)
    {
        g_watch.diag.reason = 9u; // 机器人模式不允许底盘
    }
    else if (ctrl_ok == 0u)
    {
        g_watch.diag.reason = 10u; // 底盘控制器没有占用控制域
    }
    else if (g_watch.diag.vx_set < 0.001f &&
             g_watch.diag.vx_set > -0.001f &&
             g_watch.diag.vy_set < 0.001f &&
             g_watch.diag.vy_set > -0.001f &&
             g_watch.diag.wz_set < 0.001f &&
             g_watch.diag.wz_set > -0.001f)
    {
        g_watch.diag.reason = 11u; // 命令进来了，但底盘目标仍是 0
    }
    else if (g_watch.diag.cur_on == 0u)
    {
        g_watch.diag.reason = 12u; // 底盘目标有了，但四个电流还是 0
    }
    else
    {
        g_watch.diag.reason = 0u; // Watch 侧看到已经有底盘电流输出
    }
}

#if INCLUDE_uxTaskGetStackHighWaterMark
// Optional stack watermark globals (defined in some targets). Provide weak defaults
// so the shared watch can link even if a target doesn't export them.
#if WATCH_ENABLE_GIMBAL_SINGLE || WATCH_ENABLE_GIMBAL_DUAL
__weak uint32_t GimbalHighWater = 0u;
#endif
#if WATCH_ENABLE_LOCOMOTION_CLASSIC
__weak uint32_t ChassisHighWater = 0u;
#endif
__weak uint32_t DetectTaskStack = 0u;
__weak uint32_t CalibrateTaskStack = 0u;
#endif

static void WatchCopyRtos(void)
{
    TaskHandle_t current_task = NULL;
    const char *current_name = NULL;

    g_watch.rtos.heap_free = heap_get_free();
    g_watch.rtos.heap_ever_free = heap_get_ever_free();
    g_watch.rtos.WatchUpdateCount++;
    g_watch.rtos.WatchUpdateTickMs = HAL_GetTick();
    g_watch.rtos.scheduler_state = (uint32_t)xTaskGetSchedulerState();
    g_watch.rtos.task_count = (uint32_t)uxTaskGetNumberOfTasks();
    current_task = xTaskGetCurrentTaskHandle();
    g_watch.rtos.current_task_handle = (uint32_t)current_task;
    current_name = pcTaskGetTaskName(NULL);
    memset(g_watch.rtos.current_task_name, 0, sizeof(g_watch.rtos.current_task_name));
    if (current_name != NULL)
    {
        (void)strncpy(g_watch.rtos.current_task_name, current_name, sizeof(g_watch.rtos.current_task_name) - 1u);
    }
#if INCLUDE_uxTaskGetStackHighWaterMark
    g_watch.rtos.stack_default = uxTaskGetStackHighWaterMark(NULL);
#if WATCH_ENABLE_GIMBAL_SINGLE || WATCH_ENABLE_GIMBAL_DUAL
    g_watch.rtos.stack_gimbal = GimbalHighWater;
#endif
#if WATCH_ENABLE_LOCOMOTION_CLASSIC
    g_watch.rtos.stack_chassis = ChassisHighWater;
#endif
    g_watch.rtos.stack_detect = DetectTaskStack;
    g_watch.rtos.stack_calibrate = CalibrateTaskStack;
#else
    g_watch.rtos.stack_default = 0u;
#if WATCH_ENABLE_GIMBAL_SINGLE || WATCH_ENABLE_GIMBAL_DUAL
    g_watch.rtos.stack_gimbal = 0u;
#endif
#if WATCH_ENABLE_LOCOMOTION_CLASSIC
    g_watch.rtos.stack_chassis = 0u;
#endif
    g_watch.rtos.stack_detect = 0u;
    g_watch.rtos.stack_calibrate = 0u;
#endif
}
