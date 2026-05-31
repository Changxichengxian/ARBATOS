# 新机器人和新模块接入模板

这份模板用于新增一台机器人、换一种底盘、增加一个三云台机构，或者接入完全不同的模块。核心原则是先描述实例，再写控制器；少改项目入口，少复制固定分支。

## 1. 先写清实例

先列一张表，不急着写控制逻辑：

| 名字 | 类型 | 连接 | 配置来源 | 备注 |
|---|---|---|---|---|
| `motor.xxx0` | motor | CAN1 / RS485 | `g_config` 或设备表 | 稳定名字，控制器只认这个 |
| `sensor.xxx` | sensor | CAN / UART / SPI | 设备表 | 后续接入通用设备表 |
| `link.xxx` | link | USB / UART | 设备表 | 遥控、图传、上位机链路 |

当前代码里电机先通过 `robot_device_config.h` 生成兼容设备视图。以后真正改成设备数组时，优先改这层，不要让控制器直接读 `g_config.motor.yaw` 这类固定字段。

## 2. 绑定电机实例

电机命名用稳定实例名：

```c
static const char *const outputs[] = {
    "motor.gimbal_yaw0",
    "motor.gimbal_yaw1",
    "motor.gimbal_yaw2",
};
```

控制器启动时解析一次：

```c
static actuator_id_e output_ids[3];

if (motor_instance_resolve_actuator_ids(outputs, 3, output_ids, 3) != 3)
{
    return CONTROL_RESULT_BAD_ARGUMENT;
}
```

运行时按 id 或名字发命令：

```c
int16_t current[3] = {yaw0, yaw1, yaw2};
(void)motor_instance_cmd_set_current_ids(output_ids, current, 3);
```

## 3. 加控制器

新动作、新机构优先写成 `control_controller_t`：

```c
static const control_controller_t triple_yaw_controller = {
    .id = CONTROL_CONTROLLER_CUSTOM_BASE,
    .domain = CONTROL_DOMAIN_GIMBAL,
    .claim_mask = CONTROL_RESOURCE_GIMBAL_YAW,
    .name = "controller.triple_yaw",
    .meta = {
        .period_ms = 1u,
        .output_count = 3u,
        .outputs = outputs,
    },
    .enter = triple_yaw_enter,
    .update = triple_yaw_update,
    .exit = triple_yaw_exit,
};
```

注册和切换：

```c
(void)control_manager_register(&triple_yaw_controller);
(void)control_manager_request_switch_by_name("controller.triple_yaw",
                                             CONTROL_REASON_MODE_SWITCH);
```

如果多个控制器共用一个调度任务，可以用：

```c
(void)control_manager_update_due_all(now_ms, &context);
```

## 4. 加任务模块

确实需要新任务时，再加任务模块。已有项目都用 `app_task_bootstrap.h` 创建启用任务：

1. 给模块一个 ID。内置模块继续用 `ROBOT_TASK_MODULE_XXX`；项目私有模块从 `ROBOT_TASK_MODULE_CUSTOM_BASE` 往后取。
2. 在 `g_config.profile.task_modules[]` 里启用这个 ID。
3. 在对应 `freertos.c` 的 `module_tasks[]` 里加一行：`{id, &handle, create_fn}`。
4. 任务内部优先注册控制器、跑调度器，避免把具体机构写死在任务名里。

## 5. 三云台示例

推荐改动顺序：

1. 设备表里出现三个输出：`motor.gimbal_yaw0`、`motor.gimbal_yaw1`、`motor.gimbal_yaw2`。
2. 新增 `controller.triple_yaw`，`outputs` 填这三个名字。
3. 控制器 `enter` 阶段解析输出，`update` 阶段批量写电流。
4. 如果旧云台任务能承载，就只注册新控制器；如果需要独立调度，再加一个自定义任务模块。
5. `watch.runtime` 和 SD 日志会按设备、控制器实例记录，不需要单独加“三云台日志字段”。

## 6. 换底盘示例

推荐改动顺序：

1. 先改设备实例名字和数量，例如 `motor.drive_fl`、`motor.drive_fr`、`motor.steer_fl`。
2. 新增或替换底盘控制器，输入输出都写实例名。
3. 保留旧底盘控制器，直到新控制器能独立跑通。
4. 只在必须新增任务周期时改 `module_tasks[]`。

## 完成前检查

```powershell
powershell -ExecutionPolicy Bypass -File tools\check_all.ps1
powershell -ExecutionPolicy Bypass -File tools\check_all.ps1 -AllText
git diff --check
```

如果新增了设备、控制器、任务，至少确认：

- `watch.runtime` 能看到对应实例。
- SD 日志启动记录里有设备条目。
- 控制器输出都走 `motor_instance_*` 接口。
- 新模块没有继续扩大固定的云台、底盘、发射分支。
