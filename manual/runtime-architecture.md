# 运行层当前状态和演进方向

这份文档按当前代码状态记录 ARBATOS 从 RoboMaster 多车型工程，往更通用机器人控制运行层演进的方向。

目标不是把更多机器人硬塞进“云台、底盘、射击”这些固定概念里，而是让新机器人由下面几类东西组合出来：

- 设备实例：电机、IMU、编码器、串口链路、传感器。
- 控制器实例：平衡、轨迹、关节、轮式运动、云台指向、机构动作。
- 数据连接：控制器读哪些输入，写哪些输出。
- 调度属性：周期、优先级、超时、安全策略。
- 观察接口：状态、诊断、日志、遥测。

## 当前代码状态

当前代码已经不只是“先在电机侧补一层实例名”，运行层的几块基础已经接上：

- `g_config.profile.task_modules` 是任务创建的入口。`app_task_bootstrap.h` 会按配置表创建启用任务。
- `g_config.devices` 已经进入配置层，`robot_device_config.h` 负责统一解析设备条目。
- `motor_instance_refresh()` 已经从设备表生成电机实例，控制任务可以按稳定实例名绑定输出。
- `LowCmd` 是控制任务到执行器发送任务之间的统一命令缓存。底盘、云台、射击等高频任务已经使用预绑定输出，循环里按绑定写电流。
- `control_manager` 已经提供控制域、控制器注册、资源声明、切换、停止和故障状态。默认控制器由 `robot_control_registry.h` 按 profile 注册。
- `watch.runtime` 已经能按任务、设备、电机、控制器、控制域收集运行实例。SD 日志启动记录也会写设备条目。
- 底盘、云台、CAN 发送、轮腿和 watch 等大任务入口已经拆成“主 `.c` + 私有 `.inc` 实现块”。原 `.c` 仍是唯一编译单元，私有块用于把日志、快照、调参、协议打包和控制辅助分开阅读。
- 安全保护已经存在，但当前主要分布在各任务和控制环里，例如安全档、运行编排、离线检测、限幅、轮腿 fault。后续仍在往控制器或调度策略收束。

当前仍保留兼容层。旧代码可以继续按固定 actuator id 写命令：

```c
Motor4
Motor6
LowCmdSetCurrent(Motor4, yaw_current);
```

新代码可以逐步改成按实例查询：

```c
const motor_instance_t *m = motor_instance_find_by_name("motor.yaw");
MotorId id = motor_instance_actuator_id(m);
```

如果只是发命令或读反馈，也可以直接按实例名走薄包装：

```c
motor_instance_cmd_set_current("motor.yaw", yaw_current);
motor_instance_feedback_get_copy("motor.yaw", &feedback);
```

如果一个控制器要同时管多个执行器，可以先把名字解析成 `MotorId`，后面循环里直接按 id 批量发命令：

```c
static const char *const yaw_outputs[] = {
    "motor.yaw0",
    "motor.yaw1",
    "motor.yaw2",
};

static MotorId yaw_ids[3];

void bind_outputs(void)
{
    if (motor_instance_resolve_actuator_ids(yaw_outputs, 3, yaw_ids, 3) != 3)
    {
        return;
    }
}

void run_outputs(void)
{
    int16_t current[3] = {yaw0_current, yaw1_current, yaw2_current};

    (void)motor_instance_cmd_set_current_ids(yaw_ids, current, 3);
}
```

这样后续控制器不需要只认识 `yaw`、`pitch`、`chassis0` 这类固定角色，可以先绑定到一个稳定的设备实例名。等配置层改成真正的设备表后，这些名字可以从配置来，而不是写死在代码里。

## 迁移原则

1. 旧接口先保留。
   现有车要能继续编译和上车，不为了架构升级打断当前调试。

2. 新接口先旁路接入。
   新机器人、新机构、新实验任务优先用实例表和控制器表；只有确实需要新的调度周期或独立任务栈时，才新增 `ROBOT_TASK_MODULE_XXX` 或项目私有模块。

3. 先统一执行器，再统一控制器。
   执行器是所有控制器最终写入的地方，先把这里从固定角色过渡到实例表，收益最大。

4. 调度保持静态。
   不做运行时动态加载，不引入堆分配。控制器注册、设备表、实例数量仍然用固定数组和编译期上限。

5. 日志和诊断按实例遍历。
   以后看到几个电机、几个控制器，就记录几个实例的状态，而不是为每种机器人形态单独写一套日志字段。

## 推荐路线

### 阶段 1：执行器实例化

- 保留 `MotorId`。
- 给 `motor_instance_t` 补稳定实例名。
- 提供按名字查找、取 actuator id、发命令、取电机配置、取反馈的接口。
- 提供一组名字解析、一组电流命令、一组反馈读取的接口，减少多电机控制器里的重复代码。
- 新代码优先从 `motor_instance_find_by_name()` 或后续配置绑定表拿执行器。

### 阶段 2：设备表进入配置

把当前这种固定字段：

```c
g_config.motor.yaw
g_config.motor.pitch
g_config.motor.arm[0]
```

逐步过渡成设备表：

```c
g_config.devices.motor[i] = {
    .name = "motor.left_front_joint",
    .model = ...,
    .can_bus = ...,
    .can_id = ...,
}
```

旧字段可以先由设备表生成，或者继续作为兼容层存在一段时间。

当前已经补了 `g_config.devices` 设备表，并由 `robot_device_config.h` 统一读取。旧的 `g_config.motor.*` 字段还保留给具体电机参数；设备表负责说明“有哪些设备实例”，旧字段负责说明“这个电机怎么配置”。

```c
robot_config_device_t device;

for (uint8_t i = 0; i < robot_config_device_count(); i++)
{
    if (robot_config_device_get(i, &device))
    {
        /* device.name / device.kind / device.config */
    }
}
```

电机仍然有 `robot_config_motor_device_t` 这种更具体的读取方式，`motor_instance_refresh()` 已经改成从这层读取。后面扩展传感器、链路或非电机执行器时，优先扩展设备表和 `robot_device_config.h`，电机实例和控制器不用跟着大改。

控制器也可以直接按自己的输入/输出名字解析设备：

```c
robot_config_device_binding_t devices;

if (robot_config_device_bind_controller(controller, &devices))
{
    /* devices.inputs[i] / devices.outputs[i] */
}
```

### 阶段 3：控制器实例化

控制器不再按“云台任务、底盘任务”扩张，而是按实例描述：

```c
static const char *const triple_yaw_outputs[] = {
    "motor.yaw0",
    "motor.yaw1",
    "motor.yaw2",
};

static const control_controller_t triple_yaw_controller = {
    .id = CONTROL_CONTROLLER_CUSTOM_BASE,
    .domain = CONTROL_DOMAIN_GIMBAL,
    .name = "controller.triple_yaw",
    .meta = {
        .period_ms = 1,
        .output_count = 3,
        .outputs = triple_yaw_outputs,
    },
    .enter = triple_yaw_enter,
    .update = triple_yaw_update,
};
```

控制器进入时把输出名字绑定成 id，运行时只发一组命令：

```c
static MotorId triple_yaw_ids[3];

static control_result_e triple_yaw_enter(const control_controller_t *controller,
                                         control_context_t *context)
{
    (void)context;

    if (motor_instance_resolve_controller_outputs(controller, triple_yaw_ids, 3) !=
        controller->meta.output_count)
    {
        return CONTROL_RESULT_BAD_ARGUMENT;
    }

    return CONTROL_RESULT_OK;
}

static control_result_e triple_yaw_update(const control_controller_t *controller,
                                          control_context_t *context)
{
    int16_t current[3];

    (void)controller;
    (void)context;

    current[0] = yaw0_current;
    current[1] = yaw1_current;
    current[2] = yaw2_current;

    return motor_instance_cmd_set_current_ids(triple_yaw_ids, current, 3) ?
           CONTROL_RESULT_OK :
           CONTROL_RESULT_BAD_ARGUMENT;
}
```

切换控制器也可以按名字走：

```c
(void)control_manager_request_switch_by_name("controller.triple_yaw",
                                             CONTROL_REASON_MODE_SWITCH);
```

控制器代码只关心它拿到的输入和输出，不关心这台机器人是不是 RoboMaster。

这一步当前已经有 `control_manager`、控制器元信息、注册表、按名字切换、按周期更新和运行时观察。旧的底盘、云台、射击任务仍然保留自己的主循环和状态机，新控制器优先声明输入、输出、周期和资源占用。

### 阶段 4：调度和观察统一

- 当前 `watch.runtime` 已经按设备表和控制器表遍历运行实例。
- 当前 SD 日志启动记录会写 build info、原始配置和运行时设备条目。
- 后续少量周期调度任务负责跑控制器实例，逐步减少“一个机构一套任务主循环”的写法。
- 后续安全策略继续往控制器或调度层集中，保留各子系统必要的本地限幅和故障处理。
- RoboMaster 相关逻辑保留为一组控制器和配置，不作为整个项目结构的中心。

`watch` 里的通用运行时块 `runtime.instances` 会按实例收集：

- 当前目标启用的任务模块名字。
- 电机实例数量、启用数量、名字、角色、bus 和 actuator id。
- 配置设备数量，统一条目表里的设备项来自 `robot_config_device_get()`。
- 控制器实例数量、名字、周期、输入输出数量和激活状态。
- 每个控制域当前激活的控制器、待处理请求和统计计数。
- 一张统一条目表：任务、设备、控制器、控制分组都用 `runtime_instance_ref_t` 表示，上位机可以先按名字和状态遍历。

这一步的价值是后续新增机器人时，观察和诊断可以先看“有哪些实例在跑”，不需要为每种机器人重新写一套观察字段。

## 判断一项新改动放哪里

- 新硬件：先抽象成设备实例。
- 新控制算法：先抽象成控制器实例。
- 新机器人形态：优先改配置和实例绑定表。
- 新协议：放到设备驱动或传输层。
- 新安全规则：优先放到控制器或调度策略里；迁移没完成前，可以先放在对应子系统任务里，但不要散到具体车型代码里。
