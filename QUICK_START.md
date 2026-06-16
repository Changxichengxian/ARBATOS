# 新手快速上手

这份文档给刚接触 ARBATOS 的人看。它不追求把每个技术细节讲完，只回答一个问题：新接一辆车时，先改哪里，怎么一步步把车调起来。

如果你只是想知道仓库整体结构，看 `README.md`。如果你要真正接一辆新车、上车检查、调 PID 或看 SD 日志，直接看 `manual/README.md`。如果你已经在改某一层的代码，再看 `projects/README.md`、`Robotconfig/README.md`、`boards/README.md` 和 `shared/README.md`。

## 先记住四个目录

- `projects/<TARGET>/`：能打开、编译、下载的 Keil 工程，也是 GCC/CMake 生成路线的工程清单来源。
- `Robotconfig/<TARGET>/`：这台车的配置，主要是 PID、电机 ID、输入映射和任务模块。
- `boards/<BOARD>/`：这块控制板的外设适配，主要是 CAN、UART、SPI、IMU、按键、蜂鸣器。
- `shared/`：多台车共用的控制逻辑、通信、输入、电机、诊断、日志。

判断文件放哪很简单：

- 换一台车就要改的参数，放 `Robotconfig/`。
- 换一块板才要改的引脚和外设，放 `boards/`。
- 能被多台车复用的控制逻辑，放 `shared/`。
- 工程怎么编译、包含哪些文件，放 `projects/`。

## 新写一辆车的顺序

### 1. 找最像的现有车

先不要从零开始。找一台最接近的新车：

- 用 DJI C 板：优先看 `HERO-C` 或 `MINIWHEELEG-C`。
- 用 DJI A 板：优先看 `INFANTRY-A` 或 `CARRIER-A`。
- 用 DM MC02 H7：优先看 `HERO-M`、`SENTINEL-M` 或 `MINIWHEELEG-M`。

然后复制对应的 `Robotconfig/<TARGET>/` 和 `projects/<TARGET>/`，再改名字和工程包含路径。

### 2. 先配 profile 和任务模块

在新目标的 `Robotconfig/<TARGET>/config_operation.inc` 里先看 `.profile`：

- `task_module_count`：这台车启用多少个模块。
- `task_modules`：显式列出这台车要创建哪些任务，比如 `ROBOT_TASK_MODULE_CLASSIC_CHASSIS`、`ROBOT_TASK_MODULE_SINGLE_GIMBAL`。

任务没开，后面的 PID 和电机配得再对也不会跑。现在任务创建只看 `task_modules`，所以新车先从少量模块开始，确认后再加。

### 3. 再配电机装配

看 `Robotconfig/<TARGET>/config_hardware.inc` 里的 `.motor`，先把每个轴的电机型号和 CAN ID 填对：

- `chassis[]`：底盘轮子。
- `yaw`、`pitch`：云台轴。
- `friction[]`、`trigger`：摩擦轮和拨盘。
- `arm[]`：机械臂或轮腿实验用的关节。

不用的电机先把 `can_id` 设成 `0`。达妙、宇树和 RM 电机还要确认协议、控制模式、总线和限幅。新车第一次上电时，建议先只开一个子系统，不要一口气把所有电机都接上闭环。

### 4. 配输入和安全档

输入分两层：

- `manual_input`：决定 DBUS/SBUS、ELRS、图传遥控这些输入源怎么选。
- `input`：把遥控通道映射成“底盘前后、底盘左右、云台 yaw、云台 pitch、模式拨杆”这些语义输入。

优先改 `Robotconfig/<TARGET>/config_input.inc` 里的 `.input.axis` 和 `.input.sw`，控制任务里尽量不要直接写死遥控通道号。

安全档位置在 `config_input.inc` 的 `.manual_input.semantics` 里。现在 IMU 陀螺零偏微调也会看安全档：温度稳定后，遥控器未连接，或者云台和底盘都在安全档，才会采 3 秒静止数据做微调。

### 5. 配检测项

每个目标都有自己的 `detect_task.c`。新车调试时先把关键设备配进去：

- 遥控器或输入链路。
- 底盘电机、云台电机、拨盘、摩擦轮。
- 裁判系统、视觉、主机链路。
- TF/SD 卡、IMU、板级外设。

不要为了让灯变绿就关检测。检测项乱关，后面调车会很难判断到底是代码没跑、线没接，还是设备掉线。
检测项要和 `task_modules` 对上：开了底盘模块就关心底盘电机，开了云台模块就关心云台电机和 IMU，开了日志或主机链路就能看到对应服务状态。

## 编译路线怎么选

常用路线有两条：

- 用 KEIL：直接打开 `projects/<TARGET>/MDK-ARM/<TARGET>.uvprojx`，适合本地调试、下载和继续用 uVision。
- 用 GCC/CMake：从同一个 `.uvprojx` 生成命令行构建文件，适合没有 KEIL 的人做编译验证，或者接 VS Code、CLion 这类工具。

先检查本机工具：

```powershell
powershell -NoProfile -ExecutionPolicy Bypass -File .\tools\build.ps1 -Action probe
```

跑仓库检查：

```powershell
powershell -NoProfile -ExecutionPolicy Bypass -File .\tools\build.ps1 -Action check
```

用 GCC/CMake 编一个目标或全部目标：

```powershell
powershell -NoProfile -ExecutionPolicy Bypass -File .\tools\build.ps1 -Action gcc-build -Project HERO-C
powershell -NoProfile -ExecutionPolicy Bypass -File .\tools\build.ps1 -Action gcc-build -Project all
```

生成的构建文件在 `build/gcc/<TARGET>/`，不用提交。改了 KEIL 工程里的文件列表、宏、头文件路径、启动文件或 scatter 文件后，重新跑 `gcc-build` 就会刷新。

GCC/CMake 路线按 warning-clean（没有编译警告）维护。看到新的编译警告，优先改源码，不要只忽略构建输出。

## 第一次上电怎么调

### 1. 先无动力确认程序在跑

先不装弹、不接摩擦轮电源，底盘最好架空。

确认：

- Keil 能编译下载。
- 状态灯有变化。
- `g_watch` 能看到任务状态。
- 遥控输入有变化。
- CAN 收发统计不是全 0。

### 2. 先看 IMU

IMU 正常后再调云台和底盘。重点看：

- 温度能升到目标值并稳定。
- 姿态角不会明显跳变。
- 静止时 gyro 接近 0。
- 移动车体时角度方向符合预期。

陀螺仪零偏有两种路径：

- 正常上电：温度稳定后，遥控器未连接或处于安全档，静止 3 秒后自动微调一次。
- 专门校准：把 `g_config.operation.mode` 设成 `ROBOT_RUN_MODE_CALIBRATION`，`g_config.operation.cali_target` 设成 `ROBOT_CALI_TARGET_IMU_GYRO`，温度升到 40 度并稳定后，静止采 30 秒，然后保存到 Flash。

做专门校准时不要碰车，也不要让风扇、线束、桌面震动影响机体。

### 3. 再看 CAN 反馈

先确认反馈，再给输出：

- 电机在线状态正常。
- CAN ID 和反馈 ID 对得上。
- 反馈转速和手转方向符合预期。
- 电机型号和协议没有填错。

如果反馈都不稳定，先别调 PID。

### 4. 一个子系统一个子系统调

推荐顺序：

1. IMU 和输入。
2. 云台 yaw 单轴。
3. 云台 pitch 单轴。
4. 底盘低速。
5. 摩擦轮。
6. 拨盘。
7. 裁判、视觉、日志、遥测。

每一步都先小输出、低限幅、架空或卸载。确认方向对、反馈对、限位对，再逐步提高参数。

## 常见问题先看哪里

### 任务没跑

- 看 `config_operation.inc` 里的 `.profile.task_modules` 是否开了对应任务。
- 看 `projects/<TARGET>/Core/Src/freertos.c` 或 H7 的 `board_freertos.c` 是否创建了任务。
- 看 `g_watch` 里的任务状态和运行计数。

### 电机不动

- 先看 `config_hardware.inc` 里的 `.motor` 型号、CAN ID、总线、反馈 ID。
- 再看 CAN 收发统计和电机在线检测。
- 确认控制任务是否真的写了执行器命令。
- 最后再看 PID 输出和限幅。

### 电机方向反了

- 底盘轮子优先改 `config_tuning.inc` 里的 `.chassis.motor_dir`。
- 云台方向优先改 `yaw_turn`、`pitch_turn` 或安装矩阵相关配置。
- 不要靠换 CAN ID 来掩盖方向问题。

### 遥控没反应

- 看输入源是否在线。
- 看 `manual_input_get_active_source()` 当前选的是哪个源。
- 看 `control_input` 的语义轴和语义开关有没有变化。
- 检查 `config_input.inc` 里的 `.input.axis`、`.input.sw` 映射。

### 姿态漂或上电不准

- 先确认 IMU 温控稳定。
- 做一次 `ROBOT_RUN_MODE_CALIBRATION + ROBOT_CALI_TARGET_IMU_GYRO` 专门校准。
- 正常上电后保持安全档和静止，等 3 秒微调完成。
- 如果车一上电手还扶着，微调会因为检测到扰动而放弃。

### AUX 调参改了没效果

- 只有 `config.c` 的 `g_config_blocks` 表里列出的字段能临时改。
- `config_hardware.inc` 里的 `.motor` 这类装配信息默认不走 AUX 临时调参，改完要重新编译下载。
- AUX 只改 RAM 里的当前值，重启会回到 `config_*.inc` 默认值。

## 什么时候该改 shared

先问自己一句：这个改动以后别的车会不会也要用？

- 会复用：放 `shared/`。
- 只是这台车的参数：放 `Robotconfig/`。
- 只是这块板的引脚或外设：放 `boards/`。
- 只是工程包含文件和启动入口：放 `projects/`。

新手最容易犯的错是把参数写进共享控制任务里。短期看起来快，后面换车会非常痛苦。

## 继续看

更完整的操作步骤放在：

- `manual/new-target.md`：新车接入流程。
- `manual/bringup-checklist.md`：上车检查清单。
- `manual/pid-tuning.md`：PID 调试流程。
- `manual/sdlog.md`：SD 日志和复盘。

## 最小交付清单

新车能算“初步接起来”，至少要满足：

- 目标 Keil 工程能从干净状态编译。
- `config_operation.inc`、`config_hardware.inc`、`config_input.inc` 里的任务、电机、输入映射和安全档配置正确。
- IMU 温控和零偏校准路径可用。
- 遥控输入、CAN 反馈、状态灯、`g_watch` 都能观察。
- 底盘、云台、射击每个子系统都能单独关闭或单独测试。
- `detect_task.c` 能反映关键设备在线状态。

做到这些，再开始追求手感、性能和复杂功能。
