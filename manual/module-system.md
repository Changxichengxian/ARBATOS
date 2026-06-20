# 机器人模块声明

ARBATOS 这里的模块，只覆盖机器人下位机常见模块：输入、通信、IMU、底盘、轮腿、云台、机械臂、日志、诊断和启动服务。它暂时不追求传感器项目、遥控器项目或非机器人项目的通用性。

## 模块要说清楚什么

每个任务模块都要在 `shared/application/robot/RobotModule.h` 里说清楚：

- `name`：模块自己的名字，例如 `module.classic_chassis`。
- `taskName`：对应旧任务名，例如 `task.classic_chassis`，用于兼容现有任务创建和调试视图。
- `kind`：模块类型，例如 input、comm、device、service、control、safety。
- `defaultPeriodMs` / `defaultBudgetUs`：默认周期和时间预算；运行时可由 `RobotModulePeriodMs()` 读到目标配置里的实际周期。
- `defaultStackWords` / `defaultPriority`：默认栈和优先级，只作为声明和检查依据，现阶段不替代各项目的任务创建参数。
- `requires`：模块依赖哪些资源，例如 `RobotResourceLowCmd`、`RobotResourceCan1`、`RobotResourceImu`。
- `provides`：模块提供什么能力，例如 `RobotResourceChassisOutput`、`RobotResourceMotorFeedback`。

## 新增模块流程

1. 在 `shared/application/robot/RobotConfigSchema.h` 增加 `ROBOT_TASK_MODULE_*` 枚举。
2. 在 `shared/application/robot/RobotTaskProfile.h` 的 `RobotProfileKnownModules()` 里补旧任务名。
3. 在 `shared/application/robot/RobotModule.h` 里补模块声明和资源数组。
4. 在目标的 `Robotconfig/<target>/RobotConfig.c` 里把模块加入 `task_modules`。
5. 如果这个模块确实创建 FreeRTOS 任务，在项目或板级启动文件的 `module_tasks` 里补创建函数。
6. 跑 `tools/CheckAll.ps1`，让脚本检查枚举、任务名、模块声明和资源名是否一致。

## 当前边界

现阶段模块声明只进入 Watch 和检查脚本。它会把模块类型、周期、预算、栈、优先级、依赖数量和产出数量放进 `g_watch.runtime.task_module[]`，方便实车观察。

任务创建仍然走 `AppTaskBootstrap.h` 和各项目现有 `module_tasks` 表。等声明表稳定后，再考虑把任务创建参数也收进模块声明。
