# ARBATOS Zephyr 算法兼容层

这个目录替代 Keil 工程里的两个 ARMCC 二进制库：

- `AHRS.lib`
- `arm_cortexM4lf_math.lib`

实现已完整移入 `AhrsZephyr.c` 和 `ArmMathZephyr.c`，不再依赖已删除的 GCC 转换工具目录。

## 当前实现

A、C、M 板共用这两个源码实现，新增车型不需要恢复已删除的 ARMCC 二进制库。

共享与目标源码的实际 CMSIS-DSP 调用只有：

- `arm_sin_f32()`
- `arm_cos_f32()`

因此 `include/arm_math.h` 只声明 `float32_t` 和这两个函数。它必须排在
`shared/components/algorithm/Include` 之前，避免旧版 7000 多行 CMSIS-DSP 头文件
把大量未链接 API 带回 Zephyr。以后出现新的 `arm_*` 调用，应增加真实实现，或者
正式接入 Zephyr 的 CMSIS-DSP 模块。

`include/AHRS.h` 只处理历史 `AHRS.h`/`Ahrs.h` 大小写差异，最终声明仍来自共享
公共头。

## CMake 集成

需要加入的源文件：

```cmake
shared/zephyr/port/algorithm/AhrsZephyr.c
shared/zephyr/port/algorithm/ArmMathZephyr.c
```

需要优先加入的头文件目录：

```cmake
shared/zephyr/port/algorithm/include
```

共享构建仍需自身已有的：

```cmake
shared/components/algorithm
shared/components/support
```

不要链接 ARMCC `.lib`，也不要从 Git 历史重复加入同一算法的旧实现。

## 编译验证

迁移阶段三板算法 `_smoke` 工程曾编译链接通过；当前板级独立入口见 [tests/Boards](../../../../tests/Boards/README.md)，HERO-M 使用正式工程构建。历史编译结果不代替每次改动后的验证。

## 数值验证边界

这个迁移保持 GCC 支持实现的行为，并不声称和闭源 `AHRS.lib` 逐位一致。实车前仍需
回放静止、匀速旋转、快速俯仰、磁力计缺失和加速度异常数据，比较四元数归一化、
欧拉角方向、收敛时间和控制环允许的误差。
