# ARBATOS Zephyr 算法兼容层

这个目录替代 Keil 工程里的两个 ARMCC 二进制库：

- `AHRS.lib`
- `arm_cortexM4lf_math.lib`

实现直接复用 `tools/build/gcc_support/algorithm` 中已经用于 GCC 构建的源码。
`AhrsZephyr.c` 和 `ArmMathZephyr.c` 是稳定的 Zephyr 编译入口，不要再同时编译被
包含的两个 GCC 源文件，否则会产生重复符号。

## 七目标覆盖

| 目标 | 历史工程依赖 | Zephyr 兼容方式 |
| --- | --- | --- |
| HERO-C | AHRS.lib、arm_cortexM4lf_math.lib | 两个包装源 |
| MINIWHEELEG-C | AHRS.lib、arm_cortexM4lf_math.lib | 两个包装源 |
| CARRIER-A | AHRS.lib、arm_cortexM4lf_math.lib | 两个包装源 |
| INFANTRY-A | AHRS.lib、arm_cortexM4lf_math.lib | 两个包装源 |
| HERO-M | Mc02Compat.c 弱 sin/cos | `ArmMathZephyr.c` 提供同语义弱符号；AHRS 包装可统一编译 |
| MINIWHEELEG-M | Mc02Compat.c 弱 sin/cos | 同上 |
| SENTINEL-M | Mc02Compat.c 弱 sin/cos | 同上 |

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
zephyr/port/algorithm/AhrsZephyr.c
zephyr/port/algorithm/ArmMathZephyr.c
```

需要优先加入的头文件目录：

```cmake
zephyr/port/algorithm/include
```

共享构建仍需自身已有的：

```cmake
shared/components/algorithm
shared/components/support
```

不要链接 ARMCC `.lib`，也不要同时添加
`tools/build/gcc_support/algorithm/{AHRS_gcc,arm_math_compat}.c`。

## 已做编译验证

使用当前 Zephyr 4.4 和 Zephyr SDK 1.0.1，`_smoke` 示例已在以下目标完成编译和链接：

- `dm_mc02_h7/stm32h723xx`
- `dji_c_f407/stm32f407xx`
- `dji_a_f427/stm32f427xx`

示例同时编入共享 `AhrsMiddleware.c`。H7 验证还编入 `HERO-M/Mc02Compat.c`，确认
两处同语义弱 `arm_sin_f32()`/`arm_cos_f32()` 实现可以共同链接。

## 数值验证边界

这个迁移保持 GCC 支持实现的行为，并不声称和闭源 `AHRS.lib` 逐位一致。实车前仍需
回放静止、匀速旋转、快速俯仰、磁力计缺失和加速度异常数据，比较四元数归一化、
欧拉角方向、收敛时间和控制环允许的误差。
