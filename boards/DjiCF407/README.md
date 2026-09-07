# DJI C F407

这里保留开发板支持，独立于车型。旧车型删掉后，新车仍可复用此板。

- `zephyr/`：芯片、时钟、引脚、外设和 OpenOCD 配置；Zephyr 板名 `dji_c_f407`。
- `bsp/` 和已有 `devices/`：原有板级接口、安装方向及驱动参考，保留供后续移植查阅。依赖 STM32 HAL 的实现不直接加入 Zephyr 构建。
- 当前系统适配在 `shared/zephyr/`，按开发板选择传感器，不再绑定已删除车型的名字。

本次按 Git 951857f 恢复 C 板引脚与时钟，没有用 A 板原理图推断 C 板接线。BMI088/IST8310 适配继续保留。

本次独立板级工程用于确认设备树与编译，未进行实板验证。新增车型流程见 [projects/README.md](../../projects/README.md)。
