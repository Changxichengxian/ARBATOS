# DJI A F427

这里保留开发板支持，独立于车型。旧车型删掉后，新车仍可复用此板。

- `zephyr/`：芯片、时钟、引脚、外设和 OpenOCD 配置；Zephyr 板名 `dji_a_f427`。
- `bsp/` 和已有 `devices/`：原有板级接口、安装方向及驱动参考，保留供后续移植查阅。依赖 STM32 HAL 的实现不直接加入 Zephyr 构建。
- 当前系统适配在 `shared/zephyr/`，按开发板选择传感器，不再绑定已删除车型的名字。

原理图核对及修正见 [PinAudit.md](PinAudit.md)。A 板采用 MPU6500 系列接口；磁力计通过 IMU 的辅助总线接入，目前仍为六轴融合。

本次独立板级工程用于确认设备树与编译，未进行实板验证。新增车型流程见 [projects/README.md](../../projects/README.md)。
