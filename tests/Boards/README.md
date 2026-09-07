# 不依赖车型的板级构建检查

此工程验证 A、C 板在旧车型删除后仍可独立使用。A 板同时编译 SDMMC 与 MPU6500 接口，并在编译时断言原理图确认的晶振、DBUS、按键、指示灯和 SD 检测配置。C 板检查原有外设定义与共用按键适配。

准备好 Zephyr 4.4 与 SDK 环境后，在仓库根目录执行（建议设置 `CMAKE_BUILD_PARALLEL_LEVEL=2`）：

```powershell
west build -s tests/Boards -b dji_a_f427 -d local/build/board-check-dji_a_f427 -- -DEXTRA_CONF_FILE=a-sd.conf
west build -s tests/Boards -b dji_c_f407 -d local/build/board-check-dji_c_f407
```

2026-09-08 两个工程均编译、链接通过。A 板 FLASH 33,292 B、RAM 6,720 B；C 板 FLASH 27,704 B、RAM 5,440 B。本机完整日志在 `local/build/board-check-*.log`。

这是编译检查，不是可下载到现有车辆的整车固件；本次没有执行烧录和实板测试。
