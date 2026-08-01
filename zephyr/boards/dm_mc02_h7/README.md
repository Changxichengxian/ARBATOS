# Damiao MC02 H7

该描述按 `projects/HERO-M/HERO-M.ioc` 和 `boards/DmMc02H7` 的现有实现建立。

已启用的基础资源包括 24 MHz 外部晶振、USART1 控制台、三路 FDCAN、SPI2（BMI088）和 SPI3（SD 卡）。SPI6 与 SPI3 共用 PC12，因此默认关闭 SPI6，等实板确认 LCD 与 SD 卡是否会同时使用后再选择其中一路。

USART2、USART3 的 RS485 方向脚已作为普通复用引脚列出；Zephyr 的 RS485 自动方向时序和实际收发器极性仍须实板验证。CAN 收发器待机脚、BMI088 中断脚、LCD 和电源控制脚也尚未纳入通用板描述。
