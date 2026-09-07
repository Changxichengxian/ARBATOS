/* SPDX-License-Identifier: Apache-2.0 */
#include <zephyr/devicetree.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/util.h>

/* 独立于车型，直接核对编译后的板级描述。此工程不启动电机任务。 */
BUILD_ASSERT(DT_NODE_HAS_STATUS(DT_ALIAS(can_primary), okay));
BUILD_ASSERT(DT_NODE_HAS_STATUS(DT_ALIAS(uart_rc), okay));
BUILD_ASSERT(DT_NODE_HAS_PROP(DT_PATH(arbatos_platform), key_gpios));

#if defined(CONFIG_BOARD_DJI_A_F427)
BUILD_ASSERT(DT_PROP(DT_NODELABEL(clk_hse), clock_frequency) == 12000000);
BUILD_ASSERT(!DT_NODE_HAS_STATUS(DT_NODELABEL(clk_lse), okay));
BUILD_ASSERT(!DT_HAS_CHOSEN(zephyr_console));
BUILD_ASSERT(DT_SAME_NODE(DT_ALIAS(uart_rc), DT_NODELABEL(usart1)));
BUILD_ASSERT(DT_PROP_LEN(DT_NODELABEL(usart1), pinctrl_0) == 1);
BUILD_ASSERT(DT_GPIO_PIN(DT_ALIAS(sw0), gpios) == 2);
BUILD_ASSERT(DT_GPIO_FLAGS(DT_ALIAS(sw0), gpios) == (GPIO_ACTIVE_HIGH | GPIO_PULL_DOWN));
BUILD_ASSERT(DT_SAME_NODE(DT_GPIO_CTLR(DT_ALIAS(led0), gpios), DT_NODELABEL(gpiof)));
BUILD_ASSERT(DT_GPIO_PIN(DT_ALIAS(led0), gpios) == 14);
BUILD_ASSERT(DT_GPIO_FLAGS(DT_ALIAS(led0), gpios) == GPIO_ACTIVE_LOW);
BUILD_ASSERT(DT_SAME_NODE(DT_GPIO_CTLR(DT_NODELABEL(sdmmc1), cd_gpios), DT_NODELABEL(gpioe)));
BUILD_ASSERT(DT_GPIO_PIN(DT_NODELABEL(sdmmc1), cd_gpios) == 15);
BUILD_ASSERT(DT_GPIO_FLAGS(DT_NODELABEL(sdmmc1), cd_gpios) == (GPIO_ACTIVE_LOW | GPIO_PULL_UP));
BUILD_ASSERT(DT_SAME_NODE(DT_PHANDLE(DT_PATH(arbatos_sensors), mpu6500_spi), DT_NODELABEL(spi5)));
#elif defined(CONFIG_BOARD_DJI_C_F407)
BUILD_ASSERT(DT_SAME_NODE(DT_PHANDLE(DT_PATH(arbatos_sensors), bmi088_spi), DT_NODELABEL(spi1)));
#else
#error "This check currently covers the restored A and C boards"
#endif

int main(void)
{
    return 0;
}
