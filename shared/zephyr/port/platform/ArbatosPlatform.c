/* SPDX-License-Identifier: Apache-2.0 */
#include "ArbatosPlatform.h"
#include <errno.h>
#include <zephyr/devicetree.h>
#include <zephyr/device.h>

#include "BspCan.h"
#include "BspCanZephyr.h"

extern int BspBuzzerPlatformInit(void);
extern void BspLedInit(void);

#define ARBATOS_PLATFORM_NODE DT_PATH(arbatos_platform)

int ArbatosPlatformInit(void)
{
#if !DT_NODE_EXISTS(ARBATOS_PLATFORM_NODE)
    return -ENODEV;
#else
    BspLedInit();
    int ret = BspBuzzerPlatformInit();
    if (ret != 0)
    {
        return ret;
    }

    /* 业务线程创建前完成 CAN 设备启动和接收过滤器安装，失败则不启动运行层。 */
    can_filter_init();
    return (BspCanZephyrReady() != 0u) ? 0 : -EIO;
#endif
}
