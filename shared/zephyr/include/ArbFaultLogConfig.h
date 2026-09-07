/* SPDX-License-Identifier: Apache-2.0 */
#ifndef ARB_FAULT_LOG_CONFIG_H
#define ARB_FAULT_LOG_CONFIG_H

/* 仅强制包含到 Zephyr ARM fault.c/fatal.c，普通运行日志保持原配置。
 * 内核在判断可恢复异常之前也会输出 PR_EXC，必须在编译期消掉，
 * 否则日志持锁区内发生异常时，还没进入项目停机路径就会重入锁。 */
#if defined(CONFIG_LOG)
#undef CONFIG_KERNEL_LOG_LEVEL
#define CONFIG_KERNEL_LOG_LEVEL 0
#undef CONFIG_LOG_MAX_LEVEL
#define CONFIG_LOG_MAX_LEVEL 0
#undef CONFIG_LOG_OVERRIDE_LEVEL
#define CONFIG_LOG_OVERRIDE_LEVEL 0
#else
#undef CONFIG_PRINTK
#endif

#endif
