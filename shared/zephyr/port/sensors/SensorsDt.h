/* SPDX-License-Identifier: Apache-2.0 */
#ifndef ARBATOS_ZEPHYR_SENSORS_DT_H
#define ARBATOS_ZEPHYR_SENSORS_DT_H

#include <zephyr/devicetree.h>

#define ARBATOS_SENSORS_NODE DT_PATH(arbatos_sensors)

#if !DT_NODE_EXISTS(ARBATOS_SENSORS_NODE)
#error "缺少 /arbatos_sensors。请按 shared/README.md 的传感器说明为本板补齐传感器设备树属性。"
#endif

#endif
