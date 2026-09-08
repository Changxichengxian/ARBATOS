/* SPDX-License-Identifier: Apache-2.0 */
#ifndef SERVO_CONFIG_H
#define SERVO_CONFIG_H

#include <stdint.h>

typedef struct
{
    uint8_t enabled;
    uint8_t port;         // P11 从 0 开始的物理 PWM 口
    uint8_t inputMode;    // 0=键盘步进，1=统一手动输入逻辑轴
    uint8_t inputChannel; // 统一输入轴 0..4，ELRS 通道先由输入映射转换
    uint8_t invert;
    uint16_t minUs;
    uint16_t centerUs;
    uint16_t maxUs;
    uint16_t stepUs;
} ServoChannelConfig;

typedef struct
{
    uint8_t configured;  // 仅 1 表示已配置；未配置时四路保持关闭
    ServoChannelConfig channels[4];
} ServoConfig;

#endif
