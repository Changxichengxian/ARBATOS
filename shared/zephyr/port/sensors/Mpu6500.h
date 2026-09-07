/* SPDX-License-Identifier: Apache-2.0 */
#ifndef MPU6500_H
#define MPU6500_H

#include <stdint.h>
typedef struct { int16_t accel[3]; int16_t gyro[3]; int16_t temp; } mpu6500_raw_t;
int mpu6500_init(void);
int mpu6500_read_raw(mpu6500_raw_t *out);
int mpu6500_read_raw_dma_start(void);
int mpu6500_read_raw_dma_finish(mpu6500_raw_t *out);
void mpu6500_read_raw_dma_release_cs(void);
float mpu6500_temp_c(int16_t temp_raw);

#endif
