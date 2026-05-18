#ifndef MPU6500_H
#define MPU6500_H

#include "types.h"

typedef struct
{
    int16_t accel[3]; // raw counts
    int16_t gyro[3];  // raw counts
    int16_t temp;     // raw counts
} mpu6500_raw_t;

// Init sequence matches the "original" reference project (SPI5, 1kHz-ish, LPF on).
// Returns 0 on success, <0 on error.
int mpu6500_init(void);

// Read accel/temp/gyro in one burst from ACCEL_XOUT_H (14 bytes).
// Returns 0 on success, <0 on error.
int mpu6500_read_raw(mpu6500_raw_t *out);

// DMA burst read (ACCEL_XOUT_H..GYRO_ZOUT_L, 14 bytes).
// - start: asserts CS and kicks SPI5 DMA (15 bytes: 1 cmd + 14 data)
// - finish: parses the last DMA RX buffer into out (call after DMA complete)
// - release_cs: deasserts CS and clears busy flag (call from SPI DMA callbacks / timeouts)
int mpu6500_read_raw_dma_start(void);
int mpu6500_read_raw_dma_finish(mpu6500_raw_t *out);
void mpu6500_read_raw_dma_release_cs(void);

// Convert raw temperature to degC (MPU6500 datasheet: Temp_degC = raw/333.87 + 21).
float mpu6500_temp_c(int16_t temp_raw);

#endif
