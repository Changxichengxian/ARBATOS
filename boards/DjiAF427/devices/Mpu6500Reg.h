#ifndef MPU6500_REG_H
#define MPU6500_REG_H

// Register map for MPU6500 (subset used by this project).
// Source: reference/original project in this repo.

#define MPU6500_SELF_TEST_XG (0x00)
#define MPU6500_SELF_TEST_YG (0x01)
#define MPU6500_SELF_TEST_ZG (0x02)
#define MPU6500_SELF_TEST_XA (0x0D)
#define MPU6500_SELF_TEST_YA (0x0E)
#define MPU6500_SELF_TEST_ZA (0x0F)

#define MPU6500_SMPLRT_DIV     (0x19)
#define MPU6500_CONFIG         (0x1A)
#define MPU6500_GYRO_CONFIG    (0x1B)
#define MPU6500_ACCEL_CONFIG   (0x1C)
#define MPU6500_ACCEL_CONFIG_2 (0x1D)

#define MPU6500_I2C_MST_CTRL       (0x24)
#define MPU6500_I2C_SLV0_ADDR      (0x25)
#define MPU6500_I2C_SLV0_REG       (0x26)
#define MPU6500_I2C_SLV0_CTRL      (0x27)
#define MPU6500_I2C_SLV1_ADDR      (0x28)
#define MPU6500_I2C_SLV1_REG       (0x29)
#define MPU6500_I2C_SLV1_CTRL      (0x2A)
#define MPU6500_I2C_SLV4_ADDR      (0x31)
#define MPU6500_I2C_SLV4_REG       (0x32)
#define MPU6500_I2C_SLV4_DO        (0x33)
#define MPU6500_I2C_SLV4_CTRL      (0x34)
#define MPU6500_I2C_SLV4_DI        (0x35)
#define MPU6500_INT_PIN_CFG        (0x37)
#define MPU6500_INT_ENABLE         (0x38)
#define MPU6500_INT_STATUS         (0x3A)

#define MPU6500_ACCEL_XOUT_H (0x3B)
#define MPU6500_TEMP_OUT_H  (0x41)
#define MPU6500_GYRO_XOUT_H (0x43)

#define MPU6500_USER_CTRL  (0x6A)
#define MPU6500_PWR_MGMT_1 (0x6B)
#define MPU6500_PWR_MGMT_2 (0x6C)

#define MPU6500_WHO_AM_I (0x75)

// Typical WHO_AM_I values for MPU6500 seen in the wild.
#define MPU6500_WHOAMI_0 (0x70)
#define MPU6500_WHOAMI_1 (0x71)

#endif
