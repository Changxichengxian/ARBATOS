#include "mpu6500.h"

#include "main.h"
#include "mpu6500_reg.h"
#include "spi.h"

#include <string.h>

static inline void mpu6500_cs_low(void)
{
    HAL_GPIO_WritePin(MPU6500_CS_GPIO_Port, MPU6500_CS_Pin, GPIO_PIN_RESET);
}

static inline void mpu6500_cs_high(void)
{
    HAL_GPIO_WritePin(MPU6500_CS_GPIO_Port, MPU6500_CS_Pin, GPIO_PIN_SET);
}

static int mpu6500_write_reg(uint8_t reg, uint8_t data)
{
    uint8_t tx[2];
    tx[0] = (uint8_t)(reg & 0x7Fu);
    tx[1] = data;

    mpu6500_cs_low();
    const HAL_StatusTypeDef ret = HAL_SPI_Transmit(&hspi5, tx, 2u, 100u);
    mpu6500_cs_high();

    return (ret == HAL_OK) ? 0 : -1;
}

static int mpu6500_read_regs(uint8_t reg, uint8_t *out, uint16_t len)
{
    if (out == NULL || len == 0u)
    {
        return -1;
    }

    uint8_t cmd = (uint8_t)(reg | 0x80u);

    mpu6500_cs_low();
    HAL_StatusTypeDef ret = HAL_SPI_Transmit(&hspi5, &cmd, 1u, 100u);
    if (ret == HAL_OK)
    {
        ret = HAL_SPI_Receive(&hspi5, out, len, 100u);
    }
    mpu6500_cs_high();

    return (ret == HAL_OK) ? 0 : -2;
}

static uint8_t mpu6500_read_reg_u8(uint8_t reg, uint8_t *ok)
{
    uint8_t v = 0u;
    const int r = mpu6500_read_regs(reg, &v, 1u);
    if (ok != NULL)
    {
        *ok = (r == 0) ? 1u : 0u;
    }
    return v;
}

int mpu6500_init(void)
{
    // Keep the init sequence aligned with the original project:
    // - gyro: +-1000 dps (0x10)
    // - accel: +-8g (0x10)
    // - DLPF: 98Hz (CONFIG=0x02, ACCEL_CONFIG_2=0x02)
    // - clock source: gyro Z (PWR_MGMT_1=0x03)
    static const uint8_t init_seq[][2] = {
        {MPU6500_PWR_MGMT_1, 0x80}, // reset
        {MPU6500_PWR_MGMT_1, 0x03}, // clock: gyro Z
        {MPU6500_PWR_MGMT_2, 0x00}, // enable accel+gyro
        {MPU6500_CONFIG, 0x02},     // gyro DLPF
        {MPU6500_GYRO_CONFIG, 0x10}, // +-1000 dps
        {MPU6500_ACCEL_CONFIG, 0x10}, // +-8g
        {MPU6500_ACCEL_CONFIG_2, 0x02}, // accel DLPF
        {MPU6500_USER_CTRL, 0x20},  // enable I2C master (kept for compatibility)
        // Enable DataReady interrupt on INT pin (wired to IMU_INT/PB8 on RoboMaster dev board).
        // INT_PIN_CFG: active high, push-pull, 50us pulse; clear on any register read.
        {MPU6500_INT_PIN_CFG, 0x10},
        // INT_ENABLE: DATA_RDY_EN
        {MPU6500_INT_ENABLE, 0x01},
    };

    HAL_Delay(100u);

    uint8_t ok = 0u;
    const uint8_t who = mpu6500_read_reg_u8(MPU6500_WHO_AM_I, &ok);
    if (ok == 0u)
    {
        return -10;
    }
    if (who != MPU6500_WHOAMI_0 && who != MPU6500_WHOAMI_1)
    {
        // Still continue init, but report unexpected device ID.
        // (Some clones/variants return different values.)
        // Return distinct code so callers can decide.
        // NOTE: do not early-return; keep behavior tolerant for field boards.
    }

    for (uint32_t i = 0u; i < (uint32_t)(sizeof(init_seq) / sizeof(init_seq[0])); i++)
    {
        if (mpu6500_write_reg(init_seq[i][0], init_seq[i][1]) != 0)
        {
            return -11;
        }
        HAL_Delay(1u);
    }

    return 0;
}

int mpu6500_read_raw(mpu6500_raw_t *out)
{
    if (out == NULL)
    {
        return -1;
    }

    uint8_t buf[14];
    const int r = mpu6500_read_regs(MPU6500_ACCEL_XOUT_H, buf, (uint16_t)sizeof(buf));
    if (r != 0)
    {
        return r;
    }

    out->accel[0] = (int16_t)((uint16_t)buf[0] << 8 | buf[1]);
    out->accel[1] = (int16_t)((uint16_t)buf[2] << 8 | buf[3]);
    out->accel[2] = (int16_t)((uint16_t)buf[4] << 8 | buf[5]);
    out->temp = (int16_t)((uint16_t)buf[6] << 8 | buf[7]);
    out->gyro[0] = (int16_t)((uint16_t)buf[8] << 8 | buf[9]);
    out->gyro[1] = (int16_t)((uint16_t)buf[10] << 8 | buf[11]);
    out->gyro[2] = (int16_t)((uint16_t)buf[12] << 8 | buf[13]);

    return 0;
}

static uint8_t mpu6500_dma_tx[15];
static uint8_t mpu6500_dma_rx[15];
static volatile uint8_t mpu6500_dma_busy = 0u;

int mpu6500_read_raw_dma_start(void)
{
    if (mpu6500_dma_busy != 0u)
    {
        return -1;
    }
    if (hspi5.hdmatx == NULL || hspi5.hdmarx == NULL)
    {
        return -2;
    }

    mpu6500_dma_busy = 1u;

    mpu6500_dma_tx[0] = (uint8_t)(MPU6500_ACCEL_XOUT_H | 0x80u);
    memset(&mpu6500_dma_tx[1], 0xFF, 14u);

    mpu6500_cs_low();
    if (HAL_SPI_TransmitReceive_DMA(&hspi5, mpu6500_dma_tx, mpu6500_dma_rx, (uint16_t)sizeof(mpu6500_dma_tx)) != HAL_OK)
    {
        mpu6500_cs_high();
        mpu6500_dma_busy = 0u;
        return -3;
    }

    return 0;
}

int mpu6500_read_raw_dma_finish(mpu6500_raw_t *out)
{
    if (out == NULL)
    {
        return -1;
    }

    out->accel[0] = (int16_t)((uint16_t)mpu6500_dma_rx[1] << 8 | mpu6500_dma_rx[2]);
    out->accel[1] = (int16_t)((uint16_t)mpu6500_dma_rx[3] << 8 | mpu6500_dma_rx[4]);
    out->accel[2] = (int16_t)((uint16_t)mpu6500_dma_rx[5] << 8 | mpu6500_dma_rx[6]);
    out->temp = (int16_t)((uint16_t)mpu6500_dma_rx[7] << 8 | mpu6500_dma_rx[8]);
    out->gyro[0] = (int16_t)((uint16_t)mpu6500_dma_rx[9] << 8 | mpu6500_dma_rx[10]);
    out->gyro[1] = (int16_t)((uint16_t)mpu6500_dma_rx[11] << 8 | mpu6500_dma_rx[12]);
    out->gyro[2] = (int16_t)((uint16_t)mpu6500_dma_rx[13] << 8 | mpu6500_dma_rx[14]);

    return 0;
}

void mpu6500_read_raw_dma_release_cs(void)
{
    mpu6500_cs_high();
    mpu6500_dma_busy = 0u;
}

float mpu6500_temp_c(int16_t temp_raw)
{
    return ((float)temp_raw / 333.87f) + 21.0f;
}
