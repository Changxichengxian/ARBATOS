#ifndef SD_SPI_H
#define SD_SPI_H

#include <stdint.h>

typedef enum
{
    SD_SPI_CARD_NONE = 0,
    SD_SPI_CARD_MMC,
    SD_SPI_CARD_SDSC,
    SD_SPI_CARD_SDHC,
} sd_spi_card_type_e;

/**
  * @brief  Initialize SD card (SPI mode) on SPI2 + PB12(CS).
  * @retval 0 on success, non-zero on failure.
  */
int sd_spi_init(void);

/**
  * @brief  Check if SD card is initialized and ready.
  * @retval 1 ready, 0 not ready.
  */
int sd_spi_is_ready(void);

/**
  * @brief  Get detected SD card type.
  */
sd_spi_card_type_e sd_spi_get_card_type(void);

/**
  * @brief  Read 512-byte sectors.
  * @retval 0 on success, non-zero on failure.
  */
int sd_spi_read(uint8_t *buf, uint32_t sector, uint32_t count);

/**
  * @brief  Write 512-byte sectors.
  * @retval 0 on success, non-zero on failure.
  */
int sd_spi_write(const uint8_t *buf, uint32_t sector, uint32_t count);

/**
  * @brief  Get sector count (capacity / 512).
  * @retval 0 on success, non-zero on failure.
  */
int sd_spi_get_sector_count(uint32_t *out_sectors);

/**
  * @brief  Ensure the card is not busy (write finished).
  * @retval 0 on success, non-zero on failure.
  */
int sd_spi_sync(void);

#endif

