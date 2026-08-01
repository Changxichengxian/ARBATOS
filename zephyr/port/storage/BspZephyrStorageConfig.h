/* Zephyr storage role selection for the ARBATOS compatibility layer. */
#ifndef ARB_ZEPHYR_STORAGE_CONFIG_H
#define ARB_ZEPHYR_STORAGE_CONFIG_H

#include <zephyr/devicetree.h>

/*
 * For a Zephyr disk driver, `sd-disk` points at the node which has
 * `disk-name`.  F427's existing SDMMC1 node uses the standard "SD" name.
 * A build supplied ARB_STORAGE_DISK_NAME takes precedence.
 */
#ifndef ARB_STORAGE_DISK_NAME
#if DT_HAS_ALIAS(sd_disk)
#define ARB_STORAGE_DISK_NAME DT_PROP(DT_ALIAS(sd_disk), disk_name)
#elif DT_NODE_HAS_STATUS(DT_NODELABEL(sdmmc1), okay)
#define ARB_STORAGE_DISK_NAME "SD"
#endif
#endif

#ifndef ARB_STORAGE_USE_DISK_ACCESS
#ifdef ARB_STORAGE_DISK_NAME
#define ARB_STORAGE_USE_DISK_ACCESS 1
#else
#define ARB_STORAGE_USE_DISK_ACCESS 0
#endif

#if ARB_STORAGE_USE_DISK_ACCESS && !defined(ARB_STORAGE_DISK_NAME)
#error "ARB_STORAGE_USE_DISK_ACCESS requires ARB_STORAGE_DISK_NAME"
#endif
#endif

/*
 * SPI cards retain the repository's SdSpi protocol implementation.  The
 * `sd-spi` alias must name its SPI child device node, not the SPI controller.
 */
#ifndef ARB_STORAGE_SPI_NODE
#if DT_HAS_ALIAS(sd_spi)
#define ARB_STORAGE_SPI_NODE DT_ALIAS(sd_spi)
#endif
#endif

#ifndef ARB_STORAGE_SPI_INIT_HZ
#define ARB_STORAGE_SPI_INIT_HZ 400000u
#endif
#ifndef ARB_STORAGE_SPI_FAST_HZ
#define ARB_STORAGE_SPI_FAST_HZ 8000000u
#endif

#endif
