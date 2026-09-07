/* SPDX-License-Identifier: Apache-2.0 */
#include <stdint.h>
/* Legacy absolute sector APIs cannot be safely mapped without a dedicated
 * Zephyr flash partition.  Reads remain direct only for memory-mapped flash. */
void flash_erase_address(uint32_t address, uint16_t len) { (void)address; (void)len; }
int8_t flash_write_single_address(uint32_t a,uint32_t *b,uint32_t n){(void)a;(void)b;(void)n;return -1;}
int8_t flash_write_muli_address(uint32_t a,uint32_t e,uint32_t *b,uint32_t n){(void)a;(void)e;(void)b;(void)n;return -1;}
void flash_read(uint32_t a,uint32_t *b,uint32_t n){if(!b)return; for(uint32_t i=0;i<n;i++) b[i]=*((const volatile uint32_t *)(uintptr_t)(a+i*4u));}
uint32_t ger_sector(uint32_t a){return a;} uint32_t get_next_flash_address(uint32_t a){return a;}
