/* SPDX-License-Identifier: Apache-2.0 */
#include <stdint.h>
/* Servo output stays inactive until a board-specific pwm array is declared. */
void ServoPwmSet(uint16_t pwm, uint8_t i) { (void)pwm; (void)i; }
