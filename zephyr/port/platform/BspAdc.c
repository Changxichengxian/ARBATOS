/* SPDX-License-Identifier: Apache-2.0 */
#include "Types.h"
#include <math.h>
/* ADC conversion is intentionally unavailable until board DTS supplies
 * channel, gain, reference and divider calibration.  Do not return a
 * plausible voltage/temperature. */
static uint32_t Fail;
void init_vrefint_reciprocal(void) {}
uint8_t BspAdcStart(void) { Fail++; return 0; }
uint8_t BspAdcIsStarted(void) { return 0; }
uint16_t BspAdcGetRaw(uint8_t index) { (void)index; return 0; }
fp32 BspAdcGetChannelVoltage(uint8_t index) { (void)index; return NAN; }
uint32_t BspAdcGetStartOkCount(void) { return 0; }
uint32_t BspAdcGetStartFailCount(void) { return Fail; }
fp32 get_temprate(void) { return NAN; }
fp32 get_battery_voltage(void) { return NAN; }
uint8_t get_hardware_version(void) { return 0xffu; }
