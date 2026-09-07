/* SPDX-License-Identifier: Apache-2.0 */
#ifndef BSP_BUZZER_M_H
#define BSP_BUZZER_M_H

#include <stdint.h>

typedef struct
{
    uint32_t magic;
    uint32_t mode;
    uint32_t sampleHz;
    uint32_t actualHz;
    uint32_t carrierHz;
    uint32_t samples;
    uint32_t underruns;
    uint32_t queued;
    uint32_t starts;
    uint32_t stops;
    int32_t lastError;
} BuzzerPcmDiagnostics;

extern volatile BuzzerPcmDiagnostics BuzzerPcmDiag;
void BuzzerPcmStreamFinish(void);

#endif
