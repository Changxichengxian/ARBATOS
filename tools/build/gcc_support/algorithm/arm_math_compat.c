/*
 * GCC-side fallback for the tiny CMSIS-DSP surface used by ARBATOS.
 * Keil builds still use the ARMCC library recorded in the uVision projects.
 */

#include "arm_math.h"

#include <math.h>

__attribute__((weak)) float32_t arm_sin_f32(float32_t x)
{
    return sinf(x);
}

__attribute__((weak)) float32_t arm_cos_f32(float32_t x)
{
    return cosf(x);
}
