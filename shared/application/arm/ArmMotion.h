#pragma once

#include <stdint.h>

#include "ArmTask.h"
#include "ControlOutputPermit.h"

typedef struct
{
    uint32_t configuredMask;
    uint32_t activeMask;
    uint32_t blockingMask;
    uint32_t recoveryMask;
    uint32_t inhibitMask;
    uint32_t holdZeroMask;
    uint32_t inhibitFailCount;
    uint32_t releaseFailCount;
    uint8_t initialized;
    uint8_t reserved[3];
    uint16_t reason[ARM_JOINT_COUNT];
} ArmMotionFaultStatus;

void ArmMotionInit(void);
void ArmMotionStepManual(uint16_t key_mask,
                         const ControlOutputPermit *outputPermit);
const ArmMotorFeedback *ArmMotionGetFeedback(uint8_t index);
uint8_t ArmMotionGetFaultStatus(ArmMotionFaultStatus *out);
uint8_t ArmMotionProcessCanFeedback(uint8_t bus, uint16_t std_id, uint8_t dlc, const uint8_t data[8]);
const ArmJ0UnitreeState *ArmMotionGetJ0UnitreeState(void);
