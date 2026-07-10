#ifndef ROBOT_LIFECYCLE_TEST_MANUAL_INPUT_SNAPSHOT_H
#define ROBOT_LIFECYCLE_TEST_MANUAL_INPUT_SNAPSHOT_H

#include <stdint.h>

#include "ControlInput.h"
#include "RobotConfig.h"

typedef struct ManualInputSnapshot
{
    ControlInputState control;
    ManualInputSemanticsConfig semantics;
    uint32_t semanticsSeq;
    uint8_t online;
} ManualInputSnapshot;

uint8_t ManualInputSnapshotRead(ManualInputSnapshot *out);

#endif
