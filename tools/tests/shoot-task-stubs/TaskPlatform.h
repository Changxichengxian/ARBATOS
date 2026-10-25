/* 独立发射任务的主机调度边界；真实任务循环和 ShootCtrl 接口保持原样。 */
#ifndef SHOOT_TASK_TEST_PLATFORM_H
#define SHOOT_TASK_TEST_PLATFORM_H
#include <stddef.h>
#include <stdint.h>
#include "ShootCtrl.h"
#define ROBOT_TASK_BUILD_SHOOT_RM 1
#define INPUT_SW_GIMBAL_MODE 0u
typedef uint32_t TickType_t;
#define pdMS_TO_TICKS(value) (value)
typedef enum { RtProfShootLoop } RtProfId;
typedef enum { WATCH_TASK_SHOOT_CONTROL } WatchTaskId;
typedef struct ManualInputSnapshot
{
    struct { uint16_t sw[1]; } control;
    struct { uint8_t GimbalSafePos; } semantics;
    uint32_t sourceAgeMs;
    uint32_t sourceTimeoutMs;
    uint8_t online;
    uint8_t dataValid;
} ManualInputSnapshot;
TickType_t xTaskGetTickCount(void);
void vTaskDelayUntil(TickType_t *lastWake, TickType_t ticks);
void vTaskDelay(TickType_t ticks);
uint64_t RtProfBegin(void);
void RtProfEnd(RtProfId id, uint64_t started);
void WatchTaskBeat(WatchTaskId id);
uint16_t RobotProfileShootControlPeriodMs(void);
uint32_t BspTimeGetTickMs(void);
uint8_t ManualInputSnapshotRead(ManualInputSnapshot *out);
uint8_t ControlInputSwitchIsPos(uint16_t raw, uint8_t position);
uint8_t RobotLifecycleOutputAllowed(void);
#endif
