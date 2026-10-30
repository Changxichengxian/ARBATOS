#ifndef IMU_ATTITUDE_H
#define IMU_ATTITUDE_H

#include <stdint.h>
#include "UserLib.h"

typedef struct {
    float accNormG, accRefG, normErrG, angleDeg, trust, kpGain;
    uint8_t healthy, rejected;
} ImuAccTrust;

typedef struct {
    mahony_imu_t mahony;
    second_order_filter_type_t accelFilter[3];
    float quat[4];
    /* 沿用业务接口顺序：yaw、roll、pitch；yaw 连续展开。 */
    float angle[3], filteredAcc[3], fusedAcc[3];
    float accRef, lastYaw, continuousYaw;
    uint32_t rejectUntil, gainUntil;
    ImuAccTrust acc;
    uint8_t initialized, mode, gainState, rejecting;
} ImuAttitude;

void ImuAttitudeInit(ImuAttitude *state);
void ImuAttitudeSetAccRef(ImuAttitude *state, float norm);
/* mode 沿用 imu_fusion_mode_e：0=Mahony，1=九轴；mag=NULL 时退回六轴。
 * 无有效初始重力或数据损坏时返回0，调用方不得发布新的有效姿态。 */
uint8_t ImuAttitudeUpdate(ImuAttitude *state, float dt, uint32_t now,
                          const float gyro[3], const float accel[3],
                          const float mag[3], uint8_t mode);

#endif
