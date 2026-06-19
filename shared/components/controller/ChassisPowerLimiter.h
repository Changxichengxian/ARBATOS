#ifndef CHASSIS_POWER_LIMITER_H
#define CHASSIS_POWER_LIMITER_H

#include "Types.h"
#include "RobotConfig.h"

typedef struct
{
    fp32 power_limit;
    fp32 warning_power;
    fp32 warning_power_buffer;
    fp32 no_judge_total_current_limit;
    fp32 buffer_total_current_limit;
    fp32 power_total_current_limit;
} ChassisPowerLimiterConfig;

typedef struct
{
    fp32 ChassisPower;
    fp32 ChassisPowerBuffer;
    fp32 total_current;
    fp32 total_current_limit;
    fp32 current_scale;
} ChassisPowerLimiterResult;

extern fp32 ChassisPowerLimiterCalcTotalCurrentLimit(const ChassisPowerLimiterConfig *cfg,
                                                           fp32 ChassisPower,
                                                           fp32 ChassisPowerBuffer);

extern fp32 ChassisPowerLimiterCalcPowerBudget(const ChassisPowerLimiterConfig *cfg,
                                                    fp32 ChassisPower,
                                                    fp32 ChassisPowerBuffer,
                                                    uint8_t buffer_limit_active);

extern uint8_t ChassisPowerLimiterIsPowerModelReady(const motor_node_param_t motor_nodes[4]);

extern fp32 ChassisPowerLimiterScaleCurrents(fp32 currents[4],
                                                 fp32 total_current_limit,
                                                 fp32 *out_total_current);

extern fp32 ChassisPowerLimiterScaleCurrentsByPowerModel(fp32 currents[4],
                                                                const motor_node_param_t motor_nodes[4],
                                                                const int16_t wheel_rpm[4],
                                                                fp32 power_limit,
                                                                fp32 *out_total_power);

extern uint8_t ChassisPowerLimiterCalcMotorPower(const motor_node_param_t *node,
                                                      int16_t wheel_rpm,
                                                      fp32 current_cmd,
                                                      fp32 *out_power);

extern fp32 ChassisPowerLimiterLimitSingleCurrentByPowerModel(const motor_node_param_t *node,
                                                                      int16_t wheel_rpm,
                                                                      fp32 current_cmd,
                                                                      fp32 power_limit,
                                                                      fp32 *out_current,
                                                                      fp32 *out_power);

#endif
