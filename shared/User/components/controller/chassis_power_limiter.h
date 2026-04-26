#ifndef CHASSIS_POWER_LIMITER_H
#define CHASSIS_POWER_LIMITER_H

#include "struct_typedef.h"
#include "app_config.h"

typedef struct
{
    fp32 power_limit;                  // 功率上限
    fp32 warning_power;                // 告警功率阈值
    fp32 warning_power_buffer;         // 告警缓冲阈值
    fp32 no_judge_total_current_limit; // 无裁判时电流上限
    fp32 buffer_total_current_limit;   // 缓冲段电流上限
    fp32 power_total_current_limit;    // 正常段电流上限
} chassis_power_limiter_config_t;

typedef struct
{
    fp32 chassis_power;
    fp32 chassis_power_buffer;
    fp32 total_current;
    fp32 total_current_limit;
    fp32 current_scale;
} chassis_power_limiter_result_t;

extern fp32 chassis_power_limiter_calc_total_current_limit(const chassis_power_limiter_config_t *cfg,
                                                           fp32 chassis_power,
                                                           fp32 chassis_power_buffer);

extern fp32 chassis_power_limiter_calc_power_budget(const chassis_power_limiter_config_t *cfg,
                                                    fp32 chassis_power,
                                                    fp32 chassis_power_buffer,
                                                    uint8_t buffer_limit_active);

extern uint8_t chassis_power_limiter_is_power_model_ready(const motor_node_param_t motor_nodes[4]);

extern fp32 chassis_power_limiter_scale_currents(fp32 currents[4],
                                                 fp32 total_current_limit,
                                                 fp32 *out_total_current);

extern fp32 chassis_power_limiter_scale_currents_by_power_model(fp32 currents[4],
                                                                const motor_node_param_t motor_nodes[4],
                                                                const int16_t wheel_rpm[4],
                                                                fp32 power_limit,
                                                                fp32 *out_total_power);

#endif
