#ifndef CHASSIS_POWER_LIMITER_H
#define CHASSIS_POWER_LIMITER_H

#include "types.h"
#include "config.h"

typedef struct
{
    fp32 power_limit;
    fp32 warning_power;
    fp32 warning_power_buffer;
    fp32 no_judge_total_current_limit;
    fp32 buffer_total_current_limit;
    fp32 power_total_current_limit;
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

extern uint8_t chassis_power_limiter_calc_motor_power(const motor_node_param_t *node,
                                                      int16_t wheel_rpm,
                                                      fp32 current_cmd,
                                                      fp32 *out_power);

extern fp32 chassis_power_limiter_limit_single_current_by_power_model(const motor_node_param_t *node,
                                                                      int16_t wheel_rpm,
                                                                      fp32 current_cmd,
                                                                      fp32 power_limit,
                                                                      fp32 *out_current,
                                                                      fp32 *out_power);

#endif
