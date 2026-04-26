#include "user_lib.h"
#include "arm_math.h"
#include <math.h>

//快速开方
fp32 invSqrt(fp32 num)
{
    fp32 halfnum = 0.5f * num;
    fp32 y = num;
    long i = *(long *)&y;
    i = 0x5f3759df - (i >> 1);
    y = *(fp32 *)&i;
    y = y * (1.5f - (halfnum * y * y));
    return y;
}

/**
  * @brief          斜波函数初始化
  * @author         RM
  * @param[in]      斜波函数结构体
  * @param[in]      间隔的时间，单位 s
  * @param[in]      最大值
  * @param[in]      最小值
  * @retval         返回空
  */
void ramp_init(ramp_function_source_t *ramp_source_type, fp32 frame_period, fp32 max, fp32 min)
{
    ramp_source_type->frame_period = frame_period;
    ramp_source_type->max_value = max;
    ramp_source_type->min_value = min;
    ramp_source_type->input = 0.0f;
    ramp_source_type->out = 0.0f;
}

/**
  * @brief          斜波函数计算，根据输入的值进行叠加， 输入单位为 /s 即一秒后增加输入的值
  * @author         RM
  * @param[in]      斜波函数结构体
  * @param[in]      输入值
  * @param[in]      滤波参数
  * @retval         返回空
  */
void ramp_calc(ramp_function_source_t *ramp_source_type, fp32 input)
{
    ramp_source_type->input = input;
    ramp_source_type->out += ramp_source_type->input * ramp_source_type->frame_period;
    if (ramp_source_type->out > ramp_source_type->max_value)
    {
        ramp_source_type->out = ramp_source_type->max_value;
    }
    else if (ramp_source_type->out < ramp_source_type->min_value)
    {
        ramp_source_type->out = ramp_source_type->min_value;
    }
}
/**
  * @brief          一阶低通滤波初始化
  * @author         RM
  * @param[in]      一阶低通滤波结构体
  * @param[in]      间隔的时间，单位 s
  * @param[in]      滤波参数
  * @retval         返回空
  */
void first_order_filter_init(first_order_filter_type_t *first_order_filter_type, fp32 frame_period, const fp32 num[1])
{
    first_order_filter_type->frame_period = frame_period;
    first_order_filter_type->num[0] = num[0];
    first_order_filter_type->input = 0.0f;
    first_order_filter_type->out = 0.0f;
}

/**
  * @brief          一阶低通滤波计算
  * @author         RM
  * @param[in]      一阶低通滤波结构体
  * @param[in]      间隔的时间，单位 s
  * @retval         返回空
  */
void first_order_filter_cali(first_order_filter_type_t *first_order_filter_type, fp32 input)
{
    first_order_filter_type->input = input;
    first_order_filter_type->out =
        first_order_filter_type->num[0] / (first_order_filter_type->num[0] + first_order_filter_type->frame_period) * first_order_filter_type->out + first_order_filter_type->frame_period / (first_order_filter_type->num[0] + first_order_filter_type->frame_period) * first_order_filter_type->input;
}

/**
  * @brief          二阶 IIR 滤波初始化
  * @param[in]      二阶滤波结构体
  * @param[in]      滤波系数 num[3]
  * @param[in]      初始输出
  * @retval         返回空
  */
void second_order_filter_init(second_order_filter_type_t *second_order_filter_type, const fp32 num[3], fp32 out)
{
    if (second_order_filter_type == NULL || num == NULL)
    {
        return;
    }

    second_order_filter_type->num[0] = num[0];
    second_order_filter_type->num[1] = num[1];
    second_order_filter_type->num[2] = num[2];
    second_order_filter_type->out = out;
    second_order_filter_type->out_last = out;
}

/**
  * @brief          二阶 IIR 滤波重置（清状态）
  * @param[in]      二阶滤波结构体
  * @param[in]      复位输出值
  * @retval         返回空
  */
void second_order_filter_reset(second_order_filter_type_t *second_order_filter_type, fp32 out)
{
    if (second_order_filter_type == NULL)
    {
        return;
    }
    second_order_filter_type->out = out;
    second_order_filter_type->out_last = out;
}

/**
  * @brief          二阶 IIR 滤波计算
  * @param[in]      二阶滤波结构体
  * @param[in]      输入值
  * @retval         返回滤波输出
  */
fp32 second_order_filter_cali(second_order_filter_type_t *second_order_filter_type, fp32 input)
{
    if (second_order_filter_type == NULL)
    {
        return input;
    }

    const fp32 out =
        second_order_filter_type->out * second_order_filter_type->num[0] +
        second_order_filter_type->out_last * second_order_filter_type->num[1] +
        input * second_order_filter_type->num[2];

    second_order_filter_type->out_last = second_order_filter_type->out;
    second_order_filter_type->out = out;
    return out;
}

/**
  * @brief          Mahony 姿态滤波初始化（6轴）
  * @param[in]      Mahony 结构体
  * @param[in]      Ki（积分系数，<=0 关闭积分）
  * @retval         返回空
  */
void mahony_imu_init(mahony_imu_t *mahony, fp32 ki)
{
    if (mahony == NULL)
    {
        return;
    }
    mahony->ki = ki;
    mahony_imu_reset(mahony);
}

/**
  * @brief          Mahony 状态重置（四元数 + 积分）
  * @param[in]      Mahony 结构体
  * @retval         返回空
  */
void mahony_imu_reset(mahony_imu_t *mahony)
{
    if (mahony == NULL)
    {
        return;
    }

    mahony->quat[0] = 1.0f;
    mahony->quat[1] = 0.0f;
    mahony->quat[2] = 0.0f;
    mahony->quat[3] = 0.0f;

    mahony->integral_fb[0] = 0.0f;
    mahony->integral_fb[1] = 0.0f;
    mahony->integral_fb[2] = 0.0f;
}

void mahony_imu_set_ki(mahony_imu_t *mahony, fp32 ki)
{
    if (mahony == NULL)
    {
        return;
    }
    mahony->ki = ki;
}

/**
  * @brief          Mahony 姿态滤波更新（6轴：gyro+acc）
  * @param[in]      Mahony 结构体（内部维护四元数/积分）
  * @param[in]      dt 单位 s
  * @param[in]      gyro 角速度 rad/s
  * @param[in]      acc 加速度 m/s^2（内部做归一化）
  * @param[in]      use_acc 是否使用 acc 修正
  * @param[in]      kp_gain P 增益（可动态调整）
  * @retval         返回空
  */
void mahony_imu_update(mahony_imu_t *mahony, float dt, const fp32 gyro[3], const fp32 acc[3], bool_t use_acc, float kp_gain)
{
    if (mahony == NULL || gyro == NULL || acc == NULL)
    {
        return;
    }

    float gx = gyro[0];
    float gy = gyro[1];
    float gz = gyro[2];
    float ax = acc[0];
    float ay = acc[1];
    float az = acc[2];

    float ex = 0.0f, ey = 0.0f, ez = 0.0f;

    if (use_acc)
    {
        const float recip_norm = 1.0f / sqrtf(ax * ax + ay * ay + az * az);
        ax *= recip_norm;
        ay *= recip_norm;
        az *= recip_norm;

        // 估计重力方向
        const float qw = mahony->quat[0];
        const float qx = mahony->quat[1];
        const float qy = mahony->quat[2];
        const float qz = mahony->quat[3];

        const float vx = 2.0f * (qx * qz - qw * qy);
        const float vy = 2.0f * (qw * qx + qy * qz);
        const float vz = qw * qw - qx * qx - qy * qy + qz * qz;

        // 误差 = 测量重力 × 估计重力
        ex = (ay * vz - az * vy);
        ey = (az * vx - ax * vz);
        ez = (ax * vy - ay * vx);
    }

    if (mahony->ki > 0.0f)
    {
        const float spin_rate = sqrtf(gx * gx + gy * gy + gz * gz);
        if (spin_rate < (20.0f * 0.01745329251994329577f))
        {
            mahony->integral_fb[0] += mahony->ki * ex * dt;
            mahony->integral_fb[1] += mahony->ki * ey * dt;
            mahony->integral_fb[2] += mahony->ki * ez * dt;
        }
    }
    else
    {
        mahony->integral_fb[0] = 0.0f;
        mahony->integral_fb[1] = 0.0f;
        mahony->integral_fb[2] = 0.0f;
    }

    gx += kp_gain * ex + mahony->integral_fb[0];
    gy += kp_gain * ey + mahony->integral_fb[1];
    gz += kp_gain * ez + mahony->integral_fb[2];

    gx *= 0.5f * dt;
    gy *= 0.5f * dt;
    gz *= 0.5f * dt;

    float qw = mahony->quat[0];
    float qx = mahony->quat[1];
    float qy = mahony->quat[2];
    float qz = mahony->quat[3];

    mahony->quat[0] += (-qx * gx - qy * gy - qz * gz);
    mahony->quat[1] += (qw * gx + qy * gz - qz * gy);
    mahony->quat[2] += (qw * gy - qx * gz + qz * gx);
    mahony->quat[3] += (qw * gz + qx * gy - qy * gx);

    const float recip_norm = 1.0f / sqrtf(mahony->quat[0] * mahony->quat[0] +
                                          mahony->quat[1] * mahony->quat[1] +
                                          mahony->quat[2] * mahony->quat[2] +
                                          mahony->quat[3] * mahony->quat[3]);
    mahony->quat[0] *= recip_norm;
    mahony->quat[1] *= recip_norm;
    mahony->quat[2] *= recip_norm;
    mahony->quat[3] *= recip_norm;
}

//绝对限制
void abs_limit(fp32 *num, fp32 Limit)
{
    if (*num > Limit)
    {
        *num = Limit;
    }
    else if (*num < -Limit)
    {
        *num = -Limit;
    }
}

//判断符号位
fp32 sign(fp32 value)
{
    if (value >= 0.0f)
    {
        return 1.0f;
    }
    else
    {
        return -1.0f;
    }
}

//浮点死区
fp32 fp32_deadline(fp32 Value, fp32 minValue, fp32 maxValue)
{
    if (Value < maxValue && Value > minValue)
    {
        Value = 0.0f;
    }
    return Value;
}

//int26死区
int16_t int16_deadline(int16_t Value, int16_t minValue, int16_t maxValue)
{
    if (Value < maxValue && Value > minValue)
    {
        Value = 0;
    }
    return Value;
}

//限幅函数
fp32 fp32_constrain(fp32 Value, fp32 minValue, fp32 maxValue)
{
    if (Value < minValue)
        return minValue;
    else if (Value > maxValue)
        return maxValue;
    else
        return Value;
}

//限幅函数
int16_t int16_constrain(int16_t Value, int16_t minValue, int16_t maxValue)
{
    if (Value < minValue)
        return minValue;
    else if (Value > maxValue)
        return maxValue;
    else
        return Value;
}

//循环限幅函数
fp32 loop_fp32_constrain(fp32 Input, fp32 minValue, fp32 maxValue)
{
    if (maxValue < minValue)
    {
        return Input;
    }

    if (Input > maxValue)
    {
        fp32 len = maxValue - minValue;
        while (Input > maxValue)
        {
            Input -= len;
        }
    }
    else if (Input < minValue)
    {
        fp32 len = maxValue - minValue;
        while (Input < minValue)
        {
            Input += len;
        }
    }
    return Input;
}

//弧度格式化为-PI~PI

//角度格式化为-180~180
fp32 theta_format(fp32 Ang)
{
    return loop_fp32_constrain(Ang, -180.0f, 180.0f);
}
