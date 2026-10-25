/* 运行真实功率控制入口；裁判输入和任务时钟使用固定场景。 */
#include <math.h>
#include <stdio.h>
#include <string.h>
#include "RobotConfig.h"
#include "ChassisPowerLimiter.h"

Config g_config;
static uint8_t testOffline;
static uint8_t testRobotId;
static fp32 testBuffer;
static fp32 testPowerLimit;

/* 仅替代任务容器，控制器、分配器和电机功率模型均使用产品实现。 */
#define CHASSIS_POWER_CONTROL_H
typedef struct
{
    int mode;
    fp32 vx_set, vy_set, wz_set;
    struct { fp32 out; } motor_speed_pid[4];
    struct {
        struct { int16_t speed_rpm; } measure;
        uint8_t measureValid;
        fp32 speed_set, speed;
    } motor_chassis[4];
} ChassisMove;
#define CHASSIS_VECTOR_RAW 3
#define REFEREE_H
#define RED_ENGINEER 2u
#define BLUE_ENGINEER 102u
static uint8_t get_robot_id(void) { return testRobotId; }
static fp32 get_chassis_power_limit(void) { return testPowerLimit; }
static void get_chassis_power_and_buffer(fp32 *power, fp32 *buffer)
{
    if (power != NULL) *power = 0.0f;
    *buffer = testBuffer;
}
#define DETECT_TASK_H
#define REFEREE_TOE 0u
static uint8_t DetectIsError(uint8_t item) { (void)item; return testOffline; }
#include "../../shared/application/chassis/ChassisPowerControl.c"

#define CHECK(condition, message) do { if (!(condition)) { \
    fprintf(stderr, "FAIL: %s\n", message); return 1; } } while (0)

static ChassisMove Reset(ChassisPowerLimitMode mode, uint8_t offline)
{
    ChassisMove move = {0};
    memset(&g_config, 0, sizeof(g_config));
    memset(&s_chassis_power_runtime, 0, sizeof(s_chassis_power_runtime));
    testOffline = offline;
    testRobotId = 1u;
    testBuffer = 60.0f;
    testPowerLimit = 80.0f;
    g_config.power = (power_config_t){.limitMode = mode, .power_limit = 80.0f,
        .warning_power = 40.0f, .warning_power_buffer = 50.0f,
        .no_judge_total_current_limit = 36000.0f,
        .buffer_total_current_limit = 16000.0f, .power_total_current_limit = 20000.0f};
    g_config.chassis.control_period_ms = 2u;
    for (uint8_t i = 0u; i < 4u; i++) {
        g_config.motor.chassis[i].model = MOTOR_MODEL_3508;
        g_config.motor.chassis[i].can_id = i + 1u;
        g_config.chassis.motor_dir[i] = (i % 2u == 0u) ? 1 : -1;
        move.motor_speed_pid[i].out = 16000.0f;
        move.motor_chassis[i].measureValid = 1u;
        move.motor_chassis[i].speed_set = 1.0f;
    }
    return move;
}

static fp32 Total(const ChassisMove *move)
{
    fp32 total = 0.0f;
    for (uint8_t i = 0u; i < 4u; i++) total += fabsf(move->motor_speed_pid[i].out);
    return total;
}

static fp32 Power(const ChassisMove *move)
{
    fp32 total = 0.0f;
    for (uint8_t i = 0u; i < 4u; i++) {
        fp32 power = 0.0f;
        (void)ChassisPowerLimiterCalcMotorPower(&g_config.motor.chassis[i],
            move->motor_chassis[i].measure.speed_rpm,
            move->motor_speed_pid[i].out * g_config.chassis.motor_dir[i], &power);
        total += power;
    }
    return total;
}

int main(void)
{
    ChassisMove move = Reset(ChassisPowerLimitAuto, 1u);
    ChassisPowerControl(&move, 0x0fu);
    CHECK(fabsf(Total(&move) - 36000.0f) < 0.1f, "默认离线仍使用36000总指令");

    move = Reset(ChassisPowerLimitTotalCommand, 0u);
    testPowerLimit = 10.0f;
    testBuffer = 0.0f;
    ChassisPowerControl(&move, 0x0fu);
    CHECK(fabsf(Total(&move) - 36000.0f) < 0.1f, "总指令模式不隐式切换模型");

    move = Reset(ChassisPowerLimitModel, 1u);
    g_config.power.power_limit = 20.0f;
    testBuffer = 0.0f;
    ChassisPowerControl(&move, 0x0fu);
    CHECK(Power(&move) <= 20.01f && Total(&move) > 0.0f, "模型离线按配置瓦数运行，不读取失效缓冲");

    move = Reset(ChassisPowerLimitModel, 0u);
    testPowerLimit = 30.0f;
    ChassisPowerControl(&move, 0x0fu);
    CHECK(Power(&move) <= 30.01f, "模型在线服从较小裁判功率");

    move = Reset(ChassisPowerLimitModel, 0u);
    g_config.power.power_limit = 20.0f;
    testPowerLimit = 100.0f;
    ChassisPowerControl(&move, 0x0fu);
    CHECK(Power(&move) <= 20.01f, "裁判上限不能放大手动配置上限");

    move = Reset(ChassisPowerLimitModelAndTotal, 1u);
    g_config.power.no_judge_total_current_limit = 10000.0f;
    g_config.power.power_limit = 20.0f;
    ChassisPowerControl(&move, 0x0fu);
    CHECK(Total(&move) <= 10000.1f && Power(&move) <= 20.01f, "组合模式同时满足两项约束");

    for (uint8_t sample = 0u; sample < 32u; sample++) {
        move = Reset(ChassisPowerLimitModelAndTotal, sample % 2u);
        g_config.power.no_judge_total_current_limit = 10000.0f;
        g_config.power.power_limit = 20.0f;
        for (uint8_t i = 0u; i < 4u; i++) {
            move.motor_speed_pid[i].out = ((sample >> i) & 1u) ? -12000.0f : 16000.0f;
            move.motor_chassis[i].measure.speed_rpm = (int16_t)((i % 2u) ? -1200 : 1200);
            move.motor_chassis[i].speed_set = (fp32)(i + 1u);
        }
        ChassisPowerControl(&move, 0x0fu);
        CHECK(Total(&move) <= 10000.1f && Power(&move) <= 20.01f,
            "不同轮速、驱动和制动组合仍同时满足两项约束");
    }

    move = Reset(ChassisPowerLimitModel, 1u);
    g_config.motor.chassis[0].model = MOTOR_MODEL_3510;
    ChassisPowerControl(&move, 0x0fu);
    CHECK(fabsf(Total(&move) - 36000.0f) < 0.1f && s_chassis_power_runtime.model_fallback_active,
        "模型缺失明确退回总指令");
    g_config.power.no_judge_total_current_limit = 0.0f;
    ChassisPowerControl(&move, 0x0fu);
    CHECK(Total(&move) == 0.0f, "模型与备用上限均不可用时不能无限输出");

    move = Reset(ChassisPowerLimitModelAndTotal, 1u);
    ChassisPowerControl(&move, 0x0eu);
    CHECK(move.motor_speed_pid[0].out == 0.0f, "故障轴不进入预算分配");
    puts("PASS: chassis power mode regression");
    return 0;
}
