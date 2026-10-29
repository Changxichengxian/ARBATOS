/* 用已知协议字节核对实物轮电机量程，防止控制限幅被当成编码满量程。 */
#include <assert.h>
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

static uint32_t xTaskGetTickCount(void) { return 1234u; }
#define portTICK_PERIOD_MS 1u
#include "CanMitMotorDriver.c"

static uint8_t captured[8];

int BspCanTx(uint8_t bus, uint16_t id, const uint8_t data[8], uint8_t dlc)
{
    assert(bus == 1u && id == 5u && dlc == 8u);
    memcpy(captured, data, 8u);
    return 0;
}

int BspCanTxTracked(uint8_t bus, uint16_t id, const uint8_t data[8], uint8_t dlc,
                   const BspCanTxTicket *ticket, uint8_t *tracked)
{
    assert(ticket != NULL && tracked != NULL);
    *tracked = 1u;
    return BspCanTx(bus, id, data, dlc);
}

int main(void)
{
    const MotorModelDbEntry *model = MotorModelDbGet(MOTOR_MODEL_DM_H3510_V10);
    assert(model != NULL);
    const CanMitMotorLimits *limits = &model->mit_limits;
    CanMitMotorCmd command = {.velocity = 140.0f, .torque = 0.45f};
    const uint8_t positive[8] = {0x80, 0x00, 0xbf, 0xf0, 0x00, 0x00, 0x0b, 0x99};
    const uint8_t negative[8] = {0x80, 0x00, 0x40, 0x00, 0x00, 0x00, 0x04, 0x66};
    assert(CanMitMotorSendCmd(1u, 5u, limits, &command) == 0);
    assert(memcmp(captured, positive, 8u) == 0);
    command.velocity = -140.0f;
    command.torque = -0.45f;
    BspCanTxTicket ticket = {.epoch = 1u, .seq = 1u};
    uint8_t tracked = 0u;
    assert(CanMitMotorSendCmdTracked(1u, 5u, limits, &command, &ticket, &tracked) == 0);
    assert(tracked == 1u && memcmp(captured, negative, 8u) == 0);

    const uint8_t feedback[8] = {0x15, 0x80, 0x00, 0xbf, 0xfb, 0xff, 25, 26};
    CanMitMotorFeedback state = {0};
    assert(CanMitMotorUpdateFeedback(5u, 5u, limits, 8u, feedback, &state));
    assert(state.state == 1u && state.motor_id == 5u && state.last_rx_tick == 1234u);
    assert(fabsf(state.velocity - 140.0f) < 0.1f);
    assert(fabsf(state.torque - 0.5f) < 0.001f);
    assert(state.temperature_valid && state.mos_temperature == 25u && state.coil_temperature == 26u);
    /* 短反馈没有温度，必须清除上次的有效标记。 */
    assert(CanMitMotorUpdateFeedback(5u, 5u, limits, 6u, feedback, &state));
    assert(!state.temperature_valid && state.mos_temperature == 0u && state.coil_temperature == 0u);
    assert(CanMitMotorUpdateFeedback(5u, 5u, limits, 5u, feedback + 1, &state));
    assert(!state.temperature_valid);
    /* 四个关节继续使用原有的 12.5 / 30 / 10 量程。 */
    const MotorModel joints[] = {MOTOR_MODEL_DM_J4310_2EC_V11, MOTOR_MODEL_DM_J4310_2EC_V12};
    for (unsigned i = 0; i < 2u; i++) {
        const MotorModelMitLimits *joint = &MotorModelDbGet(joints[i])->mit_limits;
        assert(joint->position_max == 12.5f && joint->velocity_max == 30.0f && joint->torque_max == 10.0f);
        /* 本地阻尼必须进入电机的 Kd 字段，速度目标保持零。 */
        const CanMitMotorCmd damped = {.kd = 0.4f, .torque = 0.2f};
        const uint8_t expected[8] = {0x80, 0x00, 0x80, 0x00, 0x00, 0x14, 0x88, 0x28};
        assert(CanMitMotorSendCmd(1u, 5u, joint, &damped) == 0);
        assert(memcmp(captured, expected, sizeof(expected)) == 0);
        /* 固定角度保持必须同时送出本地 Kp/Kd，不能退化为零力矩。 */
        const CanMitMotorCmd hold = {.kp = 8.0f, .kd = 0.4f};
        const uint8_t hold_bytes[8] = {0x80, 0x00, 0x80, 0x00, 0x42, 0x14, 0x88, 0x00};
        assert(CanMitMotorSendCmd(1u, 5u, joint, &hold) == 0);
        assert(memcmp(captured, hold_bytes, sizeof(hold_bytes)) == 0);
    }
    puts("MIT protocol: wheel bytes, feedback scale, joint ranges and local damping passed");
    return 0;
}
