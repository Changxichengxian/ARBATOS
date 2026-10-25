/* SPDX-License-Identifier: Apache-2.0 */
#include <assert.h>
#include <stdio.h>
#include "ServoOutputPolicy.h"

static uint8_t Authorized = 1u;
uint8_t LowCmdOutputSnapshotAuthorized(MotorId id,
                                       const MotorCmd *cmd,
                                       const ControlOutputStamp *owner)
{
    (void)id;
    (void)cmd;
    (void)owner;
    return Authorized;
}

int main(void)
{
    ServoConfig absent = {0};
    assert(MotorCount == 22 && LOWCMD_SERVO_FIRST == Motor18 && LOWCMD_SERVO_COUNT == 4u);
    assert(!ServoConfigValid(&absent));
    ServoConfig config = { .configured = 1u, .channels = {
        { .enabled = 1u, .port = 2u, .inputMode = 1u, .inputChannel = 3u,
          .minUs = 600u, .centerUs = 1400u, .maxUs = 2400u, .stepUs = 10u }
    }};
    ServoChannelConfig *channel = &config.channels[0];
    MotorCmd cmd = {0};
    ControlOutputStamp owner = {0};
    uint16_t pulse = 0u;

    assert(ServoConfigValid(&config));
    assert(ServoPulseFromAxis(channel, 0) == 1400u);
    assert(ServoPulseFromAxis(channel, -660) == 600u);
    assert(ServoPulseFromAxis(channel, 660) == 2400u);
    assert(ServoPulseFromAxis(channel, 32767) == 2400u);
    assert(ServoPulseFromAxis(channel, -32768) == 600u);
    assert(ServoPulseFromPosition(channel, ServoPositionFromAxis(-660)) == 600u);
    assert(ServoPulseFromPosition(channel, ServoPositionFromAxis(660)) == 2400u);
    assert(ServoPulseFromPosition(channel, ServoPositionStep(channel, 0.0f, 1)) == 1410u);

    cmd.active = 1u;
    cmd.mode = (uint8_t)MotorModePosVel;
    cmd.timeoutMs = 30u;
    cmd.tick = 100u;
    cmd.q = 0.5f;
    owner.valid = 1u;
    assert(ServoCommandPulse(channel, Motor0, &cmd, &owner, 130u, 0u, &pulse));
    assert(pulse == 1900u);
    cmd.q = 2.0f;
    assert(ServoCommandPulse(channel, Motor0, &cmd, &owner, 130u, 0u, &pulse));
    assert(pulse == 2400u);
    cmd.q = 0.5f;
    assert(!ServoCommandPulse(channel, Motor0, &cmd, &owner, 131u, 0u, &pulse));
    assert(pulse == 0u);
    assert(!ServoCommandPulse(channel, Motor0, &cmd, &owner, 110u, 1u, &pulse));
    Authorized = 0u;
    assert(!ServoCommandPulse(channel, Motor0, &cmd, &owner, 110u, 0u, &pulse));
    assert(pulse == 0u);
    Authorized = 1u;
    cmd.mode = (uint8_t)MotorModeDisable;
    assert(!ServoCommandPulse(channel, Motor0, &cmd, &owner, 110u, 0u, &pulse));
    cmd.mode = (uint8_t)MotorModePosVel;
    channel->invert = 1u;
    assert(ServoPulseFromAxis(channel, -660) == 2400u);
    assert(ServoPulseStep(channel, 1400u, 1) == 1390u);
    assert(ServoPulseStep(channel, 600u, 1) == 600u);
    config.channels[1] = *channel;
    assert(!ServoConfigValid(&config));
    config.channels[1].enabled = 0u;
    assert(ServoConfigValid(&config));
    channel->port = 4u;
    assert(!ServoConfigValid(&config));
    channel->port = 2u;
    channel->centerUs = channel->maxUs;
    assert(!ServoConfigValid(&config));
    assert(ServoPulseFromAxis(channel, 0) == 0u);
    assert(ServoPulseStep(NULL, 1500u, 1) == 0u);
    puts("PASS: servo semantic position, timeout, lock, revoke and config bounds");
    return 0;
}
