/* SPDX-License-Identifier: Apache-2.0 */
#include <assert.h>
#include <stdio.h>
#include "ServoOutputPolicy.h"

int main(void)
{
    ServoConfig absent = {0};
    assert(!ServoConfigValid(&absent));
    ServoConfig config = { .configured = 1u, .channels = {
        { .enabled = 1u, .port = 2u, .inputMode = 1u, .inputChannel = 3u,
          .minUs = 600u, .centerUs = 1400u, .maxUs = 2400u, .stepUs = 10u }
    }};
    ServoChannelConfig *channel = &config.channels[0];

    assert(ServoConfigValid(&config));
    assert(ServoPulseFromAxis(channel, 0) == 1400u);
    assert(ServoPulseFromAxis(channel, -660) == 600u);
    assert(ServoPulseFromAxis(channel, 660) == 2400u);
    assert(ServoPulseFromAxis(channel, 32767) == 2400u);
    assert(ServoPulseFromAxis(channel, -32768) == 600u);
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
    puts("PASS: servo limits, centre, inversion, ports and invalid config");
    return 0;
}
