/*
 * SPDX-FileCopyrightText: 2026 Chen Xuan <2811158416@qq.com>
 * SPDX-License-Identifier: Apache-2.0
 *
 * First published in this repository: 2026-04-06
 * Use of this file is governed by the LICENSE file in the repository root.
 */

#include "AuxPort.h"

#include "AuxTune.h"
#include "BspUsart.h"
#include "ElrsTask.h"
#include "ImageRemoteLink.h"

void AuxPortInit(void)
{
    const uint32_t baud = BspAuxLinkGetBaudrate();

    AuxPortStop();

    if (AuxPortIsElrsMode(baud))
    {
        BspAuxLinkSetRxEventCb(ElrsLinkOnRxEvent);
        BspAuxLinkSetRxByteCb(ElrsLinkOnItByte);
        BspAuxLinkSetErrorCb(ElrsLinkOnUartError);
        ElrsLinkRxStart();
    }
    else if (AuxPortIsTuneMode(baud))
    {
        AuxTuneRxStart();
    }
    else if (AuxPortIsImageMode(baud))
    {
        ImageRemoteLinkStart();
    }
}

void AuxPortPoll(void)
{
    const uint32_t baud = BspAuxLinkGetBaudrate();

    if (AuxPortIsTuneMode(baud))
    {
        AuxTunePoll();
        if (AuxPortIsTuneMode(BspAuxLinkGetBaudrate()))
        {
            AuxTuneTrySendTelem();
        }
    }
    else if (AuxPortIsImageMode(baud))
    {
        ImageRemoteLinkPoll();
    }
}

void AuxPortStop(void)
{
    ImageRemoteLinkStop();
    ElrsLinkStop();
    AuxTuneResetRx();

    BspAuxLinkSetRxEventCb(NULL);
    BspAuxLinkSetRxByteCb(NULL);
    BspAuxLinkSetErrorCb(NULL);
    BspAuxLinkRxItStop();
}

bool_t AuxPortApplyBaud(uint32_t baud)
{
    if (!AuxPortIsTuneMode(baud) &&
        !AuxPortIsElrsMode(baud) &&
        !AuxPortIsImageMode(baud))
    {
        return 0;
    }

    const uint32_t old_baud = BspAuxLinkGetBaudrate();
    if (old_baud == baud)
    {
        AuxPortInit();
        return 1;
    }

    AuxPortStop();
    if (BspAuxLinkSetBaudrate(baud) != 0)
    {
        (void)BspAuxLinkSetBaudrate(old_baud);
        AuxPortInit();
        return 0;
    }

    AuxPortInit();
    return 1;
}

uint8_t AuxPortIsElrsMode(uint32_t baud)
{
    return (baud == ELRS_LINK_BAUD) ? 1u : 0u;
}

uint8_t AuxPortIsImageMode(uint32_t baud)
{
    return (baud == IMAGE_REMOTE_LINK_BAUD) ? 1u : 0u;
}

uint8_t AuxPortIsTuneMode(uint32_t baud)
{
    return (baud == AUX_TUNE_BAUD) ? 1u : 0u;
}
