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
#if defined(__ZEPHYR__)
#include "RobotTargetConfig.h"
#endif

/* 旧 HAL 始终保留 AUX 复用；Zephyr 只在任务实际进入构建时保留该引用。 */
#if !defined(__ZEPHYR__)
#define AUX_PORT_LEGACY_ELRS 1
#elif !defined(ARB_UART_ROLES_EXPLICIT) && ROBOT_TASK_BUILD_ELRS_LINK
#define AUX_PORT_LEGACY_ELRS 1
#endif
#if defined(AUX_PORT_LEGACY_ELRS)
#include "ElrsTask.h"
#endif
#include "ImageRemoteLink.h"

void AuxPortInit(void)
{
    const uint32_t baud = BspAuxLinkGetBaudrate();

    AuxPortStop();

#if defined(AUX_PORT_LEGACY_ELRS)
    if (AuxPortIsElrsMode(baud))
    {
        BspAuxLinkSetRxEventCb(ElrsLinkOnRxEvent);
        BspAuxLinkSetRxByteCb(ElrsLinkOnItByte);
        BspAuxLinkSetErrorCb(ElrsLinkOnUartError);
        ElrsLinkRxStart();
    }
    else
#endif
    if (AuxPortIsTuneMode(baud))
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
#if defined(AUX_PORT_LEGACY_ELRS)
    ElrsLinkStop();
#endif
    AuxTuneResetRx();

    BspAuxLinkSetRxEventCb(NULL);
    BspAuxLinkSetRxByteCb(NULL);
    BspAuxLinkSetErrorCb(NULL);
    BspAuxLinkRxItStop();
}

bool_t AuxPortApplyBaud(uint32_t baud)
{
    if (!AuxPortIsTuneMode(baud) && !AuxPortIsImageMode(baud)
#if defined(AUX_PORT_LEGACY_ELRS)
        && !AuxPortIsElrsMode(baud)
#endif
    )
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
#if defined(AUX_PORT_LEGACY_ELRS)
    return (baud == ELRS_LINK_BAUD) ? 1u : 0u;
#else
    (void)baud;
    return 0u;
#endif
}

uint8_t AuxPortIsImageMode(uint32_t baud)
{
    return (baud == IMAGE_REMOTE_LINK_BAUD) ? 1u : 0u;
}

uint8_t AuxPortIsTuneMode(uint32_t baud)
{
    return (baud == AUX_TUNE_BAUD) ? 1u : 0u;
}
