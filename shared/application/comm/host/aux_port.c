/*
 * SPDX-FileCopyrightText: 2026 Chen Xuan <2811158416@qq.com>
 * SPDX-License-Identifier: Apache-2.0
 *
 * First published in this repository: 2026-04-06
 * Use of this file is governed by the LICENSE file in the repository root.
 */

#include "aux_port.h"

#include "aux_tune.h"
#include "bsp_usart.h"
#include "elrs_task.h"
#include "image_remote_link.h"

void aux_port_init(void)
{
    const uint32_t baud = bsp_aux_link_get_baudrate();

    aux_port_stop();

    if (aux_port_is_elrs_mode(baud))
    {
        bsp_aux_link_set_rx_event_cb(elrs_link_on_rx_event);
        bsp_aux_link_set_rx_byte_cb(elrs_link_on_it_byte);
        bsp_aux_link_set_error_cb(elrs_link_on_uart_error);
        elrs_link_rx_start();
    }
    else if (aux_port_is_tune_mode(baud))
    {
        aux_tune_rx_start();
    }
    else if (aux_port_is_image_mode(baud))
    {
        image_remote_link_start();
    }
}

void aux_port_poll(void)
{
    const uint32_t baud = bsp_aux_link_get_baudrate();

    if (aux_port_is_tune_mode(baud))
    {
        aux_tune_poll();
        if (aux_port_is_tune_mode(bsp_aux_link_get_baudrate()))
        {
            aux_tune_try_send_telem();
        }
    }
    else if (aux_port_is_image_mode(baud))
    {
        image_remote_link_poll();
    }
}

void aux_port_stop(void)
{
    image_remote_link_stop();
    elrs_link_stop();
    aux_tune_reset_rx();

    bsp_aux_link_set_rx_event_cb(NULL);
    bsp_aux_link_set_rx_byte_cb(NULL);
    bsp_aux_link_set_error_cb(NULL);
    bsp_aux_link_rx_it_stop();
}

bool_t aux_port_apply_baud(uint32_t baud)
{
    if (!aux_port_is_tune_mode(baud) &&
        !aux_port_is_elrs_mode(baud) &&
        !aux_port_is_image_mode(baud))
    {
        return 0;
    }

    const uint32_t old_baud = bsp_aux_link_get_baudrate();
    if (old_baud == baud)
    {
        aux_port_init();
        return 1;
    }

    aux_port_stop();
    if (bsp_aux_link_set_baudrate(baud) != 0)
    {
        (void)bsp_aux_link_set_baudrate(old_baud);
        aux_port_init();
        return 0;
    }

    aux_port_init();
    return 1;
}

uint8_t aux_port_is_elrs_mode(uint32_t baud)
{
    return (baud == ELRS_LINK_BAUD) ? 1u : 0u;
}

uint8_t aux_port_is_image_mode(uint32_t baud)
{
    return (baud == IMAGE_REMOTE_LINK_BAUD) ? 1u : 0u;
}

uint8_t aux_port_is_tune_mode(uint32_t baud)
{
    return (baud == AUX_TUNE_BAUD) ? 1u : 0u;
}
