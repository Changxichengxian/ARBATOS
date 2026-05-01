/*
 * SPDX-FileCopyrightText: 2026 Chen Yi <2811158416@qq.com>
 * SPDX-License-Identifier: PolyForm-Noncommercial-1.0.0
 * Required Notice: Copyright 2026 Chen Yi <2811158416@qq.com>
 *
 * First published in this repository: 2026-04-11
 * Use of this file is governed by the LICENSE file in the repository root.
 */

#include "watch.h"

/*
 * This carrier build does not link the full watch/diagnostic pipeline.
 * Keep shared task hooks linkable with local no-op implementations.
 */

void watch_init(void) {}
void watch_update(void) {}
void watch_diag_set_boot_stage(watch_boot_stage_e stage) { (void)stage; }
void watch_diag_mark_error_handler(uint32_t tick_ms, uint32_t ipsr)
{
    (void)tick_ms;
    (void)ipsr;
}

void watch_diag_set_error_args(uint32_t arg0, uint32_t arg1)
{
    (void)arg0;
    (void)arg1;
}

void watch_task_beat(watch_task_id_e task_id) { (void)task_id; }
void watch_task_wait(watch_task_id_e task_id) { (void)task_id; }
void watch_task_timeout(watch_task_id_e task_id) { (void)task_id; }
void watch_task_error(watch_task_id_e task_id) { (void)task_id; }
void watch_irq_hit(watch_irq_id_e irq_id) { (void)irq_id; }

const watch_block_desc_t *watch_get_block_table(uint32_t *count)
{
    if (count != 0)
    {
        *count = 0u;
    }
    return 0;
}

const watch_block_desc_t *watch_find_block(watch_block_id_e id)
{
    (void)id;
    return 0;
}

uint8_t watch_block_is_active(watch_block_id_e id)
{
    (void)id;
    return 0u;
}
