/* SPDX-License-Identifier: Apache-2.0 */
#ifndef BSP_BUZZER_C_H
#define BSP_BUZZER_C_H

/* 停止写入并等待 DMA 缓冲播完；实际停播由半缓冲完成中断确认。 */
void BuzzerPcmStreamFinish(void);

#endif
