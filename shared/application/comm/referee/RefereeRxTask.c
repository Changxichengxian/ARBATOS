/*
 * SPDX-FileCopyrightText: 2026 陈轩 <2811158416@qq.com>
 * SPDX-License-Identifier: Apache-2.0
 *
 * First published in this repository: 2026-04-06
 * Use of this file is governed by the LICENSE file in the repository root.
 */


#include "RefereeRxTask.h"

#include "FreeRTOS.h"
#include "task.h"

#include "BspUsart.h"
#include "DetectTask.h"

#include "Crc8Crc16.h"
#include "Fifo.h"
#include "RefereeProtocol.h"
#include "Referee.h"




/**
  * @brief          single byte upacked 
  * @param[in]      void
  * @retval         none
  */
/**
  * @brief          单字节解包
  * @param[in]      void
  * @retval         none
  */
static void RefereeUnpackFifoData(void);


static uint8_t RefereeRxChunkBuf[BSP_REFEREE_RX_BUF_LENGTH];

fifo_s_t RefereeFifo;
uint8_t RefereeFifoBuf[REFEREE_FIFO_BUF_LENGTH];
unpack_data_t RefereeUnpackObj;

/**
  * @brief          referee task
  * @param[in]      pvParameters: NULL
  * @retval         none
  */
/**
  * @brief          裁判系统任务
  * @param[in]      pvParameters: NULL
  * @retval         none
  */
void RefereeRxTask(void const * argument)
{
    (void)argument;

    init_referee_struct_data();
    fifo_s_init(&RefereeFifo, RefereeFifoBuf, REFEREE_FIFO_BUF_LENGTH);

    BspRefereeUartInit();
    BspRefereeRxAttachTask(xTaskGetCurrentTaskHandle());

    uint16_t rx_len = 0u;
    while (BspRefereeRxPop(RefereeRxChunkBuf, &rx_len))
    {
        fifo_s_puts(&RefereeFifo, (char *)RefereeRxChunkBuf, rx_len);
    }
    RefereeUnpackFifoData();

    while (1)
    {
        (void)ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        while (BspRefereeRxPop(RefereeRxChunkBuf, &rx_len))
        {
            fifo_s_puts(&RefereeFifo, (char *)RefereeRxChunkBuf, rx_len);
        }
        RefereeUnpackFifoData();
    }
}


/**
  * @brief          single byte upacked 
  * @param[in]      void
  * @retval         none
  */
/**
  * @brief          单字节解包
  * @param[in]      void
  * @retval         none
  */
static void RefereeUnpackFifoData(void)
{
  uint8_t byte = 0;
  uint8_t sof = HEADER_SOF;
  unpack_data_t *p_obj = &RefereeUnpackObj;

  while ( fifo_s_used(&RefereeFifo) )
  {
    byte = fifo_s_get(&RefereeFifo);
    switch(p_obj->unpack_step)
    {
      case STEP_HEADER_SOF:
      {
        if(byte == sof)
        {
          p_obj->unpack_step = STEP_LENGTH_LOW;
          p_obj->protocol_packet[p_obj->index++] = byte;
        }
        else
        {
          p_obj->index = 0;
        }
      }break;
      
      case STEP_LENGTH_LOW:
      {
        p_obj->data_len = byte;
        p_obj->protocol_packet[p_obj->index++] = byte;
        p_obj->unpack_step = STEP_LENGTH_HIGH;
      }break;
      
      case STEP_LENGTH_HIGH:
      {
        p_obj->data_len |= (byte << 8);
        p_obj->protocol_packet[p_obj->index++] = byte;

        if(p_obj->data_len < (REF_PROTOCOL_FRAME_MAX_SIZE - REF_HEADER_CRC_CMDID_LEN))
        {
          p_obj->unpack_step = STEP_FRAME_SEQ;
        }
        else
        {
          p_obj->unpack_step = STEP_HEADER_SOF;
          p_obj->index = 0;
        }
      }break;
      case STEP_FRAME_SEQ:
      {
        p_obj->protocol_packet[p_obj->index++] = byte;
        p_obj->unpack_step = STEP_HEADER_CRC8;
      }break;

      case STEP_HEADER_CRC8:
      {
        p_obj->protocol_packet[p_obj->index++] = byte;

        if (p_obj->index == REF_PROTOCOL_HEADER_SIZE)
        {
          if ( verify_CRC8_check_sum(p_obj->protocol_packet, REF_PROTOCOL_HEADER_SIZE) )
          {
            p_obj->unpack_step = STEP_DATA_CRC16;
          }
          else
          {
            p_obj->unpack_step = STEP_HEADER_SOF;
            p_obj->index = 0;
          }
        }
      }break;  
      
      case STEP_DATA_CRC16:
      {
        if (p_obj->index < (REF_HEADER_CRC_CMDID_LEN + p_obj->data_len))
        {
           p_obj->protocol_packet[p_obj->index++] = byte;  
        }
        if (p_obj->index >= (REF_HEADER_CRC_CMDID_LEN + p_obj->data_len))
        {
          p_obj->unpack_step = STEP_HEADER_SOF;
          p_obj->index = 0;

          if ( verify_CRC16_check_sum(p_obj->protocol_packet, REF_HEADER_CRC_CMDID_LEN + p_obj->data_len) )
          {
            // 未知的新命令也能证明物理链路正常；业务结构只由 RefereeDataSolve() 的已知命令分支更新。
            DetectHook(REFEREE_TOE);
            (void)RefereeDataSolve(p_obj->protocol_packet);
          }
        }
      }break;

      default:
      {
        p_obj->unpack_step = STEP_HEADER_SOF;
        p_obj->index = 0;
      }break;
    }
  }
}
