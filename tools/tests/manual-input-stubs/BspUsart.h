#ifndef MANUAL_INPUT_TEST_BSP_USART_H
#define MANUAL_INPUT_TEST_BSP_USART_H

#include <stdint.h>

typedef enum
{
    BSP_AUX_LINK_RXEVENT_UNKNOWN = 0,
    BSP_AUX_LINK_RXEVENT_IDLE,
    BSP_AUX_LINK_RXEVENT_HT,
    BSP_AUX_LINK_RXEVENT_TC,
} BspAuxLinkRxEvent;

typedef void (*BspAuxLinkRxEventCb)(uint16_t size, BspAuxLinkRxEvent evt);
typedef void (*BspAuxLinkRxByteCb)(uint8_t byte);
typedef uint8_t (*BspAuxLinkErrorCb)(void);

void BspAuxLinkSetRxEventCb(BspAuxLinkRxEventCb callback);
void BspAuxLinkSetRxByteCb(BspAuxLinkRxByteCb callback);
void BspAuxLinkSetErrorCb(BspAuxLinkErrorCb callback);
uint32_t BspAuxLinkGetBaudrate(void);
uint8_t BspAuxLinkRxHasDma(void);
int BspAuxLinkRxToIdleDmaStart(uint8_t *buffer, uint16_t length);
int BspAuxLinkRxItStart(void);
void BspAuxLinkRxItStop(void);

#endif
