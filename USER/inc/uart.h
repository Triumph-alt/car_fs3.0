#ifndef __UART_H
#define __UART_H

#include "headfile.h"

#define UART_TX_LENGTH   200
#define UART_RX_LENGTH   200

extern uint8_t g_txbuffer[UART_TX_LENGTH];
extern uint8_t g_rxbuffer[UART_RX_LENGTH];
extern uint8_t g_rxpointer, g_rxdat;

void uart4_recv_task(void);
void uart4_interrupt_callback(void);

#endif