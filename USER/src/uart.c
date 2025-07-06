#include "uart.h"

//串口收发相关数据
uint8_t g_txbuffer[UART_TX_LENGTH] = {0};
uint8_t g_rxbuffer[UART_RX_LENGTH] = {0};
uint8_t g_rxpointer = 0, g_rxdat = 0;

void uart4_recv_task(void)
{
	if (g_rxpointer != 0)
	{
		int temp = g_rxpointer;
		delay_ms(4);
		
		if (temp == g_rxpointer)
		{
			uart4_interrupt_callback();
		}
	}
}

void uart4_interrupt_callback(void)
{
	if(g_rxpointer > 0)
	{
		
	}
	
	g_rxpointer = 0;
	memset(g_rxbuffer, 0, UART_RX_LENGTH);
	memset(g_txbuffer, 0, UART_TX_LENGTH);
}
