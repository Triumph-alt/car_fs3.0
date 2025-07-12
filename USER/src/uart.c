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
		if (strncmp(g_rxbuffer, "speed_kp", 8) == 0)
		{
			sscanf(g_rxbuffer, "speed_kp:%f", &SpeedPID.kp);
			
//			sprintf(g_txbuffer, "speed_kp:%f\n", SpeedPID.kp);
//			uart_putstr(UART_4, g_txbuffer);
		}
		else if (strncmp(g_rxbuffer, "speed_ki", 8) == 0)
		{
			sscanf(g_rxbuffer, "speed_ki:%f", &SpeedPID.ki);
			
//			sprintf(g_txbuffer, "speed_ki:%f\n", SpeedPID.ki);
//			uart_putstr(UART_4, g_txbuffer);
		}
		else if (strncmp(g_rxbuffer, "speed_kd", 8) == 0)
		{
			sscanf(g_rxbuffer, "speed_kd:%f", &SpeedPID.kd);
			
//			sprintf(g_txbuffer, "speed_kd:%f\n", SpeedPID.kd);
//			uart_putstr(UART_4, g_txbuffer);
		}
		else if (strncmp(g_rxbuffer, "turn_kp", 7) == 0)
		{
			sscanf(g_rxbuffer, "turn_kp:%f", &TurnPID.kp);
			
//			sprintf(g_txbuffer, "turn_kp:%f\n", TurnPID.kp);
//			uart_putstr(UART_4, g_txbuffer);
		}
		else if (strncmp(g_rxbuffer, "turn_ki", 7) == 0)
		{
			sscanf(g_rxbuffer, "turn_ki:%f", &TurnPID.ki);
			
//			sprintf(g_txbuffer, "turn_ki:%f\n", TurnPID.ki);
//			uart_putstr(UART_4, g_txbuffer);
		}
		else if (strncmp(g_rxbuffer, "turn_kd", 7) == 0)
		{
			sscanf(g_rxbuffer, "turn_kd:%f", &TurnPID.kd);
			
//			sprintf(g_txbuffer, "turn_kd:%f\n", TurnPID.kd);
//			uart_putstr(UART_4, g_txbuffer);
		}
		else if (strncmp(g_rxbuffer, "stop", 4) == 0)
		{			
			protection_flag = 1;
		}
		
	}
	
	g_rxpointer = 0;
	memset(g_rxbuffer, 0, UART_RX_LENGTH);
	memset(g_txbuffer, 0, UART_TX_LENGTH);
}
