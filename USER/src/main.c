#include "headfile.h"

/*
 * 系统频率，可查看board.h中的 FOSC 宏定义修改。
 * board.h文件中FOSC的值设置为0,则程序自动设置系统频率为33.1776MHZ
 * 在board_init中,已经将P54引脚设置为复位
 * 如果需要使用P54引脚,可以在board.c文件中的board_init()函数中删除SET_P54_RESRT即可
 */
void main(void)
{
	int state = 5;
	uint16 sum_value = 0;    
	uint16 value[7] = {0};   //调试用数组
	
	board_init();			 // 初始化寄存器,勿删除此句代码
	
	electromagnetic_init();  //初始化电磁传感器
	
	iic_init(IIC_2, IIC2_SCL_P25, IIC2_SDA_P24, 0);
	uart_init(UART_4, UART4_RX_P02, UART4_TX_P03, 115200, TIM_4);
	
	motor_init();
	encoder_init();
	
	imu963ra_init();
	oled_init();

	pid_init(&SpeedPID, 1.0f, 2.0f, 3.0f, 5000.0f, 6000.0f); //初始化速度PID
	pid_init(&TurnPID, 0.0f, 0.0f, 0.0f, 0.0f, 6000.0f);  //初始化位置PID
	
	lowpass_init(&leftSpeedFilt, 0.556);   //初始化低通滤波器
	lowpass_init(&rightSpeedFilt, 0.556);
	
	kalman_init(&imu693_kf, 0.98, 0.02, imu693kf_Q, imu693kf_R, 0.0);
	
	pit_timer_ms(TIM_1, 10);
	pit_timer_ms(TIM_2, 5);
	
    while(1)
	{
		key_task();         // 处理按键任务
		display_task();     // OLED显示任务
		
//		uart4_recv_task();  // 串口4接收任务
		
//		sprintf(g_TxData, "%f,%f\n",Gyro_Z,filtered_GyroZ);
//		uart_putstr(UART_4, g_TxData);
		
#if 0
		// 通过串口输出七电感数据
		sprintf(g_TxData, "%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d\n",
		 (uint16)normalized_data[SENSOR_HL], 
		 (uint16)normalized_data[SENSOR_VL], 
		 (uint16)normalized_data[SENSOR_HML], 
		 (uint16)normalized_data[SENSOR_HC],
		 (uint16)normalized_data[SENSOR_HMR], 
		 (uint16)normalized_data[SENSOR_VR], 
		 (uint16)normalized_data[SENSOR_HR], 
		  position,
		 (uint16)signal_strength_value,
		  track_type,
		  track_route,
		  track_route_status,
		  track_type_zj);
		 uart_putstr(UART_4, g_TxData);


		// 获取滤波后的ADC数据		
		mid_filter();      // 使用中位值滤波获取电感数据

		// 归一化电感数组·
		normalize_sensors();
		
		// 计算位置偏差
		position = calculate_position_improved();
		
		//检查电磁保护
		protection_flag = check_electromagnetic_protection();
#endif		
		
		/* 调试功能 */
#if 0
		//读取七电感ADC值（用于调试）
		value[0] = adc_once(ADC_HL,  ADC_10BIT);
		value[1] = adc_once(ADC_VL,  ADC_10BIT);
		value[2] = adc_once(ADC_HML, ADC_10BIT);
		value[3] = adc_once(ADC_HC,  ADC_10BIT); 
		value[4] = adc_once(ADC_HMR, ADC_10BIT);
		value[5] = adc_once(ADC_VR,  ADC_10BIT);
		value[6] = adc_once(ADC_HR,  ADC_10BIT);	

		// 计算所有电感值的总和
//		sum_value = (uint16)normalized_data[SENSOR_HL] + (uint16)normalized_data[SENSOR_VL] + 
//		            (uint16)normalized_data[SENSOR_HML] + (uint16)normalized_data[SENSOR_HC] + 
//		            (uint16)normalized_data[SENSOR_HMR] + (uint16)normalized_data[SENSOR_VR] + 
//		            (uint16)normalized_data[SENSOR_HR];

		 // 通过串口输出七电感原始数据
		  sprintf(g_TxData, "%d,%d,%d,%d,%d,%d,%d\n",
					value[0], 
					value[1], 
					value[2], 
					value[3], 
					value[4],
					value[5],
          value[6]);
		  uart_putstr(UART_4, g_TxData);

		  delay_ms(5);
#endif	
		
    }
}

