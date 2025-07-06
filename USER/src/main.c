#include "headfile.h"

/*
 * ϵͳƵ�ʣ��ɲ鿴board.h�е� FOSC �궨���޸ġ�
 * board.h�ļ���FOSC��ֵ����Ϊ0,������Զ�����ϵͳƵ��Ϊ33.1776MHZ
 * ��board_init��,�Ѿ���P54��������Ϊ��λ
 * �����Ҫʹ��P54����,������board.c�ļ��е�board_init()������ɾ��SET_P54_RESRT����
 */
void main(void)
{
	int state = 5;
	uint16 sum_value = 0;    
	uint16 value[7] = {0};   //����������
	
	board_init();			 // ��ʼ���Ĵ���,��ɾ���˾����
	
	electromagnetic_init();  //��ʼ����Ŵ�����
	
	iic_init(IIC_2, IIC2_SCL_P25, IIC2_SDA_P24, 0);
	uart_init(UART_4, UART4_RX_P02, UART4_TX_P03, 115200, TIM_4);
	
	motor_init();
	encoder_init();
	
	imu963ra_init();
	oled_init();

	pid_init(&SpeedPID, 0.0f, 0.0f, 0.0f, 5000.0f, 6000.0f); //��ʼ���ٶ�PID
	pid_init(&TurnPID, 0.0f, 0.0f, 0.0f, 0.0f, 6000.0f);  //��ʼ��λ��PID
	
	lowpass_init(&leftSpeedFilt, 0.556);   //��ʼ����ͨ�˲���
	lowpass_init(&rightSpeedFilt, 0.556);
	
	kalman_init(&imu693_kf, 0.98, 0.02, imu693kf_Q, imu693kf_R, 0.0);
	
	pit_timer_ms(TIM_1, 10);
	pit_timer_ms(TIM_2, 5);
	
    while(1)
	{
		key_task();         // ����������
		display_task();     // OLED��ʾ����
		
		uart4_recv_task();  // ����4��������
		
//		sprintf(g_TxData, "%f,%f\n",Gyro_Z,filtered_GyroZ);
//		uart_putstr(UART_4, g_TxData);
		
#if 0
		// ͨ����������ߵ������
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
#endif

		// ��ȡ�˲����ADC����		
		mid_filter();      // ʹ����λֵ�˲���ȡ�������

		// ��һ��������顤
		normalize_sensors();
		
		// ����λ��ƫ��
		position = calculate_position_improved();
		
		//����ű���
		protection_flag = check_electromagnetic_protection();
		
		
		/* ���Թ��� */
#if 0
		//��ȡ�ߵ��ADCֵ�����ڵ��ԣ�
		value[0] = adc_once(ADC_HL,  ADC_10BIT);
		value[1] = adc_once(ADC_VL,  ADC_10BIT);
		value[2] = adc_once(ADC_HML, ADC_10BIT);
		value[3] = adc_once(ADC_HC,  ADC_10BIT); 
		value[4] = adc_once(ADC_HMR, ADC_10BIT);
		value[5] = adc_once(ADC_VR,  ADC_10BIT);
		value[6] = adc_once(ADC_HR,  ADC_10BIT);	

		// �������е��ֵ���ܺ�
//		sum_value = (uint16)normalized_data[SENSOR_HL] + (uint16)normalized_data[SENSOR_VL] + 
//		            (uint16)normalized_data[SENSOR_HML] + (uint16)normalized_data[SENSOR_HC] + 
//		            (uint16)normalized_data[SENSOR_HMR] + (uint16)normalized_data[SENSOR_VR] + 
//		            (uint16)normalized_data[SENSOR_HR];

		 // ͨ����������ߵ��ԭʼ����
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

