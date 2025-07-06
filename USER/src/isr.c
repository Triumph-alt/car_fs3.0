///*********************************************************************************************************************
// * COPYRIGHT NOTICE
// * Copyright (c) 2020,��ɿƼ�
// * All rights reserved.
// * ��������QQȺ��һȺ��179029047(����)  ��Ⱥ��244861897(����)  ��Ⱥ��824575535
// *
// * �����������ݰ�Ȩ������ɿƼ����У�δ��������������ҵ��;��
// * ��ӭ��λʹ�ò������������޸�����ʱ���뱣����ɿƼ��İ�Ȩ������
// *
// * @file       		isr
// * @company	   		�ɶ���ɿƼ����޹�˾
// * @author     		��ɿƼ�(QQ790875685)
// * @version    		�鿴doc��version�ļ� �汾˵��
// * @Software 			MDK FOR C251 V5.60
// * @Target core		STC32G12K128
// * @Taobao   			https://seekfree.taobao.com/
// * @date       		2020-4-14
// ********************************************************************************************************************/
#include "headfile.h"

int g_encoder_average = 0;                       //���ұ�������ƽ��ֵ
float Gyro_Z = 0, filtered_GyroZ = 0;            // �����ǽ��ٶȵ�ԭʼֵ�Ϳ������˲�֮���ֵ
int32_t g_DutyLeft = 0, g_DutyRight = 0;         // �������Ҫ�������PWMֵ

//pid������ر���
float speed_pid = 0, turn_pid = 0;               //�ٶȻ���ת��pid��ֵ
int g_speedpoint = 80;
int g_leftpoint = 0, g_rightpoint = 0;           //�����ֵ�Ŀ���ٶ�

// ������������ر���
uint8_t beep_flag = 0;                           // ������������־��1��ʾ����
uint16_t beep_count = 0;                         // ��������ʱ������
uint8_t track_ten_cnt = 0;                       //���뻷�ظ��ж���ʱ��

//UART1�ж�
void UART1_Isr() interrupt 4
{
//  uint8 res;
//	static uint8 dwon_count;
	
    if(UART1_GET_TX_FLAG)
    {
        UART1_CLEAR_TX_FLAG;
        busy[1] = 0;
    }
	
    if(UART1_GET_RX_FLAG)
    {
        UART1_CLEAR_RX_FLAG;
//        res = SBUF;
//        //�����Զ�����
//        if(res == 0x7F)
//        {
//            if(dwon_count++ > 20)
//                IAP_CONTR = 0x60;
//        }
//        else
//        {
//            dwon_count = 0;
//        }
    }
}

//UART2�ж�
void UART2_Isr() interrupt 8
{
    if(UART2_GET_TX_FLAG)
	{
        UART2_CLEAR_TX_FLAG;
		busy[2] = 0;
	}
    if(UART2_GET_RX_FLAG)
	{
        UART2_CLEAR_RX_FLAG;
		
		//�������ݼĴ���Ϊ��S2BUF

	}
}


//UART3�ж�
void UART3_Isr() interrupt 17
{
    if(UART3_GET_TX_FLAG)
	{
        UART3_CLEAR_TX_FLAG;
		busy[3] = 0;
	}
    if(UART3_GET_RX_FLAG)
	{
        UART3_CLEAR_RX_FLAG;
		
		//�������ݼĴ���Ϊ��S3BUF

	}
}


//UART4�ж�
void UART4_Isr() interrupt 18
{
    if(UART4_GET_TX_FLAG)
	{
        UART4_CLEAR_TX_FLAG;
		busy[4] = 0;
	}
    if(UART4_GET_RX_FLAG)
	{
        UART4_CLEAR_RX_FLAG;
		
		//�������ݼĴ���Ϊ��S4BUF;
		g_rxdat = S4BUF;
		g_rxbuffer[g_rxpointer++] = g_rxdat;
	}
}

void INT0_Isr() interrupt 0
{
	
}


void INT1_Isr() interrupt 2
{

}


void INT2_Isr() interrupt 10
{
	INT2_CLEAR_FLAG;  //����жϱ�־
}


void INT3_Isr() interrupt 11
{
	INT3_CLEAR_FLAG;  //����жϱ�־
}

void INT4_Isr() interrupt 16
{
	INT4_CLEAR_FLAG;  //����жϱ�־
}

void TM0_Isr() interrupt 1
{

}


void TM1_Isr() interrupt 3
{
	int i = 0;
	
	key[0].state = P72;
	key[1].state = P71;
	key[2].state = P70;
	key[3].state = P73;
	
	for (i = 0; i < 4; i++)
	{
		switch (key[i].step)
		{
			case 0:
			{
				if (key[i].state == 0)
				{
					key[i].step = 1;
				}
			}
			break;
			
			case 1:
			{
				if (key[i].state == 0)
				{
					key[i].step = 2;
					key[i].flag = 1;
				}
				else
				{
					key[i].step = 0;
				}
			}
			break;
			
			case 2:
			{
				if (key[i].state == 1)
				{
					key[i].step = 0;
				}
			}
			break;
		}
	}

	/* ����������ͱ仯�����Ʒ����� */
    if (track_type != track_type_last)
    {
        // �������ͷ����仯������������
        beep_flag = 1;
        beep_count = 0;  // ���ü�����
        P26 = 0;         // �򿪷�����
        
        // ������һ����������
        track_type_last = track_type;
    }
    
    /* ��������ʱ���� */
    if (beep_flag)
    {
        beep_count++;
        // 10ms * 20 = 200ms
        if (beep_count >= 10)
        {
            beep_count = 0;
            beep_flag = 0;
            P26 = 1;  // �رշ�����
        }
    }

	/* ����ʮ��Բ����ʱ�ж� */
	if (ten_change_flag == 1)
	{
		track_ten_cnt++;
		if (track_ten_cnt >= 150)
		{
			track_ten_flag = 1;
			track_ten_cnt = 0;
			ten_change_flag = 0;
		}
	}
}


void TM2_Isr() interrupt 12
{
	TIM2_CLEAR_FLAG;  //����жϱ�־
	
	/* ������ȡ�������������ֵ */
	EncoderL.encoder_original = get_left_encoder();
	EncoderR.encoder_original = get_right_encoder();

	/* �Ա�������ֵ�����˲� */
	EncoderL.encoder_final = lowpass_filter(&leftSpeedFilt, EncoderL.encoder_original);
	EncoderR.encoder_final = lowpass_filter(&rightSpeedFilt, EncoderR.encoder_original);

	/* �Ա�������ֵ�����쳣���� */
	EncoderL.encoder_final = encoder_debounce(&EncoderDeboL, EncoderR.encoder_final);
	EncoderR.encoder_final = encoder_debounce(&EncoderDeboR, EncoderR.encoder_final);

	/* ȡ���ұ�����ƽ��ֵ */
	g_encoder_average = (EncoderL.encoder_final + EncoderR.encoder_final) / 2;

	/* ��ȡ������ԭʼ���ݲ�����ת��Ϊ�������� */
	imu963ra_get_gyro();
	Gyro_Z = imu963ra_gyro_transition(imu963ra_gyro_z);

	/* ��Gyro_Z���п������˲� */
	filtered_GyroZ = kalman_update(&imu693_kf, Gyro_Z);
	
	/* ת��PID���� */
	turn_pid = pid_poisitional_turnning(&TurnPID, position, filtered_GyroZ);

	/* ���¿������˲�����ֵ */
	kalman_predict(&imu693_kf, turn_pid);

	/* �ٶȻ�PID���� */
	speed_pid = pid_increment(&SpeedPID, g_encoder_average, g_speedpoint);

	/* ���Ƶ�� */
	g_DutyLeft = (int32_t)(speed_pid - turn_pid);
	g_DutyRight = (int32_t)(speed_pid + turn_pid);

	if (protection_flag == 1)
	{
		pid_clean(&SpeedPID);  // ����ٶȻ�PID
		pid_clean(&TurnPID);   // ���ת��PID

		set_motor_pwm(0, 0);
	}
	else
	{
		set_motor_pwm(g_DutyLeft, g_DutyRight);
	}
}


void TM3_Isr() interrupt 19
{
	TIM3_CLEAR_FLAG; //����жϱ�־
	
}

void TM4_Isr() interrupt 20
{
	TIM4_CLEAR_FLAG; //����жϱ�־

}

//void  INT0_Isr()  interrupt 0;
//void  TM0_Isr()   interrupt 1;
//void  INT1_Isr()  interrupt 2;
//void  TM1_Isr()   interrupt 3;
//void  UART1_Isr() interrupt 4;
//void  ADC_Isr()   interrupt 5;
//void  LVD_Isr()   interrupt 6;
//void  PCA_Isr()   interrupt 7;
//void  UART2_Isr() interrupt 8;
//void  SPI_Isr()   interrupt 9;
//void  INT2_Isr()  interrupt 10;
//void  INT3_Isr()  interrupt 11;
//void  TM2_Isr()   interrupt 12;
//void  INT4_Isr()  interrupt 16;
//void  UART3_Isr() interrupt 17;
//void  UART4_Isr() interrupt 18;
//void  TM3_Isr()   interrupt 19;
//void  TM4_Isr()   interrupt 20;
//void  CMP_Isr()   interrupt 21;
//void  I2C_Isr()   interrupt 24;
//void  USB_Isr()   interrupt 25;
//void  PWM1_Isr()  interrupt 26;
//void  PWM2_Isr()  interrupt 27;