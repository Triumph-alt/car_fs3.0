///*********************************************************************************************************************
// * COPYRIGHT NOTICE
// * Copyright (c) 2020,逐飞科技
// * All rights reserved.
// * 技术讨论QQ群：一群：179029047(已满)  二群：244861897(已满)  三群：824575535
// *
// * 以下所有内容版权均属逐飞科技所有，未经允许不得用于商业用途，
// * 欢迎各位使用并传播本程序，修改内容时必须保留逐飞科技的版权声明。
// *
// * @file       		isr
// * @company	   		成都逐飞科技有限公司
// * @author     		逐飞科技(QQ790875685)
// * @version    		查看doc内version文件 版本说明
// * @Software 			MDK FOR C251 V5.60
// * @Target core		STC32G12K128
// * @Taobao   			https://seekfree.taobao.com/
// * @date       		2020-4-14
// ********************************************************************************************************************/
#include "headfile.h"
#include "zf_nvic.h"
// #include "zf_exti.h"
#include "zf_uart.h"
#include "zf_tim.h"

uint8_t SPEED_STRAIGHT = 35, SPEED_ISLAND = 35; //速度环目标速度

int g_encoder_average = 0;                       //左右编码器的平均值
float Gyro_Z = 0, filtered_GyroZ = 0;            // 陀螺仪角速度的原始值和卡尔曼滤波之后的值
int32_t g_DutyLeft = 0, g_DutyRight = 0;         // 最后真正要给电机的PWM值

//pid控制相关变量
float speed_pid = 0, turn_pid = 0;               //速度环和转向环pid的值
int g_speedpoint = 0;
int g_leftpoint = 0, g_rightpoint = 0;           //左右轮的目标速度
volatile int16_t positionReal = 0; 

// 蜂鸣器控制相关变量
uint8_t beep_flag = 0;                           // 蜂鸣器开启标志，1表示开启
uint16_t beep_count = 0;                         // 蜂鸣器计时计数器
uint8_t track_ten_cnt = 0;                       //出入环重复判定计时器
uint16_t outisland_cnt = 0;                      //出入环岛重复判定计时器

volatile uint8_t intoisland_pos = 92;            //入环岛的偏差
volatile uint16_t intoisland_str_dist = 10200;   //入环岛直走距离
volatile uint16_t intoisland_all_dist = 12800;   //入环岛总距离

volatile uint8_t outisland_pos = 60;             //出环岛的偏差
volatile uint16_t outisland_turn_dist = 5700;    //出环岛拐弯距离
volatile uint16_t outisland_all_dist = 7500;     //出环岛总距离

int count = 0, flag = 0;

//UART1中断
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
//        //程序自动下载
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

//UART2中断
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
		
		//接收数据寄存器为：S2BUF

	}
}


//UART3中断
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
		
		//接收数据寄存器为：S3BUF

	}
}


//UART4中断
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
		
		//接收数据寄存器为：S4BUF;
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


// void INT2_Isr() interrupt 10
// {
// 	INT2_CLEAR_FLAG;  //清除中断标志
// }


// void INT3_Isr() interrupt 11
// {
// 	INT3_CLEAR_FLAG;  //清除中断标志
// }

// void INT4_Isr() interrupt 16
// {
// 	INT4_CLEAR_FLAG;  //清除中断标志
// }

void TM0_Isr() interrupt 1
{

}


/* 10ms */
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
					key[i].key_time = 0;
				}
			}
			break;
			
			case 1:
			{
				if (key[i].state == 0)
				{
					key[i].step = 2;
				}
				else
				{
					key[i].step = 0;
				}
			}
			break;
			
			case 2:
			{
				if (key[i].state == 1)//松开
				{
					key[i].step = 0;
					
					if (key[i].key_time <= 80)//短按
					{
						key[i].short_flag = 1;
					}
				}
				else
				{
					key[i].key_time++;
					
					if (key[i].key_time > 80)//长按
					{
						key[i].long_flag = 1;
					}
				}
			}
			break;
		}
	}
	
	/* 普通定时功能，备用 */
	count++;
	if (count >= 50)
	{
		flag = 1;
		count = 0;
	}

	/* 检测赛道类型变化并控制蜂鸣器 */
    if (track_type != track_type_last)
    {
        // 赛道类型发生变化，启动蜂鸣器
        beep_flag = 1;
        beep_count = 0;  // 重置计数器
		P26 = 0;         // 打开蜂鸣器
        
        // 更新上一次赛道类型
        track_type_last = track_type;
    }
    
    /* 蜂鸣器计时控制 */
    if (beep_flag)
    {
        beep_count++;
        // 10ms * 20 = 200ms
        if (beep_count >= 10)
        {
            beep_count = 0;
            beep_flag = 0;
            P26 = 1;  // 关闭蜂鸣器
        }
    }

	/* 出环辅助判定，备用 */
    // if (track_route_status == 3)
    // {
	// 	P26 = 0;
    //     beep_count++;
    //     // 10ms * 20 = 200ms
    //     if (beep_count >= 10)
    //     {
    //         beep_count = 0;
    //         P26 = 1;  // 关闭蜂鸣器
	// 		track_route_status = 2;
    //     }
    // }

	/* 出入十字圆环计时判定 */
	// if (ten_ch_flag == 1)
	// {
	// 	track_ten_cnt++;
	// 	if (track_ten_cnt >= 150)
	// 	{
	// 		track_ten_flag = 1;
	// 		track_ten_cnt = 0;
	// 		ten_ch_flag = 0;
	// 	}
	// }

	/* 出入环岛计时判定 */
	if (island_ch_flag == 1)
	{
		outisland_cnt++;
		if (outisland_cnt >= 500)
		{
			track_island_flag = 1;
			outisland_cnt = 0;
			island_ch_flag = 0;
		}
	}

}


void TM2_Isr() interrupt 12
{
	TIM2_CLEAR_FLAG;  //清除中断标志
	
	/* 初步读取并清除编码器的值 */
	EncoderL.encoder_original = get_left_encoder();
	EncoderR.encoder_original = get_right_encoder();

	/* 对编码器的值进行滤波 */
	EncoderL.encoder_final = lowpass_filter(&leftSpeedFilt, EncoderL.encoder_original);
	EncoderR.encoder_final = lowpass_filter(&rightSpeedFilt, EncoderR.encoder_original);

	/* 对编码器的值进行异常消除 */
	EncoderL.encoder_final = encoder_debounce(&EncoderDeboL, EncoderL.encoder_final);
	EncoderR.encoder_final = encoder_debounce(&EncoderDeboR, EncoderR.encoder_final);

	/* 取左右编码器平均值 */
	g_encoder_average = (EncoderL.encoder_final + EncoderR.encoder_final) / 2;

	/* 读取陀螺仪原始数据并将其转化为物理数据 */
	imu963ra_get_gyro();
	Gyro_Z = imu963ra_gyro_transition(imu963ra_gyro_z);
	
	SpeedPID.kp = speed_kp;
	SpeedPID.ki = speed_ki;
	TurnPID.kp = turn_kp;
	TurnPID.kd = turn_kd;
	
	if (track_type == 0)//普通直线
	{
		g_speedpoint = (int)SPEED_STRAIGHT;
		positionReal = position;
	}
	else if (track_type == 1)//直角
	{
		g_speedpoint = (int)SPEED_STRAIGHT;
		
		positionReal = position;
		
		TurnPID.kp = angle_kp;
		TurnPID.kd = angle_kd;
		
		//			if (track_type_zj == 1)//左   写死效果不好
//			{
//				positionReal = 80;
//			}
//			else if (track_type_zj == 2)//右
//			{
//				positionReal = -80;
//			}
	}
	else if (track_type == 3 && track_route_status == 1)//圆环入环
	{
		g_speedpoint = (int)SPEED_ISLAND;
		g_intencoderALL += g_encoder_average;
		
		if(g_intencoderALL <= intoisland_str_dist)//第一阶段先直行
		{
			positionReal = 0;
		}
		else//进入第二阶段打死进环
		{
			if (track_route == 1)//左环
			{
				positionReal = intoisland_pos;
			}
			else if (track_route == 2)//右环
			{
				positionReal = -intoisland_pos;
			}
						
			if (g_intencoderALL >= intoisland_all_dist)//入环完毕
			{
				track_route_status = 2;
				g_intencoderALL = 0;
			}
		}
	}
	else if (track_type == 3 && track_route_status == 2)//环岛内部
	{
		g_speedpoint = (int)SPEED_ISLAND;
		positionReal = position;
	}
	else if (track_type == 3 && track_route_status == 3)//圆环出环
	{
		g_speedpoint = (int)SPEED_ISLAND;
		g_intencoderALL += g_encoder_average;
		
		if (g_intencoderALL <= outisland_turn_dist)//第一阶段打死出环
		{
			if (track_route == 1)//左环
			{
				positionReal = outisland_pos;
			}
			else if (track_route == 2)//右环
			{
				positionReal = -outisland_pos;
			}
		}
		else//第二阶段直走
		{
			positionReal = 0;
			
			if (g_intencoderALL >= outisland_all_dist)//出环完毕
			{
				track_type = 0;
				track_route = 0;
				track_route_status = 0;
				
				g_intencoderALL = 0;
			}
		}
	}
	
	if (startKeyFlag == 1)
	{
		/* 对Gyro_Z进行卡尔曼滤波 */
		filtered_GyroZ = kalman_update(&imu693_kf, Gyro_Z);
		
		/* 转向环PID控制 */
		turn_pid = pid_poisitional_turnning(&TurnPID, positionReal, filtered_GyroZ);

		/* 更新卡尔曼滤波器的值 */
		kalman_predict(&imu693_kf, turn_pid);

		/* 速度环PID控制 */
		speed_pid = pid_increment(&SpeedPID, g_encoder_average, g_speedpoint);

		/* 控制电机 */
		g_DutyLeft = (int32_t)(speed_pid - turn_pid);
		g_DutyRight = (int32_t)(speed_pid + turn_pid);

		if (protection_flag == 1)
		{
			pid_clean(&SpeedPID);  // 清除速度环PID
			pid_clean(&TurnPID);   // 清除转向环PID

			set_motor_pwm(0, 0);
		}
		else
		{
			set_motor_pwm(g_DutyLeft, g_DutyRight);
		}
	}
}


void TM3_Isr() interrupt 19
{
	TIM3_CLEAR_FLAG; //清除中断标志
	
}

void TM4_Isr() interrupt 20
{
	TIM4_CLEAR_FLAG; //清除中断标志

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