/*********************************************************************************************************************
 * COPYRIGHT NOTICE
 * Copyright (c) 2020,逐飞科技
 * All rights reserved.
 * 技术讨论QQ群：一群：179029047(已满)  二群：244861897(已满)  三群：824575535
 *
 * 以下所有内容版权均属逐飞科技所有，未经允许不得用于商业用途，
 * 欢迎各位使用并传播本程序，修改内容时必须保留逐飞科技的版权声明。
 *
 * @file       		isr
 * @company	   		成都逐飞科技有限公司
 * @author     		逐飞科技(QQ790875685)
 * @version    		查看doc内version文件 版本说明
 * @Software 		MDK FOR C251 V5.60
 * @Target core		STC32G12K128
 * @Taobao   		https://seekfree.taobao.com/
 * @date       		2020-4-14
 ********************************************************************************************************************/

#ifndef __ISR_H_
#define __ISR_H_

#define SPEED_STRAIGHT     35
#define SPEED_ANGLE        35
#define SPEED_ISLAND       35

extern int g_encoder_average;
extern int32_t g_DutyLeft, g_DutyRight;
extern int g_speedpoint;
extern float Gyro_Z, filtered_GyroZ;

extern float speed_pid;
extern float turn_pid;

extern volatile uint8_t intoisland_pos;    
extern volatile uint16_t intoisland_str_dist;  
extern volatile uint16_t intoisland_all_dist;

extern volatile uint8_t outisland_pos;
extern volatile uint16_t outisland_turn_dist;
extern volatile uint16_t outisland_all_dist;

extern int flag;

extern volatile int16_t positionReal; 


#endif