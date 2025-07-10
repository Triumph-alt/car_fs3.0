/*********************************************************************************************************************
 * COPYRIGHT NOTICE
 * Copyright (c) 2020,��ɿƼ�
 * All rights reserved.
 * ��������QQȺ��һȺ��179029047(����)  ��Ⱥ��244861897(����)  ��Ⱥ��824575535
 *
 * �����������ݰ�Ȩ������ɿƼ����У�δ����������������ҵ��;��
 * ��ӭ��λʹ�ò������������޸�����ʱ���뱣����ɿƼ��İ�Ȩ������
 *
 * @file       		common
 * @company	   		�ɶ���ɿƼ����޹�˾
 * @author     		��ɿƼ�(QQ790875685)
 * @version    		�鿴doc��version�ļ� �汾˵��
 * @Software 		MDK FOR C251 V5.60
 * @Target core		STC32G12K128
 * @Taobao   		https://seekfree.taobao.com/
 * @date       		2020-4-14
 ********************************************************************************************************************/

#ifndef __COMMON_H_
#define __COMMON_H_


#include "STC32Gxx.h"
#include <string.h>
#include <stdio.h>
#include "intrins.h"


//������������
typedef unsigned char   u8;     //  8 bits 
typedef unsigned int    u16;    // 16 bits 
typedef unsigned long   u32;    // 32 bits 

typedef unsigned char   uint8  ;	//  8 bits 
typedef unsigned int  	uint16 ;	// 16 bits 
typedef unsigned long  	uint32 ;	// 32 bits 
		
typedef signed char     int8   ;	//  8 bits 
typedef signed int      int16  ;	// 16 bits 
typedef signed long     int32  ;	// 32 bits 
		
typedef volatile int8   vint8  ;	//  8 bits 
typedef volatile int16  vint16 ;	// 16 bits 
typedef volatile int32  vint32 ;	// 32 bits 
														
typedef volatile uint8  vuint8 ;	//  8 bits 
typedef volatile uint16 vuint16;	// 16 bits 
typedef volatile uint32 vuint32;	// 32 bits 

typedef uint8   uint8_t;	        //  8 bits 
typedef uint16  uint16_t;	        // 16 bits 
typedef uint32  uint32_t;	        // 32 bits 

typedef int8   int8_t;	            //  8 bits 
typedef int16  int16_t;          	// 16 bits 
typedef int32  int32_t;	            // 32 bits 

//===================================================

#define	Priority_0			0	//中断优先级为 0 级（最低级）
#define	Priority_1			1	//中断优先级为 1 级（较低级）
#define	Priority_2			2	//中断优先级为 2 级（较高级）
#define	Priority_3			3	//中断优先级为 3 级（最高级）

#define ENABLE		1
#define DISABLE		0

#define SUCCESS		0
#define FAIL		-1





//typedef enum //����ģ��
//{
//    NO_WIRELESS_MODE = 0,   // û������ģ��
//    WIRELESS_SI24R1 = 1,    // ����ת����
//    WIRELESS_CH9141 = 2,    // ����ת����
//    WIRELESS_CH573 = 3,     // CH573ģ��
//	WIRELESS_BLE6A20 = 4,   // BLE6A20����ģ��
//	
//}WIRELESS_TYPE_enum;



//extern WIRELESS_TYPE_enum wireless_type;

//extern void (*wireless_module_uart_handler)(uint8 dat);


#endif
