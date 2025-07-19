#ifndef __TRANSFER_H
#define __TRANSFER_H

#include "headfile.h"

typedef struct
{
	uint8 step;
	uint8 state;
	uint8 short_flag;
	uint8 long_flag;
	uint16_t key_time;
} Key_t;

enum state//小车运行状态
{
	CHARGE,       //无线充电中
	ELECT_PARA,   //调节电磁
	PID_PARA,     //调节PID参数
	ISLAND_PARA,  //调节环岛参数
	STRAIGHT,     //直走
	RUNNING       //运行
};

extern Key_t key[4];
extern enum state car_state, prev_state; 


extern uint8_t startKeyFlag, uartSendFlag;


void key_task(void);
void display_task(void);

#endif
