#ifndef __TRANSFER_H
#define __TRANSFER_H

#include "headfile.h"

typedef struct
{
	uint8 step;
	uint8 state;
	uint8 flag;
} Key_t;

enum state//小车运行状态
{
	ELECT_PARA,   //调节电磁
	PID_PARA,     //调节PID参数
	CHARGE,       //无线充电中
	RUNNING       //运行
};

extern Key_t key[4];
extern enum state car_state; 

void key_task(void);
void display_task(void);

#endif
