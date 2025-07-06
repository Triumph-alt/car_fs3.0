#ifndef __TRANSFER_H
#define __TRANSFER_H

#include "headfile.h"

typedef struct
{
	uint8 step;
	uint8 state;
	uint8 flag;
} Key_t;

enum state//С������״̬
{
	ELECT_PARA,   //���ڵ��
	PID_PARA,     //����PID����
	CHARGE,       //���߳����
	RUNNING       //����
};

extern Key_t key[4];
extern enum state car_state; 

void key_task(void);
void display_task(void);

#endif
