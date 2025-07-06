#ifndef __PID_H
#define __PID_H

#include "headfile.h"

typedef struct
{
	float kp;
	float ki;
	float kd;
	
	float error;
	float lasterror;
	float preverror;
	float interror;
	
	float p_out;
	float i_out;
	float d_out;
	float output;
	
	float i_limit;//�����޷�
	float o_limit;//����޷�
} PID_t;

extern PID_t SpeedPID, TurnPID;

void pid_init(PID_t* pid, float kp, float ki, float kd, float i_limit, float o_limit);
void pid_set(PID_t* pid, float kp, float ki, float kd);
void pid_clean(PID_t* pid);

float pid_poisitional(PID_t* pid, float real, float target);
float pid_increment(PID_t* pid, float real, float target);
float pid_poisitional_turnning(PID_t* pid, float position, float GyroZ);

#endif