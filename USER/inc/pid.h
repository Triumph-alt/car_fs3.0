#ifndef __PID_H
#define __PID_H

#include "headfile.h"
#include "fixed_point.h"

typedef struct
{
	int32_t kp;   // 定点数参数 (放大 FX_SCALE 倍)
	int32_t ki;
	int32_t kd;
	
	int32_t error;
	int32_t lasterror;
	int32_t preverror;
	int32_t interror;
	
	int32_t p_out;
	int32_t i_out;
	int32_t d_out;
	int32_t output;
	
	int32_t i_limit; // 积分限幅 (定点数)
	int32_t o_limit; // 输出限幅 (定点数)
} PID_t;

extern PID_t SpeedPID, TurnPID;

void pid_init(PID_t* pid, float kp, float ki, float kd, float i_limit, float o_limit);
void pid_set(PID_t* pid, float kp, float ki, float kd);
void pid_clean(PID_t* pid);

int32_t pid_poisitional(PID_t* pid, int32_t real, int32_t target);
int32_t pid_increment(PID_t* pid, int32_t real, int32_t target);
int32_t pid_positional_turning(PID_t* pid, int32_t position, int32_t GyroZ);
int32_t pid_positional_turning_fixed(PID_t* pid, int32_t position, int32_t gyro_z_fixed);

#endif

