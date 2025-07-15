#include "pid.h"
#include "fixed_point.h"

PID_t SpeedPID; //速度PID
PID_t TurnPID;  //位置PID

void pid_init(PID_t* pid, float kp, float ki, float kd, float i_limit, float o_limit)
{
	// 参数转换为定点并保存
	pid->kp = FLOAT_TO_FIXED(kp);
	pid->ki = FLOAT_TO_FIXED(ki);
	pid->kd = FLOAT_TO_FIXED(kd);

	pid->i_limit = FLOAT_TO_FIXED(i_limit);
	pid->o_limit = FLOAT_TO_FIXED(o_limit);

	// 状态变量清零
	pid->error = 0;
	pid->lasterror = 0;
	pid->preverror = 0;
	pid->interror = 0;

	pid->p_out = 0;
	pid->i_out = 0;
	pid->d_out = 0;
	pid->output = 0;
}

// 位置式 PID（速度环等） -- 定点实现
int32_t pid_poisitional(PID_t* pid, int32_t real, int32_t target)
{
	// 误差与积分
	pid->error = target - real;
	pid->interror += pid->error;

	// 积分限幅
	if (pid->interror > pid->i_limit)
		pid->interror = pid->i_limit;
	else if (pid->interror < -pid->i_limit)
		pid->interror = -pid->i_limit;

	// P I D 计算
	pid->p_out = FIXED_MUL(pid->kp, pid->error);
	pid->i_out = FIXED_MUL(pid->ki, pid->interror);
	pid->d_out = FIXED_MUL(pid->kd, (pid->error - pid->lasterror));

	pid->output = pid->p_out + pid->i_out + pid->d_out;

	pid->lasterror = pid->error;

	// 输出限幅
	if (pid->output > pid->o_limit)
		pid->output = pid->o_limit;
	else if (pid->output < -pid->o_limit)
		pid->output = -pid->o_limit;

	return pid->output;
}

//增量式PID
int32_t pid_increment(PID_t* pid, int32_t real, int32_t target)
{
	int32_t diff = 0;
	
    // 当前误差
    pid->error = target - real;

    // ΔP = Kp * (e(k) - e(k-1))
    pid->p_out = FIXED_MUL(pid->kp, (pid->error - pid->lasterror));

    // ΔI = Ki * e(k)
    pid->i_out = FIXED_MUL(pid->ki, pid->error);

    // ΔD = Kd * (e(k) - 2e(k-1) + e(k-2))
    diff = pid->error - (pid->lasterror << 1) + pid->preverror;
    pid->d_out = FIXED_MUL(pid->kd, diff);

    pid->output += pid->p_out + pid->i_out + pid->d_out;

    // 保存历史误差
    pid->preverror = pid->lasterror;
    pid->lasterror = pid->error;

    // 输出限幅
    if (pid->output > pid->o_limit)
    {
        pid->output = pid->o_limit;
    }
    else if (pid->output < -pid->o_limit)
    {
        pid->output = -pid->o_limit;
    }

    return pid->output;
}

//转向环pid
int32_t pid_positional_turning(PID_t* pid, int32_t position, int32_t GyroZ)
{
    // P = Kp * 位置误差  (此处位置本身就是误差值)
    pid->p_out = FIXED_MUL(pid->kp, position);

    // D = Kd * 角速度  (GyroZ)
    pid->d_out = FIXED_MUL(pid->kd, GyroZ);

    pid->output = pid->p_out + pid->d_out;

    // 输出限幅
    if (pid->output > pid->o_limit)
    {
        pid->output = pid->o_limit;
    }
    else if (pid->output < -pid->o_limit)
    {
        pid->output = -pid->o_limit;
    }

    // 记录上次误差（这里使用位置作为误差量）
    pid->lasterror = position;

    return pid->output;
}

void pid_set(PID_t* pid, float kp, float ki, float kd)
{
	pid->kp = FLOAT_TO_FIXED(kp);
	pid->ki = FLOAT_TO_FIXED(ki);
	pid->kd = FLOAT_TO_FIXED(kd);
}

void pid_clean(PID_t* pid)
{
	pid->lasterror = 0;
	pid->interror = 0;
	pid->preverror = 0;
	pid->output = 0;
}

// 全定点数版本转向环 PID
int32_t pid_positional_turning_fixed(PID_t* pid, int32_t position, int32_t gyro_z_fixed)
{
    // 变量声明区
    int32_t output = 0;

    // 直接使用位置误差作为P项输入
    pid->p_out = FIXED_MUL(pid->kp, position);

    // 使用角速度（已为定点数，单位 °/s * FX_SCALE）作为D项输入
    pid->d_out = FIXED_MUL(pid->kd, gyro_z_fixed);

    // 转向环通常不需要积分项，保持为0
    pid->i_out = 0;

    output = pid->p_out + pid->d_out;

    // 输出限幅
    if (output > pid->o_limit)
    {
        output = pid->o_limit;
    }
    else if (output < -pid->o_limit)
    {
        output = -pid->o_limit;
    }

    pid->output = output;

    // 保存误差（位置即误差）
    pid->lasterror = position;

    return output;
}

