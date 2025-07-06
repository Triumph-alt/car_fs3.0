#include "filter.h"

const float imu693kf_Q = 0.17;
const float imu693kf_R = 0.20;

KalmanFilter imu693_kf;

LowPassFilter leftSpeedFilt;
LowPassFilter rightSpeedFilt;

void kalman_init(KalmanFilter* kf, float F, float B, float Q, float R, float initial_x)
{
    kf->F = F;
    kf->B = B;
    kf->Q = Q;
    kf->R = R;
    kf->P = 1.0;       // ��ʼЭ����
    kf->x = initial_x;
}

void kalman_predict(KalmanFilter* kf, float u)
{
    kf->x = kf->F * kf->x + kf->B * u;
    kf->P = kf->F * kf->P * kf->F + kf->Q;
}

float kalman_update(KalmanFilter* kf, float z)
{
    kf->K = kf->P / (kf->P + kf->R);
    kf->x += kf->K * (z - kf->x);
    kf->P *= (1 - kf->K);
    return kf->x;
}

void lowpass_init(LowPassFilter* instance, float alpha)
{
	instance->alpha = alpha;
	instance->output = 0;
	instance->lastoutput = 0;
}

float lowpass_filter(LowPassFilter* instance, float input) 
{
    // ʵ��һ�׵�ͨ�˲��ĺ����㷨
    instance->output = instance->alpha * input + (1 - instance->alpha) * instance->lastoutput;
    
    // ������һʱ�̵�����ֵ
    instance->lastoutput = instance->output;
    
    return instance->output;
}