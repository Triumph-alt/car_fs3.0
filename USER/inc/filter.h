#ifndef __FILTER_H
#define __FILTER_H

#include "headfile.h"

typedef struct {
    float F;        // ״̬ת��ϵ����ϵͳ����ѧ��
    float B;        // ��������ϵ��
    float Q;        // ��������Э����
    float R;        // ��������Э����
    float P;        // �������Э����
    float K;        // ����������
    float x;        // ״̬����ֵ��gyroz��
} KalmanFilter;

typedef struct {
	float alpha;
	float output;
	float lastoutput;
} LowPassFilter;

extern const float imu693kf_Q;
extern const float imu693kf_R;

extern KalmanFilter imu693_kf;

extern LowPassFilter leftSpeedFilt;
extern LowPassFilter rightSpeedFilt;

void kalman_init(KalmanFilter* kf, float F, float B, float Q, float R, float initial_x);
void kalman_predict(KalmanFilter* kf, float u);
float kalman_update(KalmanFilter* kf, float z);

void lowpass_init(LowPassFilter* instance, float alpha);
float lowpass_filter(LowPassFilter* instance, float input);


#endif