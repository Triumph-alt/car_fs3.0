#ifndef __FILTER_H
#define __FILTER_H

#include "headfile.h"

typedef struct {
    float F;        // 状态转移系数（系统动力学）
    float B;        // 控制输入系数
    float Q;        // 过程噪声协方差
    float R;        // 测量噪声协方差
    float P;        // 估计误差协方差
    float K;        // 卡尔曼增益
    float x;        // 状态估计值（gyroz）
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