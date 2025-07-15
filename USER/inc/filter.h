#ifndef __FILTER_H
#define __FILTER_H

#include "headfile.h"

//---------------------------------------卡尔曼滤波--------------------------------------------------------

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

// extern const float imu693kf_Q;
// extern const float imu693kf_R;
// extern KalmanFilter imu693_kf;

void kalman_init(KalmanFilter* kf, float F, float B, float Q, float R, float initial_x);
void kalman_predict(KalmanFilter* kf, float u);
float kalman_update(KalmanFilter* kf, float z);

void lowpass_init(LowPassFilter* instance, float alpha);
float lowpass_filter(LowPassFilter* instance, float input);

//---------------------------------------定点数低通滤波--------------------------------------------------------
// alpha、output、last_output 均为定点数表示（放大 FX_SCALE 倍）
typedef struct {
    int32_t alpha;          // 滤波系数 α (0~FX_SCALE)
    int32_t output;         // 当前输出 y(n)
    int32_t last_output;    // 上一次输出 y(n-1)
} FixedLowPassFilter;

extern FixedLowPassFilter leftSpeedFilt, rightSpeedFilt, gyro_z_filt; // 定点数低通滤波器

void fixed_lowpass_init(FixedLowPassFilter* instance, int32_t alpha_fixed);
int32_t fixed_lowpass_filter(FixedLowPassFilter* instance, int32_t input_fixed);
void encoder_lowpass_init(FixedLowPassFilter* instance, float alpha_float);
int32_t encoder_lowpass_filter(FixedLowPassFilter* instance, int32_t input_int);


#endif