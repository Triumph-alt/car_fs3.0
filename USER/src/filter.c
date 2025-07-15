#include "filter.h"
#include "fixed_point.h"

// const float imu693kf_Q = 0.17;
// const float imu693kf_R = 0.20;

// KalmanFilter imu693_kf;

FixedLowPassFilter leftSpeedFilt, rightSpeedFilt, gyro_z_filt; // 定点数低通滤波器

//---------------------------------------卡尔曼滤波--------------------------------------------------------
void kalman_init(KalmanFilter* kf, float F, float B, float Q, float R, float initial_x)
{
    kf->F = F;
    kf->B = B;
    kf->Q = Q;
    kf->R = R;
    kf->P = 1.0;       // 初始协方差
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

//---------------------------------------浮点数低通滤波--------------------------------------------------------

void lowpass_init(LowPassFilter* instance, float alpha)
{
	instance->alpha = alpha;
	instance->output = 0;
	instance->lastoutput = 0;
}

float lowpass_filter(LowPassFilter* instance, float input) 
{
    // 实现一阶低通滤波的核心算法
    instance->output = instance->alpha * input + (1 - instance->alpha) * instance->lastoutput;
    
    // 更新上一时刻的输入值
    instance->lastoutput = instance->output;
    
    return instance->output;
}

//---------------------------------------定点数低通滤波--------------------------------------------------------
/**
 * @brief  初始化定点数一阶低通滤波器
 * @param  instance      滤波器实例指针
 * @param  alpha_fixed   滤波系数α，范围 0~FX_SCALE (放大 FX_SCALE 倍的整数)
 *                       值越小，滤波越强；值越大，输出更跟随输入
 */
void fixed_lowpass_init(FixedLowPassFilter* instance, int32_t alpha_fixed)
{
    // 变量声明区
    // alpha_fixed 应在 0~FX_SCALE 之间
    if(alpha_fixed < 0) {
        alpha_fixed = 0;
    } else if(alpha_fixed > FX_SCALE) {
        alpha_fixed = FX_SCALE;
    }
    instance->alpha = alpha_fixed;
    instance->output = 0;
    instance->last_output = 0;
}

/**
 * @brief  定点数一阶低通滤波核心函数
 * @param  instance      已初始化的滤波器实例
 * @param  input_fixed   当前输入值 (已放大 FX_SCALE 倍的定点数)
 * @return int32_t       滤波后输出值 (定点数)
 *
 * y(n) = α·x(n) + (1-α)·y(n-1)
 */
int32_t fixed_lowpass_filter(FixedLowPassFilter* instance, int32_t input_fixed)
{
    // 变量声明区
    int32_t temp_output = 0;

    // y(n) = α·x(n) + (1-α)·y(n-1)
    temp_output = (int32_t)((instance->alpha * input_fixed + (FX_SCALE - instance->alpha) * instance->last_output ) / FX_SCALE);

    instance->output = temp_output;
    instance->last_output = temp_output;

    return temp_output;
}

//---------------------------------------编码器定点数低通滤波--------------------------------------------------------

/**
 * @brief  编码器速度定点低通滤波器初始化 (浮点 α 转定点)
 * @param  instance     定点滤波器实例指针
 * @param  alpha_float  滤波系数 α (0~1 浮点)
 */
void encoder_lowpass_init(FixedLowPassFilter* instance, float alpha_float)
{
    // 将浮点 alpha 转为定点
    int32_t alpha_fixed = FLOAT_TO_FIXED(alpha_float);
    fixed_lowpass_init(instance, alpha_fixed);
}

/**
 * @brief  编码器速度定点低通滤波函数
 * @param  instance     已初始化的定点滤波器实例
 * @param  input_int    编码器原始速度值 (普通整数)
 * @return int32_t      滤波后速度值 (普通整数)
 */
int32_t encoder_lowpass_filter(FixedLowPassFilter* instance, int32_t input_int)
{
    int32_t input_fixed = INT_TO_FIXED(input_int);
    int32_t output_fixed = fixed_lowpass_filter(instance, input_fixed);
    return output_fixed / FX_SCALE;   // 转回普通整数速度值
}

