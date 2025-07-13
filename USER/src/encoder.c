#include "encoder.h"

#define ENCODER_JUMP_THRESHOLD    20  // 编码器读数突变的阈值（略大于正常的最大值）
#define MIN_STABILITY_COUNT       5   // 判定为稳定状态所需的最小连续计数
#define COUNTER_SAFETY_RESET   10000  // 计数器防溢出重置值

Encoder_t EncoderL, EncoderR;
EncoderDebo_t EncoderDeboL, EncoderDeboR;


void encoder_init(void)
{
    EncoderL.encoder_original = 0;
    EncoderL.encoder_integral = 0;
    EncoderL.encoder_final = 0;

    EncoderR.encoder_integral = 0;
    EncoderR.encoder_original = 0;
    EncoderR.encoder_final = 0;
	
	EncoderDeboL.encoderlast = 0;
	EncoderDeboL.count = 0;
	
	EncoderDeboR.encoderlast = 0;
	EncoderDeboR.count = 0;
	
	ctimer_count_init(CTIM0_P34);
	ctimer_count_init(CTIM3_P04);
}

int get_left_encoder(void)
{
	int encoder_left;
	
	if(LEFT_DIR == 1)
	{
		encoder_left = ctimer_count_read(CTIM0_P34);
	}
	else
	{
		encoder_left = ctimer_count_read(CTIM0_P34) * -1;
	}
	
	ctimer_count_clean(CTIM0_P34);
	
	return encoder_left;
}

int get_right_encoder(void)
{
	int encoder_right;
	
	if(RIGHT_DIR == 1)
	{
		encoder_right = ctimer_count_read(CTIM3_P04) * -1;
	}
	else
	{
		encoder_right = ctimer_count_read(CTIM3_P04);
	}
	
	ctimer_count_clean(CTIM3_P04);
	
	return encoder_right;
}

int encoder_debounce(EncoderDebo_t* instance, int encoder)
{
	// 使用宏替换魔法数，代码意图一目了然
	if (myabs(encoder - instance->encoderlast) > ENCODER_JUMP_THRESHOLD && instance->count >= MIN_STABILITY_COUNT)
	{
		encoder = instance->encoderlast;
		instance->count = 0;
	}
	else
	{
		instance->encoderlast = encoder;
		
		instance->count++;
		if (instance->count >= COUNTER_SAFETY_RESET)
		{
			// 可以保持清零，或者设置为一个不会立即触发抖动判断的值
			instance->count = MIN_STABILITY_COUNT; 
		}
	}
	
	return encoder;
}